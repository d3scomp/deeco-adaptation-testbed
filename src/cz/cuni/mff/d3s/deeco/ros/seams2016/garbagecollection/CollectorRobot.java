package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.List;

import cz.cuni.mff.d3s.deeco.annotations.Component;
import cz.cuni.mff.d3s.deeco.annotations.In;
import cz.cuni.mff.d3s.deeco.annotations.InOut;
import cz.cuni.mff.d3s.deeco.annotations.Local;
import cz.cuni.mff.d3s.deeco.annotations.Out;
import cz.cuni.mff.d3s.deeco.annotations.PeriodicScheduling;
import cz.cuni.mff.d3s.deeco.annotations.Process;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;
import cz.cuni.mff.d3s.deeco.timer.CurrentTimeProvider;
import cz.cuni.mff.d3s.jdeeco.position.Position;
import cz.cuni.mff.d3s.jdeeco.ros.Positioning;
import cz.cuni.mff.d3s.jdeeco.ros.datatypes.Orientation;
import cz.cuni.mff.d3s.jdeeco.ros.datatypes.PoseWithCovariance;
import cz.cuni.mff.d3s.jdeeco.ros.datatypes.ROSPosition;

@Component
public class CollectorRobot {
	public static double SAME_POSITION_THRESHOLD = 0.01;
	
	enum State {
		Free, Blocked
	}

	public Position goal;
	public Position exchangeGoal;
	
	@Local
	public Position curGoal;
	
	/**
	 * Id of the vehicle component.
	 */
	public String id;

	public Position position;

	public State state;

	@Local
	public Long blockedCounter;

	@Local
	public List<Position> route;

	@Local
	public Positioning positioning;

	@Local
	public CurrentTimeProvider clock;

	@Local
	public PositionMonitor monitor;

	public CollectorRobot(String id, Positioning positioning, CurrentTimeProvider clock, List<Position> garbage,
			PositionMonitor monitor) {
		this.id = id;
		this.positioning = positioning;
		this.position = new Position(0, 0, 0);
		this.clock = clock;
		this.monitor = monitor;

		state = State.Free;

		// Set waypoints and initial goal
		route = garbage;
		goal = route.get(0);
		exchangeGoal = null;
		
		blockedCounter = 0l;
	}

	@Process
	@PeriodicScheduling(period = 100)
	public static void sense(@Out("position") ParamHolder<Position> position,
			@In("positioning") Positioning positioning) {
		PoseWithCovariance pos = positioning.getPoseWithCovariance();
		if (pos != null) {
			position.value = new Position(pos.position.x, pos.position.y, pos.position.z);
		} else {
			position.value = new Position(0, 0, 0);
		}
	}

/*	@Process
	@PeriodicScheduling(period = 1000)
	public static void reportStatus(@In("id") String id, @In("position") Position position,
			@In("clock") CurrentTimeProvider clock, @In("goalPosition") Position goalPosition,
			@In("goalExchangePosition") Position goalExchangePosition, @In("goalReached") Boolean goalReached,
			@In("goalSet") Boolean goalSet, @In("route") List<Position> route, @In("state") State state) {
		System.out.format("%d: id: %s", clock.getCurrentMilliseconds(), id);
		System.out.format(", pos: %s, state: %s", position.toString(), state);
		System.out.format(", goal: (pos: %s, dist: %f, reached: %s, set: %s, exchange: %s) remaining:%d%n",
				goalPosition.toString(), goalPosition.euclidDistanceTo(position), String.valueOf(goalReached),
				String.valueOf(goalSet), goalExchangePosition != null ? goalExchangePosition.toString() : "none",
				route.size());
	}*/
/*
	@Process
	@PeriodicScheduling(period = 1000)
	public static void autoUnblock(@In("id") String id, @In("clock") CurrentTimeProvider clock,
			@InOut("blockedCounter") ParamHolder<Long> blockedCounter, @InOut("state") ParamHolder<State> state,
			@InOut("goalSet") ParamHolder<Boolean> goalSet, @In("goalPosition") Position goalPosition,
			@In("positioning") Positioning positioning) {
		if (state.value == State.Blocked) {
			blockedCounter.value++;
		} else {
			blockedCounter.value = 0l;
		}

		if (blockedCounter.value > 15) {
			System.out.println(">>>>>>> Auto unblocking: " + id + "<<<<<<");
			state.value = State.Free;

			System.out.format("%d: Id: %s, Setting goal%n", clock.getCurrentMilliseconds(), id);
			positioning.setSimpleGoal(ROSPosition.fromPosition(goalPosition), new Orientation(0, 0, 0, 1));
			goalSet.value = true;
		}
	}*/

	@Process
	@PeriodicScheduling(period = 500)
	public static void setGoal(@In("id") String id, @In("position") Position position, @InOut("route") ParamHolder<List<Position>> route,
			@InOut("goal") ParamHolder<Position> goal,
			@In("clock") CurrentTimeProvider clock, @In("monitor") PositionMonitor monitor) {
		if (goal != null && position.euclidDistanceTo(goal.value) < 1) {
			// Report to monitor
			monitor.reportReached(goal.value, id);

			// Remove reached waypoint
			route.value.remove(goal.value);
		}
		
		if (goal == null || position.euclidDistanceTo(goal.value) < 1) {
			// Try to set new goal
			if (route.value.isEmpty()) {
				System.out.format("%d: Id: %s, No more waypoints to reach%n", clock.getCurrentMilliseconds(), id);
				goal.value = null;
			} else {
				goal.value = route.value.get(0);
			}
		}
	}

	@Process
	@PeriodicScheduling(period = 2000)
	public static void driveRobot(@In("id") String id, @In("position") Position position,
			@In("positioning") Positioning positioning, @In("goal") Position goal, @InOut("curGoal") ParamHolder<Position> curGoal,
			@In("clock") CurrentTimeProvider clock, @InOut("state") ParamHolder<State> state) throws Exception {
		if(goal == null) {
			System.out.format("%d: Id: %s, No goal to set%n", clock.getCurrentMilliseconds(), id);
			return;
		}
		
		// Set goal if not yet set
		if (curGoal.value == null || goal.euclidDistanceTo(curGoal.value) < SAME_POSITION_THRESHOLD || positioning.getMoveBaseResult() == null) {
			System.out.format("%d: Id: %s, Setting goal%n", clock.getCurrentMilliseconds(), id);
			positioning.setSimpleGoal(ROSPosition.fromPosition(goal), new Orientation(0, 0, 0, 1));
			curGoal.value = goal;
		}

		// Process move result
		if (positioning.getMoveBaseResult() != null && goal.euclidDistanceTo(position) < SAME_POSITION_THRESHOLD) {
			switch (positioning.getMoveBaseResult().status) {
			case Succeeded:
				System.out.format("%d: Id: %s, at: %s reached %s%n", clock.getCurrentMilliseconds(), id, position, goal);
				break;
			case Rejected:
				System.out.format("%d: Id: %s, at: %s rejected goal %s%n", clock.getCurrentMilliseconds(), id, position, goal);
				state.value = State.Blocked;
				break;
			case Canceled:
				System.out.format("%d: Id: %s, Goal canceled: %s%n", clock.getCurrentMilliseconds(), id,
						positioning.getMoveBaseResult().toString());
				break;
			default:
				System.out.format("%d: Id: %s, unknown result: %s%n", clock.getCurrentMilliseconds(), id,
						positioning.getMoveBaseResult().toString());
			}
		}
	}
}
