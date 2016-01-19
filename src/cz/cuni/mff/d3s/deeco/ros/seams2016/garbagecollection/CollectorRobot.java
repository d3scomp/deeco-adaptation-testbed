package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.List;

import cz.cuni.mff.d3s.deeco.annotations.Component;
import cz.cuni.mff.d3s.deeco.annotations.In;
import cz.cuni.mff.d3s.deeco.annotations.InOut;
import cz.cuni.mff.d3s.deeco.annotations.Local;
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
	enum State {
		Free, Blocked
	}

	public Position goalPosition;
	public Position goalExchangePosition;
	public Boolean goalReached;
	public Boolean goalSet;

	/**
	 * Id of the vehicle component.
	 */
	public String id;

	public Position position;

	public State state;

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
		goalPosition = route.get(0);
		goalExchangePosition = null;
		goalReached = false;
		goalSet = false;
	}

	@Process
	@PeriodicScheduling(period = 100)
	public static void sense(@InOut("position") ParamHolder<Position> position,
			@In("positioning") Positioning positioning) {
		PoseWithCovariance pos = positioning.getPoseWithCovariance();
		if (pos != null) {
			position.value = new Position(pos.position.x, pos.position.y, pos.position.z);
		}
	}

	@Process
	@PeriodicScheduling(period = 1000)
	public static void reportStatus(@In("id") String id, @In("position") Position position,
			@In("clock") CurrentTimeProvider clock, @In("goalPosition") Position goalPosition,
			@In("goalExchangePosition") Position goalExchangePosition, @In("goalReached") Boolean goalReached,
			@In("goalSet") Boolean goalSet, @In("route") List<Position> route, @In("state") State state) {
		System.out.format("%d: id: %s", clock.getCurrentMilliseconds(), id);
		System.out.format(", pos: %s, state: %s", position.toString(), state);
		System.out.format(", goal: (pos: %s, dist: %f, reached: %s, set: %s, exchange: %s) remaining:%d%n",
				goalPosition.toString(), goalPosition.euclidDistanceTo(position), String.valueOf(goalReached),
				String.valueOf(goalSet), goalExchangePosition != null?goalExchangePosition.toString():"none", route.size());
	}

	@Process
	@PeriodicScheduling(period = 500)
	public static void setGoal(@In("id") String id, @InOut("route") ParamHolder<List<Position>> route,
			@InOut("goalPosition") ParamHolder<Position> goalPosition,
			@InOut("goalReached") ParamHolder<Boolean> goalReached, @InOut("goalSet") ParamHolder<Boolean> goalSet,
			@In("clock") CurrentTimeProvider clock, @In("monitor") PositionMonitor monitor) {
		if (goalReached.value) {
			// Report to monitor
			monitor.reportReached(goalPosition.value, id);

			// Remove reached waypoint
			route.value.remove(goalPosition.value);

			// Try to set new goal
			if (route.value.isEmpty()) {
				System.out.format("%d: Id: %s, No more waypoints to reach%n", clock.getCurrentMilliseconds(), id);
			} else {
				goalPosition.value = route.value.get(0);
				goalReached.value = false;
				goalSet.value = false;
			}
		}
	}

	@Process
	@PeriodicScheduling(period = 2000)
	public static void driveRobot(@In("id") String id, @In("position") Position position,
			@In("positioning") Positioning positioning, @InOut("goalPosition") ParamHolder<Position> goalPosition,
			@InOut("goalReached") ParamHolder<Boolean> goalReached, @InOut("goalSet") ParamHolder<Boolean> goalSet,
			@In("clock") CurrentTimeProvider clock, @InOut("state") ParamHolder<State> state) throws Exception {
		// Set goal if not yet set
		if (!goalSet.value || positioning.getMoveBaseResult() == null) {
			System.out.format("%d: Id: %s, Setting goal%n", clock.getCurrentMilliseconds(), id);
			positioning.setSimpleGoal(ROSPosition.fromPosition(goalPosition.value), new Orientation(0, 0, 0, 1));
			goalSet.value = true;
		}

		// Process move result
		if (positioning.getMoveBaseResult() != null && !goalReached.value) {
			switch (positioning.getMoveBaseResult().status) {
			case Succeeded:
				goalReached.value = true;
				System.out.format("%d: Id: %s, at: %s reached %s%n", clock.getCurrentMilliseconds(), id, position,
						goalPosition.value);
				/*
				 * positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value.position), new Orientation(0, 0, 0,
				 * 1)); goal.value.set = true;
				 */
				break;
			case Rejected:
				System.out.format("%d: Id: %s, at: %s rejected goal %s%n", clock.getCurrentMilliseconds(), id, position,
						goalPosition.value);
				// positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value.position), new Orientation(0, 0, 0,
				// 1));
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
