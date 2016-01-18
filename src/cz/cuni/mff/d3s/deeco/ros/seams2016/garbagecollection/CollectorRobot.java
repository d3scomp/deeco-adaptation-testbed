package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.io.Serializable;
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
	static class Goal implements Serializable {
		private static final long serialVersionUID = 1L;
		public Position position;
		public boolean reached;
	}

	enum State {
		Free, Blocked, Recovering
	}

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
	public Goal goal = new Goal();

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
		this.state = State.Free;

		// Set waypoints and initial goal
		route = garbage;
		goal.position = route.get(0);
		goal.reached = false;
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
			@In("clock") CurrentTimeProvider clock, @In("goal") Goal goal, @In("route") List<Position> route,
			@In("state") State state) {
		System.out.format("%d: Id: %s, pos: %s, %s, goal: %s (dist: %f, reached: %s) remaining:%d%n",
				clock.getCurrentMilliseconds(), id, position.toString(), state, goal.position.toString(),
				goal.position.euclidDistanceTo(position), String.valueOf(goal.reached), route.size());
	}

	@Process
	@PeriodicScheduling(period = 500)
	public static void setGoal(@In("id") String id, @InOut("route") ParamHolder<List<Position>> route,
			@InOut("goal") ParamHolder<Goal> goal, @In("clock") CurrentTimeProvider clock,
			@In("monitor") PositionMonitor monitor) {
		if (goal.value.reached) {
			// Report to monitor
			monitor.reportReached(goal.value.position, id);

			// Remove reached waypoint
			route.value.remove(goal.value.position);

			// Try to set new goal
			if (route.value.isEmpty()) {
				System.out.format("%d: Id: %s, No more waypoints to reach%n", clock.getCurrentMilliseconds(), id);
			} else {
				goal.value.position = route.value.get(0);
				goal.value.reached = false;
			}
		}
	}

	@Process
	@PeriodicScheduling(period = 2000)
	public static void driveRobot(@In("id") String id, @In("position") Position position,
			@In("positioning") Positioning positioning, @InOut("goal") ParamHolder<Goal> goal,
			@In("clock") CurrentTimeProvider clock, @Out("state") ParamHolder<State> state) throws Exception {

		if (positioning.getMoveBaseResult() != null) {
			switch (positioning.getMoveBaseResult().status) {
			case Succeeded:
				goal.value.reached = true;
				System.out.format("%d: Id: %s, at: %s reached %s%n", clock.getCurrentMilliseconds(), id, position,
						goal.value.position);

				positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value.position), new Orientation(0, 0, 0, 1));
				break;
			case Rejected:
				System.out.format("%d: Id: %s, at: %s rejected goal %s%n", clock.getCurrentMilliseconds(), id, position,
						goal.value.position);
				// positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value.position), new Orientation(0, 0, 0, 1));
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
		} else {
			System.out.format("%d: Id: %s, Setting initial goal%n", clock.getCurrentMilliseconds(), id);
			positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value.position), new Orientation(0, 0, 0, 1));
		}
	}
}
