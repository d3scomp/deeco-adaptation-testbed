package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.io.Serializable;
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
	static class Goal implements Serializable {
		private static final long serialVersionUID = 1L;
		public Position position;
		public boolean reached;
	}
	
	/**
	 * Id of the vehicle component.
	 */
	public String id;

	public Position position;

	@Local
	public List<Position> route;

	@Local
	public Positioning positioning;

	@Local
	public Goal goal = new Goal();

	@Local
	public CurrentTimeProvider clock;

	public CollectorRobot(String id, Positioning positioning, CurrentTimeProvider clock, List<Position> garbage) {
		this.id = id;
		this.positioning = positioning;
		this.position = new Position(0, 0, 0);
		this.clock = clock;

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
			@In("clock") CurrentTimeProvider clock, @In("goal") Goal goal, @In("route") List<Position> route) {

		System.out.format("%d: Id: %s, pos: %s%n", clock.getCurrentMilliseconds(), id, position.toString());
		System.out.format("%d: Id: %s, goal: %s (dist: %f, reached: %s) remaining:%n", clock.getCurrentMilliseconds(), id,
				goal.position.toString(), goal.position.euclidDistanceTo(position), String.valueOf(goal.reached));
		for (Position p : route) {
			System.out.format("%d: Id: %s, >>> %s (dist: %f)%n", clock.getCurrentMilliseconds(), id, p.toString(), p.euclidDistanceTo(position));
		}
	}

	@Process
	@PeriodicScheduling(period = 500)
	public static void setGoal(@In("id") String id, @InOut("route") ParamHolder<List<Position>> route,
			@InOut("goal") ParamHolder<Goal> goal, @In("clock") CurrentTimeProvider clock) {
		if(goal.value.reached) {
			// Remove reached waypoint
			route.value.remove(goal.value.position);
			
			// Try to set new goal
			if(route.value.isEmpty()) {
				System.out.format("%d: Id: %s, No more waypoints to reach", clock.getCurrentMilliseconds(), id);
			} else {
				goal.value.position = route.value.get(0);
				goal.value.reached = false;
			}
		}
	}

	@Process
	@PeriodicScheduling(period = 2000)
	public static void driveRobot(@In("id") String id, @In("position") Position position,
			@In("positioning") Positioning positioning, @InOut("goal") ParamHolder<Goal> goal, @In("clock") CurrentTimeProvider clock) throws Exception {

		if (positioning.getMoveBaseResult() != null) {
			switch (positioning.getMoveBaseResult().status) {
			case Succeeded:
				goal.value.reached = true;
				System.out.format("%d: Id: %s, at: %s reached %s%n", clock.getCurrentMilliseconds(), id, position,
						goal.value.position);
				
				positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value.position), new Orientation(0, 0, 0, 1));
				break;
			case Rejected:
				positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value.position), new Orientation(0, 0, 0, 1));
				break;
			case Canceled:
				System.out.format("%d: Id: %s, Goal canceled: %s%n", clock.getCurrentMilliseconds(), id,
						positioning.getMoveBaseResult().toString());
			default:
				System.out.format("%d: Id: %s, unknown result: %s%n", clock.getCurrentMilliseconds(), id,
						positioning.getMoveBaseResult().toString());
			}

			System.out.format("%d: Id: %s, result: %s%n", clock.getCurrentMilliseconds(), id,
					positioning.getMoveBaseResult().toString());
		} else {
			positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value.position), new Orientation(0, 0, 0, 1));
		}
	}
}
