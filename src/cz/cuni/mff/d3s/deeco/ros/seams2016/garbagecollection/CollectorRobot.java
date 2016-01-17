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
	public Position goal;

	@Local
	public CurrentTimeProvider clock;

	public CollectorRobot(String id, Positioning positioning, CurrentTimeProvider clock, List<Position> garbage) {
		this.id = id;
		this.positioning = positioning;
		this.position = new Position(0, 0, 0);
		this.clock = clock;

		// Set waypoints and initial goal
		route = garbage;
		goal = route.get(0);
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
			@In("clock") CurrentTimeProvider clock, @In("goal") Position goal, @In("route") List<Position> route) {

		System.out.format("%d: Id: %s, pos: %s%n", clock.getCurrentMilliseconds(), id, position.toString());
		System.out.format("%d: Id: %s, goal: %s (dist: %f) remaining:%n", clock.getCurrentMilliseconds(), id, goal.toString(), goal.euclidDistanceTo(position));
		for(Position p: route) {
			System.out.format(">>> %s (dist: %f)%n", p.toString(), p.euclidDistanceTo(position));
		}
	}

	@Process
	@PeriodicScheduling(period = 2000)
	public static void planRouteAndDrive(@In("id") String id, @In("position") Position position,
			@In("positioning") Positioning positioning, @InOut("route") ParamHolder<List<Position>> route,
			@InOut("goal") ParamHolder<Position> goal, @In("clock") CurrentTimeProvider clock) throws Exception {

		if (positioning.getMoveBaseResult() != null) {
			switch (positioning.getMoveBaseResult().status) {
			case Succeeded:
				Position reached = goal.value;
				System.out.format("%d: Id: %s, at: %s reached %s%n", clock.getCurrentMilliseconds(), id, position, reached);
				route.value.remove(reached);
				
				if(route.value.isEmpty()) {
					System.out.println("No more waypoints to reach");
				} else {
					goal.value = route.value.get(0);
					positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value), new Orientation(0, 0, 0, 1));
				}
				break;
			case Rejected:
				positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value), new Orientation(0, 0, 0, 1));
			default:
				System.out.format("%d: Id: %s, unknown result: %s%n", clock.getCurrentMilliseconds(), id, positioning.getMoveBaseResult().toString());
			}

			System.out.format("%d: Id: %s, result: %s%n", clock.getCurrentMilliseconds(), id, positioning.getMoveBaseResult().toString());
		} else {
			positioning.setSimpleGoal(ROSPosition.fromPosition(goal.value), new Orientation(0, 0, 0, 1));
		}
	}
}
