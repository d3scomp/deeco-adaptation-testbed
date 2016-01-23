package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

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

/**
 * This class implements Collector robot component. It is used to control single collector robot which collects garbage
 * at predefined location. The robot locally adapts to being stuck and if possible also remotely resolved deadlocks in
 * movements of multiple robots.
 * 
 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
 *
 */
@Component
public class CollectorRobot {
	/**
	 * Deterministic pseudo-random source
	 * 
	 * Used when robot wants to make random decision.
	 */
	static final Random random = new Random(42);

	/**
	 * Maximum distance to goal at which the goal is considered reached
	 */
	public static double REACHED_POS_THRESH_M = 0.5;

	/**
	 * Positions at this or shorter distance are considered equal
	 */
	public static double SAME_POS_THRESH_M = 0.01;

	/**
	 * State of the robot
	 */
	enum State {
		/**
		 * Robot is not blocked, can move at will
		 */
		Free,

		/**
		 * Robot is blocked. Want to move, but can't. It is time for recovery behaviors.
		 */
		Blocked
	}

	/**
	 * When the robot is in blocked state for this time or longer local adaptation mechanisms kick-in.
	 */
	static final int BLOCKED_AUTORECOVERY_THRESHOLD_S = 6;

	/**
	 * How many times the robot needs to be found in the same place in order to declare it blocked.
	 */
	static final long NOCHANGE_POS_THRESH_CNT = 3;

	/**
	 * Current goal of the robot
	 * 
	 * This goal is followed by driveRobot process.
	 */
	public Position goal;

	/**
	 * How many cycles is the robot blocked
	 */
	@Local
	public Long blockedCounter;

	/**
	 * Time-stamp of the last remote adaptation
	 */
	@Local
	public Long lastAdoption;

	/**
	 * Goals adopted from other robots
	 * 
	 * This needs to be non @Local as other robots needs to know and remove such goals.
	 */
	public Collection<Position> adoptedGoals;

	/**
	 * Goal currently set to robots navigation stack
	 */
	@Local
	public Position curGoal;

	/**
	 * Id of the robot component.
	 */
	public String id;

	/**
	 * Current robot's position
	 */
	public Position position;

	/**
	 * Current state of the robot
	 */
	public State state;

	/**
	 * Last position of the robot
	 * 
	 * Used to detect movement
	 */
	@Local
	public Position oldPosition;

	/**
	 * Number of passes when the robot's position has not changed
	 */
	@Local
	public Long noPosChangeCounter;

	/**
	 * List of garbage locations to be cleaned by robot
	 */
	@Local
	public List<Position> route;

	/**
	 * Reference to robot sensors and actuators
	 * 
	 * In this example localization and routing is used
	 */
	@Local
	public Positioning positioning;

	/**
	 * Reference to time provider
	 */
	@Local
	public CurrentTimeProvider clock;

	/**
	 * Reference to monitor object
	 * 
	 * Monitor counts statistics about robot behavior.
	 */
	@Local
	public PositionMonitor monitor;

	/**
	 * Generate random position within reachable space of the map
	 */
	@Local
	public PositionGenerator positionGenerator;

	/**
	 * Constructor sets robots initial knowledge
	 * 
	 * @param id
	 *            Robot's identification
	 * @param positioning
	 *            Reference to sensor and actuator provider
	 * @param clock
	 *            Reference to time source
	 * @param garbage
	 *            Initial list of garbage locations
	 * @param monitor
	 *            Reference to monitoring object
	 * @param generator
	 *            Reference to location generator
	 */
	public CollectorRobot(String id, Positioning positioning, CurrentTimeProvider clock, List<Position> garbage,
			PositionMonitor monitor, PositionGenerator generator) {
		this.id = id;
		this.positioning = positioning;
		this.position = new Position(0, 0, 0);
		this.clock = clock;
		this.monitor = monitor;
		this.positionGenerator = generator;
		this.noPosChangeCounter = 0l;
		this.adoptedGoals = new ArrayList<>();
		this.state = State.Free;
		this.blockedCounter = 0l;

		// Set waypoints and initial goal
		this.route = garbage;
		this.goal = route.get(0);
	}

	/**
	 * Collects information from sensors
	 * 
	 * @param position
	 *            Collected position
	 * @param positioning
	 *            Position sensor access object
	 */
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

	/**
	 * Reports robots status by printing information to console
	 */
	@Process
	@PeriodicScheduling(period = 1000)
	public static void reportStatus(@In("id") String id, @In("position") Position position,
			@In("clock") CurrentTimeProvider clock, @In("goal") Position goal, @In("curGoal") Position curGoal,
			@In("route") List<Position> route, @In("state") State state) {
		System.out.format("%d: id: %s", clock.getCurrentMilliseconds(), id);
		System.out.format(", pos: %s, state: %s", position.toString(), state);
		System.out.format(", goal: (pos: %s, dist: %f, set: %s) remaining:%d%n", goal != null ? goal : "none",
				goal != null ? goal.euclidDistanceTo(position) : -1.0, curGoal, route.size());
	}

	/**
	 * Local adaptation to robot blocking
	 * 
	 * If robot is blocked long enough this tries to set different goal from list of entirely random one.
	 */
	@Process
	@PeriodicScheduling(period = 1000)
	public static void autoUnblock(@In("id") String id, @In("clock") CurrentTimeProvider clock,
			@InOut("blockedCounter") ParamHolder<Long> blockedCounter, @InOut("state") ParamHolder<State> state,
			@InOut("goal") ParamHolder<Position> goal, @In("positionGenerator") PositionGenerator generator,
			@In("route") List<Position> route) {
		// Increase blocked counter
		if (state.value == State.Blocked) {
			blockedCounter.value++;
		} else {
			blockedCounter.value = 0l;
		}

		if (blockedCounter.value > BLOCKED_AUTORECOVERY_THRESHOLD_S) {
			System.out.println(">>>>>>> Auto unblocking robot " + id + "<<<<<<");

			if (random.nextDouble() > (1.0 / (route.size() + 1.0)) && !route.isEmpty()) {
				// Set completely random goal
				goal.value = generator.getRandomPosition();
				System.out.println("Random");
			} else {
				// Set random goal from list
				System.out.println("Another");
				goal.value = route.get(random.nextInt(route.size()));
			}

			System.out.println("New goal: " + goal.value);

			state.value = State.Free;
			blockedCounter.value = 0l;
		}
	}

	/**
	 * Sets current goal according to garbage locations
	 */
	@Process
	@PeriodicScheduling(period = 500)
	public static void setGoal(@In("id") String id, @In("position") Position position,
			@InOut("route") ParamHolder<List<Position>> route, @InOut("goal") ParamHolder<Position> goal,
			@In("clock") CurrentTimeProvider clock, @In("monitor") PositionMonitor monitor) {
		// Report and remove reached position
		List<Position> toRemove = new LinkedList<>();
		for (Position pos : route.value) {
			if (position.euclidDistanceTo(pos) < REACHED_POS_THRESH_M) {
				monitor.reportReached(pos, id);
				toRemove.add(pos);
			}
		}
		route.value.removeAll(toRemove);

		if (goal.value == null || position.euclidDistanceTo(goal.value) < REACHED_POS_THRESH_M) {
			// Try to set new goal
			if (route.value.isEmpty()) {
				System.out.format("%d: Id: %s, No more waypoints to reach%n", clock.getCurrentMilliseconds(), id);
				goal.value = null;
			} else {
				System.out.format("%d: Id: %s, Setting next waypoint as goal%n", clock.getCurrentMilliseconds(), id);
				goal.value = route.value.get(0);
			}
		}
	}

	/**
	 * Sets goal to robot navigation and routing stack
	 */
	@Process
	@PeriodicScheduling(period = 2000)
	public static void driveRobot(@In("id") String id, @In("position") Position pos,
			@In("positioning") Positioning positioning, @In("goal") Position goal,
			@InOut("curGoal") ParamHolder<Position> curGoal, @In("clock") CurrentTimeProvider clock,
			@InOut("state") ParamHolder<State> state, @In("monitor") PositionMonitor monitor) throws Exception {
		if (goal == null) {
			System.out.format("%d: Id: %s, No goal to set%n", clock.getCurrentMilliseconds(), id);
			return;
		}

		// Set goal if not yet set
		if (curGoal.value == null || goal.euclidDistanceTo(curGoal.value) > SAME_POS_THRESH_M
				|| positioning.getMoveBaseResult() == null) {
			System.out.format("%d: Id: %s, Setting goal%n", clock.getCurrentMilliseconds(), id);
			positioning.setSimpleGoal(ROSPosition.fromPosition(goal), new Orientation(0, 0, 0, 1));
			curGoal.value = goal;
		}

		// Process move result
		if (positioning.getMoveBaseResult() != null && goal.euclidDistanceTo(pos) < SAME_POS_THRESH_M) {
			switch (positioning.getMoveBaseResult().status) {
			case Succeeded:
				System.out.format("%d: Id: %s, Goal reached: %s%n", clock.getCurrentMilliseconds(), id, goal);
				monitor.reportReached(pos, id);
				break;
			case Rejected:
				System.out.format("%d: Id: %s, Goal rejected: %s%n", clock.getCurrentMilliseconds(), id, goal);
				break;
			case Canceled:
				System.out.format("%d: Id: %s, Goal canceled: %s%n", clock.getCurrentMilliseconds(), id, goal);
				break;
			default:
				System.out.format("%d: Id: %s, unknown result: %s%n", clock.getCurrentMilliseconds(), id,
						positioning.getMoveBaseResult().toString());
			}
		}
	}

	/**
	 * Detects whenever the robot is blocked by chacking current and last position
	 */
	@Process
	@PeriodicScheduling(period = 1000)
	public static void detectBlocked(@In("id") String id, @In("position") Position pos,
			@InOut("oldPosition") ParamHolder<Position> oldPos,
			@InOut("noPosChangeCounter") ParamHolder<Long> noPosChangeCounter, @InOut("state") ParamHolder<State> state,
			@In("goal") Position goal) {
		// If we have nothing to do just return
		if (goal == null) {
			return;
		}
		// Increment no change counter
		boolean noMove = oldPos.value != null && oldPos.value.euclidDistanceTo(pos) < SAME_POS_THRESH_M;
		boolean wantMove = pos.euclidDistanceTo(goal) > REACHED_POS_THRESH_M;
		if (wantMove && noMove) {
			noPosChangeCounter.value++;
		} else {
			noPosChangeCounter.value = 0l;
		}
		oldPos.value = pos;

		// Adjust blocked
		if (noPosChangeCounter.value > NOCHANGE_POS_THRESH_CNT) {
			state.value = State.Blocked;
		} else {
			state.value = State.Free;
		}
	}
}
