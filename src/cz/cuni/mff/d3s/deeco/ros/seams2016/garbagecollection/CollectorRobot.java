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

@Component
public class CollectorRobot {
	public static double REACHED_POS_THRESH_M = 0.5;
	public static double SAME_POS_THRESH_M = 0.01;
	static final Random random = new Random(42);

	enum State {
		Free, Blocked
	}

	enum SelfRecoveryState {
		GoalChange, RandomGoal
	}

	static final int BLOCKED_AUTORECOVERY_THRESHOLD_S = 6;
	static final long NOCHANGE_POS_THRESH_CNT = 3;

	public Position goal;
	public Position exchangeGoal;
	
	public Long blockedCounter;
	
	public Long lastAdoption;
	
	public Collection<Position> adoptedGoals;


	@Local
	public Position curGoal;

	/**
	 * Id of the vehicle component.
	 */
	public String id;

	public Position position;

	public State state;

	@Local
	public Position oldPosition;

	@Local
	public Long noPosChangeCounter;

	public List<Position> route;

	@Local
	public Positioning positioning;

	@Local
	public CurrentTimeProvider clock;

	@Local
	public PositionMonitor monitor;

	@Local
	public PositionGenerator positionGenerator;

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
			System.out.println(">>>>>>> Auto unblocking: " + id + "<<<<<<");

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

	@Process
	@PeriodicScheduling(period = 2000)
	public static void driveRobot(@In("id") String id, @In("position") Position pos,
			@In("positioning") Positioning positioning, @In("goal") Position goal,
			@InOut("curGoal") ParamHolder<Position> curGoal, @In("clock") CurrentTimeProvider clock,
			@InOut("state") ParamHolder<State> state) throws Exception {
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

	@Process
	@PeriodicScheduling(period = 1000)
	public static void driveRobot(@In("id") String id, @In("position") Position pos,
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
