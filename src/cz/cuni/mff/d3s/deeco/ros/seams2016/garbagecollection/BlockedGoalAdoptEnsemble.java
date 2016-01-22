package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.ArrayList;
import java.util.List;

import cz.cuni.mff.d3s.deeco.annotations.Ensemble;
import cz.cuni.mff.d3s.deeco.annotations.In;
import cz.cuni.mff.d3s.deeco.annotations.InOut;
import cz.cuni.mff.d3s.deeco.annotations.KnowledgeExchange;
import cz.cuni.mff.d3s.deeco.annotations.Membership;
import cz.cuni.mff.d3s.deeco.annotations.PeriodicScheduling;
import cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection.CollectorRobot.State;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;
import cz.cuni.mff.d3s.deeco.timer.CurrentTimeProvider;
import cz.cuni.mff.d3s.jdeeco.position.Position;

@Ensemble
@PeriodicScheduling(period = 3000)
public class BlockedGoalAdoptEnsemble {
	static final double MAX_DISTANCE_M = 3.0;
	static final int BLOCKED_REMOTERECOVERY_THRESHOLD_S = 1;
	static final long REMOTE_RECOVERY_BACKOFF_MS = 10000;

	@Membership
	public static boolean membership(@In("coord.id") String coordId, @In("coord.position") Position coordPosition,
			@In("member.id") String memberId, @In("member.position") Position memberPosition,
			@In("coord.state") State coordState, @In("member.state") State memberState) {
		System.out
				.println(BlockedGoalSwapEnsemble.class.getSimpleName() + " MEMBERSHIP:" + coordId + " -> " + memberId);
		System.out.println("coordState:" + coordState + " memberState: " + memberState + " coordId: " + coordId
				+ " memberId: " + memberId + " dist: " + memberPosition.euclidDistanceTo(coordPosition));
		boolean ret = coordState == State.Blocked && memberState == State.Blocked && !coordId.equals(memberId)
				&& coordPosition.euclidDistanceTo(memberPosition) < MAX_DISTANCE_M;
		System.out.println("membership: " + ret);
		return ret;
	}

	@KnowledgeExchange
	public static void exchange(@In("member.clock") CurrentTimeProvider clock, @In("coord.id") String coordId,
			@In("member.id") String memberId, @In("coord.goal") Position coordGoal,
			@InOut("member.goal") ParamHolder<Position> memberGoal,
			@InOut("member.blockedCounter") ParamHolder<Long> memberBlockedCounter,
			@InOut("member.lastAdoption") ParamHolder<Long> lastAdoption,
			@InOut("member.adoptedGoals") ParamHolder<ArrayList<Position>> memberAdopedGoals,
			@InOut("member.route") ParamHolder<List<Position>> memberRoute) {
		System.err.println(BlockedGoalAdoptEnsemble.class.getSimpleName() + " EXCHANGE");
		System.err.println("memberGoal: " + memberGoal.value + " coordGoal: " + coordGoal);
		// Check if the member is blocked for long enough
		if (memberBlockedCounter.value < BLOCKED_REMOTERECOVERY_THRESHOLD_S) {
			System.err.println(memberId + " Not yet blocked for long enough");
			return;
		}

		// Limit adoption rate
		if (lastAdoption.value != null
				&& (clock.getCurrentMilliseconds() - lastAdoption.value) < REMOTE_RECOVERY_BACKOFF_MS) {
			System.err.println(memberId + " Adopt rate limiter prevents adoption: "
					+ (clock.getCurrentMilliseconds() - lastAdoption.value) + " ms ago");
			return;
		}

		// Adopt coordinator's goal
		System.err.println(memberId + " adopting " + coordGoal + " from " + coordId);
		memberAdopedGoals.value.add(coordGoal);
		memberGoal.value = coordGoal;
		memberRoute.value.add(coordGoal);

		// Reset counters
		memberBlockedCounter.value = 0l;
		lastAdoption.value = clock.getCurrentMilliseconds();
	}
}