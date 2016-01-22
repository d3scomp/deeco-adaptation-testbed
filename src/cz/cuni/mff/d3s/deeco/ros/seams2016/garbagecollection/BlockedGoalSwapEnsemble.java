package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import cz.cuni.mff.d3s.deeco.annotations.Ensemble;
import cz.cuni.mff.d3s.deeco.annotations.In;
import cz.cuni.mff.d3s.deeco.annotations.InOut;
import cz.cuni.mff.d3s.deeco.annotations.KnowledgeExchange;
import cz.cuni.mff.d3s.deeco.annotations.Membership;
import cz.cuni.mff.d3s.deeco.annotations.PeriodicScheduling;
import cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection.CollectorRobot.State;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;
import cz.cuni.mff.d3s.jdeeco.position.Position;

@Ensemble
@PeriodicScheduling(period = 3000)
public class BlockedGoalSwapEnsemble {
	static final int MAX_DISTANCE_M = 5;
	static final int BLOCKED_REMOTERECOVERY_THRESHOLD_S = 1;
	static final long REMOTE_RECOVERY_BACKOFF = 20;

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
	public static void exchange(@In("coord.id") String coordId, @In("coord.position") Position coordPosition,
			@In("member.id") String memberId, @In("member.position") Position memberPosition,
			@In("coord.goal") Position coordGoal, @In("coord.exchangeGoal") Position coordGoalExchange,
			@InOut("member.goal") ParamHolder<Position> memberGoal,
			@InOut("member.exchangeGoal") ParamHolder<Position> memberGoalExchange,
			@InOut("member.blockedCounter") ParamHolder<Long> memberBlockedCounter,
			@InOut("member.swapRateLimiter") ParamHolder<Long> swapRateLimiter) {
		System.err.println(BlockedGoalSwapEnsemble.class.getSimpleName() + " EXCHANGE");
		System.err.println("memberGoal: " + memberGoal.value + " coordGoal: " + coordGoal + " coordExchange:"
				+ coordGoalExchange + " memberExchange:" + memberGoalExchange.value);
		// Check if the member is blocked for long enough
		if (memberBlockedCounter.value < BLOCKED_REMOTERECOVERY_THRESHOLD_S) {
			System.err.println(memberId + " Not yet blocked for long enough");
			return;
		}

		if (swapRateLimiter.value > 0) {
			swapRateLimiter.value--;
			System.err.println(memberId + " Swap rate limiter prevents swap");
			return;
		}

		// Stage 1, member is signaling will to adopt coord's goal
		if (memberGoalExchange.value == null) {
			System.err.println(memberId + " Goal exchange stage 1");
			memberGoalExchange.value = coordGoal;
			return;
		}

		// Stage 2, coordinator is willing to adopt our goal
		if (coordGoalExchange != null && memberGoal != null
				&& coordGoalExchange.euclidDistanceTo(memberGoal.value) < CollectorRobot.SAME_POS_THRESH_M) {
			System.err.println(memberId + "Goal exchange stage 2");
			// Set coord's goal as member's target
			memberGoalExchange.value = null;
			memberGoal.value = coordGoal;
			memberBlockedCounter.value = 0l;
			swapRateLimiter.value = REMOTE_RECOVERY_BACKOFF;
			return;
		}
	}
}
