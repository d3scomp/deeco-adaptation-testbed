package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import cz.cuni.mff.d3s.deeco.annotations.Ensemble;
import cz.cuni.mff.d3s.deeco.annotations.In;
import cz.cuni.mff.d3s.deeco.annotations.InOut;
import cz.cuni.mff.d3s.deeco.annotations.KnowledgeExchange;
import cz.cuni.mff.d3s.deeco.annotations.Membership;
import cz.cuni.mff.d3s.deeco.annotations.PeriodicScheduling;
import cz.cuni.mff.d3s.deeco.logging.Log;
import cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection.CollectorRobot.State;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;
import cz.cuni.mff.d3s.jdeeco.position.Position;

@Ensemble
@PeriodicScheduling(period = 3000)
public class BlockedGoalSwapEnsemble {
	private static int MAX_DISTANCE_M = 5;

	@Membership
	public static boolean membership(@In("coord.id") String coordId, @In("coord.position") Position coordPosition,
			@In("member.id") String memberId, @In("member.position") Position memberPosition,
			@In("coord.state") State coordState, @In("member.state") State memberState) {
		System.out.println(BlockedGoalSwapEnsemble.class.getName() + " MEMBERSHIP:" + coordId + " " + memberId);
		return coordState == State.Blocked && memberState == State.Blocked && !coordId.equals(memberId)
				&& coordPosition.euclidDistanceTo(memberPosition) < MAX_DISTANCE_M;
	}

	@KnowledgeExchange
	public static void exchange(@In("coord.goalPosition") Position coordGoalPosition,
			@In("coord.goalExchangePosition") Position coordGoalExchangePosition,
			@InOut("member.goalPosition") ParamHolder<Position> memberGoalPosition,
			@InOut("member.goalExchangePosition") ParamHolder<Position> memberGoalExchangePosition,
			@InOut("member.goalSet") ParamHolder<Boolean> memberGoalSet,
			@InOut("member.state") ParamHolder<State> memberState) {
		System.out.println(BlockedGoalSwapEnsemble.class.getName() + " EXCHANGE");
		
		// Stage 1, member is signaling will to adopt coord's goal
		if (memberGoalExchangePosition.value == null) {
			System.out.println("Goal exchange stage 1");
			memberGoalExchangePosition.value = coordGoalPosition;
			return;
		}

		// Stage 2, coordinator is willing to adopt our goal
		if (coordGoalExchangePosition == memberGoalPosition.value) {
			System.out.println("Goal exchange stage 2");
			// Set coord's goal as member's target
			memberGoalExchangePosition.value = null;
			memberGoalPosition.value = coordGoalPosition;
			memberGoalSet.value = false;
			memberState.value = State.Free;
		}
	}
}
