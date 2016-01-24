package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.ArrayList;
import java.util.List;

import cz.cuni.mff.d3s.deeco.annotations.Ensemble;
import cz.cuni.mff.d3s.deeco.annotations.In;
import cz.cuni.mff.d3s.deeco.annotations.InOut;
import cz.cuni.mff.d3s.deeco.annotations.KnowledgeExchange;
import cz.cuni.mff.d3s.deeco.annotations.Membership;
import cz.cuni.mff.d3s.deeco.annotations.PeriodicScheduling;
import cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection.CleanerRobot.State;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;
import cz.cuni.mff.d3s.deeco.timer.CurrentTimeProvider;
import cz.cuni.mff.d3s.jdeeco.position.Position;

/**
 * This ensemble implements adaptation to deadlocks in robot navigation
 * 
 * When two robots are stuck and close enough they are considered to be in deadlock. For example this happens when two
 * robots attempt to pass the narrow corridor in opposite directions. This adaptation mechanism tries to detect such
 * situation and resolve it by swapping the current goals of deadlocked robots. Idea is that when the robots cannot swap
 * themselves in narrow corridor they can still swap their goals.
 * 
 * The implementation is done using ensemble construct. Ensemble is a knowledge exchange mechanism consisting of
 * condition and data exchange. In our case we will detect deadlocked robots in membership condition and implement the
 * goal swapping in data exchange. As the ensemble is evaluated on all nodes independently it is sufficient to code the
 * data exchange in such a way that one (by design always local as ensemble execution modifies always only local data)
 * robot adopts the goal of the other one. The problem with adopted goal remaining in the knowledge of the other robot
 * is solved by another ensemble that just removes way-points adopted by other robots. This technique is used to limit
 * possibility that some goal will be lost.
 * 
 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
 *
 */
@Ensemble
@PeriodicScheduling(period = 3000)
public class DestinationAdoptionEnsemble {
	/**
	 * Maximum distance in between deadlocked robots
	 */
	static final double MAX_DISTANCE_M = 3.0;

	/**
	 * How long to wait before the adaptation takes palce
	 */
	static final int BLOCKED_REMOTERECOVERY_THRESHOLD_S = 1;

	/**
	 * Minimum delay in between adaptations
	 * 
	 * This is important as we do not want deadlocked robots to swap the goals multiple times. As the odd swap would
	 * revert the efect of even onw.
	 */
	static final long REMOTE_RECOVERY_BACKOFF_MS = 10000;

	/**
	 * Ensemble membership
	 * 
	 * This returns true when two different blocked robots are close enough. Informally this checks whenever robots are
	 * in mutual deadlock.
	 */
	@Membership
	public static boolean membership(@In("coord.id") String coordId, @In("coord.position") Position coordPosition,
			@In("member.id") String memberId, @In("member.position") Position memberPosition,
			@In("coord.state") State coordState, @In("member.state") State memberState) {
		return coordState == State.Blocked && memberState == State.Blocked && !coordId.equals(memberId)
				&& coordPosition.euclidDistanceTo(memberPosition) < MAX_DISTANCE_M;
	}

	/**
	 * Data exchange
	 * 
	 * When the membership condition passes this method is executed in order to pass the goal from one robot to another.
	 */
	@KnowledgeExchange
	public static void exchange(@In("member.clock") CurrentTimeProvider clock, @In("coord.id") String coordId,
			@In("member.id") String memberId, @In("coord.destination") Position coordDestination,
			@InOut("member.destination") ParamHolder<Position> memberDestination,
			@InOut("member.blockedCounter") ParamHolder<Long> memberBlockedCounter,
			@InOut("member.lastAdoption") ParamHolder<Long> lastAdoption,
			@InOut("member.adoptedDestinations") ParamHolder<ArrayList<Position>> memberAdopedDestinations,
			@InOut("member.route") ParamHolder<List<Position>> memberRoute) {
		System.err.println(DestinationAdoptionEnsemble.class.getSimpleName() + " EXCHANGE");
		System.err.println("memberDestination: " + memberDestination.value + " coordDestination: " + coordDestination);
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

		// Adopt coordinator's destination
		System.err.println(memberId + " adopting " + coordDestination + " from " + coordId);
		memberAdopedDestinations.value.add(coordDestination);
		memberDestination.value = coordDestination;
		memberRoute.value.add(coordDestination);

		// Reset counters
		memberBlockedCounter.value = 0l;
		lastAdoption.value = clock.getCurrentMilliseconds();
	}
}