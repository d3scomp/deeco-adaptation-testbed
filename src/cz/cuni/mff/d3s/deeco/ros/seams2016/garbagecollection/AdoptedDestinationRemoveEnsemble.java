package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;

import cz.cuni.mff.d3s.deeco.annotations.Ensemble;
import cz.cuni.mff.d3s.deeco.annotations.In;
import cz.cuni.mff.d3s.deeco.annotations.InOut;
import cz.cuni.mff.d3s.deeco.annotations.KnowledgeExchange;
import cz.cuni.mff.d3s.deeco.annotations.Membership;
import cz.cuni.mff.d3s.deeco.annotations.PeriodicScheduling;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;
import cz.cuni.mff.d3s.jdeeco.position.Position;

/**
 * This ensemble removes goals adopted by other robots
 * 
 * Goals adopted by coordinator are removed from members route.
 * 
 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
 *
 */
@Ensemble
@PeriodicScheduling(period = 3000)
public class AdoptedDestinationRemoveEnsemble {
	/**
	 * Ensemble membership
	 * 
	 * Returns true when there is way-point to remove from members route
	 */
	@Membership
	public static boolean membership(@In("coord.id") String coordId, @In("coord.adoptedGoals") ArrayList<Position> coordAdopted,
			@In("member.id") String memberId, @In("member.route") List<Position> memberRoute) {
		System.out.println(AdoptedDestinationRemoveEnsemble.class.getSimpleName() + " MEMBERSHIP:" + coordId + " -> " + memberId);
		// Do not remove goals from ourself
		if(coordId.equals(memberId)) {
			return false;
		}
		
		// If member has goal adopted by coordinator then do exchange
		for(Position mbrPos: memberRoute) {
			for(Position cordPos: coordAdopted)
			if(mbrPos.euclidDistanceTo(cordPos) < CleanerRobot.SAME_POS_THRESH_M) {
				System.out.println(AdoptedDestinationRemoveEnsemble.class.getSimpleName() + " members");
				return true;
			}
		}
		
		return false;
	}

	/**
	 * Knowledge exchange
	 * 
	 * When membership test passes this removes the adopted waypoints
	 */
	@KnowledgeExchange
	public static void exchange(@In("coord.adoptedGoals") ArrayList<Position> coordAdopted, @InOut("member.route") ParamHolder<List<Position>> memberRoute) {
		System.err.println(AdoptedDestinationRemoveEnsemble.class.getSimpleName() + " EXCHANGE");
		
		// If member has goal adopted by coordinator then remove it
		Collection<Position> toRemove = new HashSet<Position>();
		for(Position mbrPos: memberRoute.value) {
			for(Position cordPos: coordAdopted)
			if(mbrPos.euclidDistanceTo(cordPos) < CleanerRobot.SAME_POS_THRESH_M) {
				System.err.println("Removing adopted goal " + mbrPos);
				toRemove.add(mbrPos);
			}
		}
		memberRoute.value.removeAll(toRemove);
	}
}