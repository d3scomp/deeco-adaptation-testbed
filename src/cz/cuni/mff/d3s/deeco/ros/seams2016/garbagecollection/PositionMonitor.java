package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Collection;
import java.util.HashSet;

import cz.cuni.mff.d3s.deeco.timer.CurrentTimeProvider;
import cz.cuni.mff.d3s.jdeeco.position.Position;

/**
 * Helper class for collecting statistics about collected garbage.
 * 
 * All garbage locations are added to the instance of this class. Later, when robot thinks that it reached some of the
 * desired locations it reports the event here. If the robot reached the garbage location the event is recorded together
 * with the time and robot identification.
 * 
 * This is used to check how long it took to reach all goals and which goal was reached by which robot. This is
 * important especially when robots are swapping the goals at the runtime
 * 
 * Example output of printStatus method:
 * 
 * [[3.3m, 12.0m, 0.0m], initialOwner: Collector1, reachedBy: Collector3, time: 166500 ms]
 * [[8.5m, 12.0m, 0.0m], initialOwner: Collector0, reachedBy: Collector2, time: 164000 ms]
 * [[20.8m, 11.7m, 0.0m], initialOwner: Collector1, reachedBy: Collector1, time: 51500 ms]
 * 
 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
 *
 */
public class PositionMonitor {
	/**
	 * Info about single position (garbage location)
	 * 
	 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
	 *
	 */
	static class PositionInfo {
		public Position position;
		public String initialOwner;
		public String reachedBy;
		public long reachedAtMs;

		public PositionInfo(Position position, String initalOwner) {
			this.position = position;
			this.initialOwner = initalOwner;
		}

		public boolean isReached() {
			return reachedBy != null;
		}

		/**
		 * Reports position as reached
		 * 
		 * @param reachedBy
		 *            Who reached the position
		 * @param reachedAtMs
		 *            When the position was reached
		 */
		public void reached(String reachedBy, long reachedAtMs) {
			if (isReached()) {
				System.out.println("Position reached again!!!");
				return;
			}
			this.reachedBy = reachedBy;
			this.reachedAtMs = reachedAtMs;
		}

		@Override
		public String toString() {
			return String.format("[%s, initialOwner: %s, reachedBy: %s, time: %d ms]", position.toString(),
					initialOwner, isReached() ? reachedBy : "none", reachedAtMs);
		}
	}

	private Collection<PositionInfo> positions = new HashSet<>();

	/**
	 * Time provider used for time-stamps
	 */
	private final CurrentTimeProvider clock;

	public PositionMonitor(CurrentTimeProvider clock) {
		this.clock = clock;
	}

	/**
	 * Pretty prints the statistics
	 */
	@Override
	public String toString() {
		StringBuilder builder = new StringBuilder();

		for (PositionInfo info : positions) {
			builder.append(info.toString());
			builder.append(", ");
		}

		return builder.toString();
	}

	/**
	 * Reports goal reached by robot
	 * 
	 * @param position
	 *            Reached position
	 * @param reachedBy
	 *            Robot identification
	 */
	public void reportReached(Position position, String reachedBy) {
		for (PositionInfo posInfo : positions) {
			if (posInfo.position.euclidDistanceTo(position) < CollectorRobot.SAME_POS_THRESH_M) {
				posInfo.reached(reachedBy, clock.getCurrentMilliseconds());
			}
		}
	}

	/**
	 * Prints final statistics
	 */
	public void printStatus() {
		System.out.println(">>> Waypoint status:");
		for (PositionInfo info : positions) {
			System.out.println(info.toString());
		}
	}
	
	/**
	 * Writes final statistics to file
	 * 
	 * @throws IOException 
	 */
	public void writeStatsToFile(String filename) throws IOException {
		BufferedWriter out = new BufferedWriter(new FileWriter(filename));
		for (PositionInfo info : positions) {
			out.write(info.initialOwner);
			out.write(" ");
			out.write(info.reachedBy!=null?info.reachedBy:"none");
			out.write(" ");
			out.write(String.valueOf(info.reachedAtMs));
			out.newLine();
		}
		out.close();
	}

	/**
	 * Add garbage location to monitored locations
	 * 
	 * @param position
	 *            Position to monitor
	 * @param owner
	 *            Initially assigned robot
	 */
	public void addPosition(Position position, String owner) {
		positions.add(new PositionInfo(position, owner));
	}
}
