package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.HashMap;
import java.util.Map;

import cz.cuni.mff.d3s.deeco.timer.CurrentTimeProvider;
import cz.cuni.mff.d3s.jdeeco.position.Position;

public class PositionMonitor {
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

		public void reached(String reachedBy, long reachedAtMs) {
			if(isReached()) {
				System.out.println("Position reached again!!!");
				return;
			}
			this.reachedBy = reachedBy;
			this.reachedAtMs = reachedAtMs;
		}

		@Override
		public String toString() {
			return String.format("[%s, initialOwner: %s, reachedBy: %s, time: %d ms]", position.toString(), initialOwner,
					isReached() ? reachedBy : "none", reachedAtMs);
		}
	}

	private Map<Position, PositionInfo> positions = new HashMap<>();
	private final CurrentTimeProvider clock;

	public PositionMonitor(CurrentTimeProvider clock) {
		this.clock = clock;
	}

	@Override
	public String toString() {
		StringBuilder builder = new StringBuilder();

		for (PositionInfo info : positions.values()) {
			builder.append(info.toString());
			builder.append(", ");
		}

		return builder.toString();
	}

	public void reportReached(Position position, String reachedBy) {
		positions.get(position).reached(reachedBy, clock.getCurrentMilliseconds());
	}

	public void printStatus() {
		System.out.println(">>> Waypoint status:");
		for (PositionInfo info : positions.values()) {
			System.out.println(info.toString());
		}
	}
	
	public void addPosition(Position position, String owner) {
		positions.put(position, new PositionInfo(position, owner));
	}
}
