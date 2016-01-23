package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import cz.cuni.mff.d3s.jdeeco.position.Position;

/**
 * Helper class for generating random positions
 * 
 * This is used to generate random position within the predefined areas. This is needed to set safe/reachable goals to
 * robots.
 * 
 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
 *
 */
public class PositionGenerator {
	/**
	 * Holds information about rectangular area
	 * 
	 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
	 *
	 */
	static class Area {
		protected double left, right, top, bottom;

		Area(double left, double right, double top, double bottom) {
			this.left = left;
			this.right = right;
			this.top = top;
			this.bottom = bottom;

			if (left >= right || top >= bottom) {
				throw new RuntimeException("Invalid area defined: " + this.toString());
			}
		}

		@Override
		public String toString() {
			return String.format("Area: [%f,%f]-[%f,%f]", left, top, right, bottom);
		}

		protected double getSurfeceArea() {
			return Math.abs((right - left) * (bottom - top));
		}
	}

	private List<Area> areas = new LinkedList<>();
	private Random random;

	/**
	 * Constructs position generator
	 * 
	 * @param random
	 *            Random data source
	 * @param areas
	 *            Initial areas used to generate locations from
	 */
	public PositionGenerator(Random random, Area... areas) {
		this.random = random;
		this.areas.addAll(Arrays.asList(areas));
	}

	/**
	 * Adds position source area
	 * 
	 * @param area
	 *            Area to add
	 */
	public void addArea(Area area) {
		areas.add(area);
	}

	/**
	 * Generates random position from known areas
	 * 
	 * Probability of position being from particular area is determined by area size
	 * 
	 * @return Random location
	 */
	public Position getRandomPosition() {
		Area area = getRandomArea();
		double x = boundedRandom(area.left, area.right);
		double y = boundedRandom(area.top, area.bottom);
		return new Position(x, y);
	}

	/**
	 * Returns random number within range
	 * 
	 * @param from
	 *            Low bound
	 * @param to
	 *            Top bound
	 * @return Random number as double
	 */
	private double boundedRandom(double from, double to) {
		return from + (to - from) * random.nextDouble();
	}

	/**
	 * Randomly picks one of defined areas
	 * 
	 * Probability of selecting particular area is determined by area size
	 * 
	 * @return Random area
	 */
	private Area getRandomArea() {
		// Get total surface area
		double totalSurfeace = 0;
		for (Area a : areas) {
			totalSurfeace += a.getSurfeceArea();
		}

		// Get random part of the surface
		double randomSurface = random.nextDouble() * totalSurfeace;

		// Get area for random surface
		for (Area a : areas) {
			if (randomSurface < a.getSurfeceArea()) {
				return a;
			}
			randomSurface -= a.getSurfeceArea();
		}
		return areas.listIterator().next();
	}

}
