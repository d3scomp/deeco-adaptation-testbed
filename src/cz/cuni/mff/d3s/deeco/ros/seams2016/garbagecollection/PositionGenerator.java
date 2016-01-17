package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import cz.cuni.mff.d3s.jdeeco.position.Position;

public class PositionGenerator {
	static class Area {
		protected double left, right, top, bottom;
		
		Area(double left, double right, double top, double bottom) {
			this.left = left;
			this.right = right;
			this.top = top;
			this.bottom = bottom;
			
			if(left >= right || top >= bottom) {
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
	
	public PositionGenerator(Random random) {
		this.random = random;
	}
	
	public void addArea(Area area) {
		areas.add(area);
	}
	
	public Position getRandomPosition() {
		Area area = getRandomArea();
		double x = boundedRandom(area.left, area.right);
		double y = boundedRandom(area.top, area.bottom);
		return new Position(x, y);
	}
	
	private double boundedRandom(double from, double to) {
		return from + (to - from) * random.nextDouble();
	}
	
	private Area getRandomArea() {
		// Get total surface area
		double totalSurfeace = 0;		
		for(Area a: areas) {
			totalSurfeace += a.getSurfeceArea();
		}
		
		// Get random part of the surface
		double randomSurface = random.nextDouble() * totalSurfeace;
		
		// Get area for random surface
		for(Area a: areas) {
			if(randomSurface < a.getSurfeceArea()) {
				return a;
			}
			randomSurface -= a.getSurfeceArea();
		}
		return areas.listIterator().next();
	}
	
	
}
