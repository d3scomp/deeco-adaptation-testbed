package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection.PositionGenerator.Area;
import cz.cuni.mff.d3s.deeco.runners.DEECoSimulation;
import cz.cuni.mff.d3s.deeco.runtime.DEECoNode;
import cz.cuni.mff.d3s.jdeeco.network.Network;
import cz.cuni.mff.d3s.jdeeco.network.device.SimpleBroadcastDevice;
import cz.cuni.mff.d3s.jdeeco.network.l2.strategy.KnowledgeInsertingStrategy;
import cz.cuni.mff.d3s.jdeeco.position.Position;
import cz.cuni.mff.d3s.jdeeco.position.PositionPlugin;
import cz.cuni.mff.d3s.jdeeco.publishing.DefaultKnowledgePublisher;
import cz.cuni.mff.d3s.jdeeco.ros.Positioning;
import cz.cuni.mff.d3s.jdeeco.ros.sim.ROSSimulation;

/**
 * Example of vehicles traveling across the map
 * 
 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
 *
 */
public class GarbageCollectDemo {
	private static int GARBAGE_PER_ROBOT = 10;
	private static int ROBOTS = 2;
	private static String[] colors = {
			"red",
			"blue",
			"green",
			"black",
			"gray"
	};
	private static PositionPlugin[] positionPlugins = {
			new PositionPlugin(12, 5),
			//new PositionPlugin(17.50, 5),
			new PositionPlugin(12, 13),
			
			new PositionPlugin(12, 13),
			new PositionPlugin(26, 11),
			new PositionPlugin(28, 11),
			new PositionPlugin(9, 11),
			new PositionPlugin(11, 13),
			new PositionPlugin(10, 13),
			new PositionPlugin(9, 13),
	};
	private static PositionGenerator generator = new PositionGenerator(
			new Random(42),
/*			new Area(00.05, 01.05, 11.1, 13.10), // Kitchen*/
			new Area(11.50, 13.50, 1.00, 8.00), // Office1
			new Area(16.50, 18.25, 1.00, 8.00), // Office2
			new Area(02.25, 04.75, 10.50, 13.75), // Corridor left
			new Area(06.25, 14.25, 10.50, 13.75), // Corridor center left
			new Area(15.75, 23.75, 10.50, 13.75), // Corridor center right
			new Area(25.25, 28.75, 10.50, 13.75) // Corridor right);
	);
		
	public static void main(String[] args) throws Exception {
		// Simulation
		ROSSimulation rosSim = new ROSSimulation("192.168.56.101", 11311, "192.168.56.1", "corridor", 0.02, 100);

		// Create main application container
		DEECoSimulation realm = new DEECoSimulation(rosSim.getTimer());
		
		// Configure loop-back networking for all nodes
		realm.addPlugin(Network.class);
		realm.addPlugin(DefaultKnowledgePublisher.class);
		realm.addPlugin(KnowledgeInsertingStrategy.class);
		//realm.addPlugin(BeeClick.class);
		realm.addPlugin(new SimpleBroadcastDevice());
		
		
		PositionMonitor monitor = new PositionMonitor(rosSim.getTimer());
				
		// Add robots
		for(int i = 0; i < ROBOTS; ++i) {
			final String name = "Collector" + i; 
			List<Position> garbage = new LinkedList<>();
			for(int j = 0; j < GARBAGE_PER_ROBOT; ++j) {
				Position pos = generator.getRandomPosition();
				garbage.add(pos);
				monitor.addPosition(pos, name);
			}
			
			Positioning positioning = new Positioning();
			DEECoNode robot = realm.createNode(i, positioning, rosSim.createROSServices(colors[i]), positionPlugins[i]);
			robot.deployComponent(new CollectorRobot(name, positioning, rosSim.getTimer(), garbage, monitor, generator));
			robot.deployEnsemble(BlockedGoalSwapEnsemble.class);
		}
		
		// Simulate for specified time
		realm.start(300_000);
		
		monitor.printStatus();
		
		System.out.println("!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#");
		System.out.println("!@!#!@!#@!@#!@#!@# As we cannot make ROS exit nicely we are now going to terminate the whole JVM !@#!@#!@#!@#!@#!@#");
		System.out.println("!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#");
		System.exit(0);
	}
}
