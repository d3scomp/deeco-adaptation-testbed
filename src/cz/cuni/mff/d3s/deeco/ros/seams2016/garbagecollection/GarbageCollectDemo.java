package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection.PositionGenerator.Area;
import cz.cuni.mff.d3s.deeco.runners.DEECoSimulation;
import cz.cuni.mff.d3s.deeco.runtime.DEECoNode;
import cz.cuni.mff.d3s.jdeeco.network.Network;
import cz.cuni.mff.d3s.jdeeco.network.l2.strategy.KnowledgeInsertingStrategy;
import cz.cuni.mff.d3s.jdeeco.position.Position;
import cz.cuni.mff.d3s.jdeeco.position.PositionPlugin;
import cz.cuni.mff.d3s.jdeeco.publishing.DefaultKnowledgePublisher;
import cz.cuni.mff.d3s.jdeeco.ros.BeeClick;
import cz.cuni.mff.d3s.jdeeco.ros.Positioning;
import cz.cuni.mff.d3s.jdeeco.ros.sim.ROSSimulation;

/**
 * Example of robots cleaning garbage
 * 
 * This simulation shows ${ROBOTS} robots in "corridor" map cleaning garbage from predefined locations. Each robot is
 * initially assigned ${GARBAGE_PER_ROBOT} garbage locations to visit.
 * 
 * This class defines main method and can be directly launched as java application, but it is necessary to adjust
 * SIMUALTION_SERVER_ADDRESS to math the simulation server which needs to be executed in advice.
 * 
 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
 *
 */
public class GarbageCollectDemo {
	/**
	 * Simulation server address
	 */
	private static final String SIMUALTION_SERVER_ADDRESS = "127.0.0.1";
	
	/**
	 * Number of garbage locations initially assigned to each robot
	 */
	private static int GARBAGE_PER_ROBOT = 10;
	
	/**
	 * Number of robots in the simulation
	 */
	private static int ROBOTS = 4;
	
	/**
	 * Colors of robots in simulation
	 */
	private static String[] colors = { "red", "blue", "green", "black", "gray" };
	
	/**
	 * Initial robot positions
	 */
	private static PositionPlugin[] positionPlugins = {
			new PositionPlugin(12, 5),
			new PositionPlugin(17.50, 5),
			new PositionPlugin(12, 13),
			new PositionPlugin(26, 11),
			new PositionPlugin(28, 11),
			new PositionPlugin(9, 11),
			new PositionPlugin(11, 13),
			new PositionPlugin(10, 13),
			new PositionPlugin(9, 13),
	};
	
	/**
	 * Random position generator
	 */
	private static PositionGenerator generator = new PositionGenerator(new Random(42),
			new Area(11.50, 13.50, 1.00, 7.00), // Office1
			new Area(16.50, 18.25, 1.00, 7.00), // Office2
			new Area(02.25, 04.75, 11.10, 13.75), // Corridor left
			new Area(06.25, 14.25, 11.10, 13.75), // Corridor center left
			new Area(15.75, 23.75, 11.10, 13.75), // Corridor center right
			new Area(25.25, 28.75, 11.10, 13.75) // Corridor right
	);

	public static void main(String[] args) throws Exception {
		// Create ROS based simulation
		// "corridor" is prefix of map files to use in the simulation		
		ROSSimulation rosSim = new ROSSimulation(SIMUALTION_SERVER_ADDRESS, 11311, SIMUALTION_SERVER_ADDRESS,
				"corridor", 0.02, 100);

		// Create main application container
		DEECoSimulation realm = new DEECoSimulation(rosSim.getTimer());

		// Configure loop-back networking for all nodes
		realm.addPlugin(Network.class);
		realm.addPlugin(DefaultKnowledgePublisher.class);
		realm.addPlugin(KnowledgeInsertingStrategy.class);
		realm.addPlugin(BeeClick.class); // Network device simulation using OMNeT++ and INET
		// realm.addPlugin(new SimpleBroadcastDevice()); // Simple fake network device (simulates range and delivery latency)

		PositionMonitor monitor = new PositionMonitor(rosSim.getTimer());

		// Add robots
		for (int i = 0; i < ROBOTS; ++i) {
			// Robot name
			final String name = "Collector" + i;

			// Initial garbage location for robot
			List<Position> garbage = new LinkedList<>();
			for (int j = 0; j < GARBAGE_PER_ROBOT; ++j) {
				Position pos = generator.getRandomPosition();
				garbage.add(pos);
				monitor.addPosition(pos, name);
			}

			// Deploy DEECo node with robot specific plugins
			Positioning positioning = new Positioning();
			DEECoNode robot = realm.createNode(i, positioning, rosSim.createROSServices(colors[i]), positionPlugins[i]);

			// Deploy Collector robot component
			robot.deployComponent(
					new CollectorRobot(name, positioning, rosSim.getTimer(), garbage, monitor, generator));

			// Deploy ensembles
			robot.deployEnsemble(BlockedGoalAdoptEnsemble.class);
			robot.deployEnsemble(AdoptedGoalRemoveEnsemble.class);
		}

		// Simulate for 10 minutes
		realm.start(600_000);

		// print final report on reached garbage locations
		monitor.printStatus();

		/**
		 * Kill the whole JVM as ROS might not exit nicely
		 */
		System.out.println(
				"!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#");
		System.out.println(
				"!@!#!@!#@!@#!@#!@# As we cannot make ROS exit nicely we are now going to terminate the whole JVM !@#!@#!@#!@#!@#!@#");
		System.out.println(
				"!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#!@#!@#!#!@!#!@!#@!@#");
		System.exit(0);
	}
}
