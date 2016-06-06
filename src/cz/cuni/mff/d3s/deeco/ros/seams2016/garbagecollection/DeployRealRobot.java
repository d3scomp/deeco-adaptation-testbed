package cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection;

import java.net.InetAddress;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import cz.cuni.mff.d3s.deeco.ros.seams2016.garbagecollection.PositionGenerator.Area;
import cz.cuni.mff.d3s.deeco.runtime.DEECoNode;
import cz.cuni.mff.d3s.deeco.timer.WallTimeTimer;
import cz.cuni.mff.d3s.jdeeco.network.Network;
import cz.cuni.mff.d3s.jdeeco.network.l2.strategy.KnowledgeInsertingStrategy;
import cz.cuni.mff.d3s.jdeeco.position.Position;
import cz.cuni.mff.d3s.jdeeco.position.PositionPlugin;
import cz.cuni.mff.d3s.jdeeco.publishing.DefaultKnowledgePublisher;
import cz.cuni.mff.d3s.jdeeco.ros.BeeClick;
import cz.cuni.mff.d3s.jdeeco.ros.Positioning;
import cz.cuni.mff.d3s.jdeeco.ros.RosServices;

/**
 * This program deploys single real Cleaner robot
 * 
 * @author Vladimir Matena <matena@d3s.mff.cuni.cz>
 *
 */
public class DeployRealRobot {
	/**
	 * Number of garbage locations initially assigned to each robot
	 */
	private static int GARBAGE_PER_ROBOT = 10;
	
	/**
	 * Initial robot position
	 */
	private static PositionPlugin positionPlugin = new PositionPlugin(12, 5);
	
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
	
	public final static int ROBOT_ID = 42; 

	public static void main(String[] args) throws Exception {
		WallTimeTimer wallTimer = new WallTimeTimer();
		
		RosServices rosServices = new RosServices(
				System.getenv("ROS_MASTER_URI"),
				InetAddress.getLocalHost().getHostName());

		// Create main application container
		DEECoNode node = new DEECoNode(ROBOT_ID, wallTimer,
				new Network(),
				new BeeClick(),
				new DefaultKnowledgePublisher(),
				new KnowledgeInsertingStrategy(),
				rosServices, positionPlugin);

		final String name = "Collector" + ROBOT_ID;
		
		PositionMonitor monitor = new PositionMonitor(wallTimer);

		// Initial garbage location for robot
		List<Position> garbage = new LinkedList<>();
		for (int j = 0; j < GARBAGE_PER_ROBOT; ++j) {
			Position pos = generator.getRandomPosition();
			garbage.add(pos);
		}

		// Deploy DEECo node with robot specific plugins
		Positioning positioning = new Positioning();

		// Deploy Collector robot component
		node.deployComponent(new CleanerRobot(name, positioning, wallTimer, garbage, monitor, generator));

		// Deploy ensembles
		node.deployEnsemble(DestinationAdoptionEnsemble.class);
		node.deployEnsemble(AdoptedDestinationRemoveEnsemble.class);

		wallTimer.start();
	}
}
