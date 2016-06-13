# README
The code in this repository is the implementation and launcher for the experiment designed to help practice and research in development of adaptation techniques in the field of Smart Cyber-physical systems. In detail a scenario based on an autonomous cleaning of dirty locations in an office by a cloud of robots was provided together with an experimental application and adaptation code. The code can be executed either as an simulation of a possibly large robotic system, or it can run on top of a real robots.
For further detail on the scenario see [Model problem and testbed for experiments with adaptation in smart cyber-physical systems](http://dx.doi.org/10.1145/2897053.2897065)

## Getting started
This repository contains a demo application together with launcher code that can either run pure software simulation of the robotic system or run on real robots. Moreover a virtual machine with already compiled and configured simulation is available for quick and easy overview of the technology. Thus there are three options how to run the provided experiment:

1. Run **simulation** in provided **virtual machine** (approximately **30 minutes** to accomplish)
2. Compile **simulation** from source, configure and **deploy on a server** (approximately **a day** of work)
3. Run experiment on **real robots**, put robots together, customize experiment (may take a **week or two** or even longer)

Regarding the setup time demands of different experiment configurations mentioned above it is recommended to think about desired experience with the tool in advance. The virtual machine is good enough to observe provided code and hands on experience with the experiment scenario and provided architecture structure. Extending to native installation of the simulation system on a server while running the application and adaptation logic in Eclipse on a desktop may be reasonable time investment to enable more comfortable development and large scale simulations. Finally the real deployment is intended to verify results obtained from the simulation. Even when the real deployment is accelerated by sharing the user code and most of the interfaces a lot of time is required to deploy the system on real robots. As all the robot components can be obtained on the market, building the robots do not require intense technical skills. But knowledge of different software technologies may be required to successfully accomplish the real deployment. Finally, conducting experiments with a cloud of real robots may be also time demanding. Thus it is recommended to have all the code written and tested in the simulation before proceeding to real robots.

Instructions to run the experiment are given in the following sections. Each section describes one way how to run the experiment.

## 1. Simulation in the VM
The virtual machine can be obtained as part of the bundle available [here](http://d3s.mff.cuni.cz/software/deeco/files/seams-2016-artifact.zip). It contains all necessary software and configuration. Simulating the experiment should be very easy. Basically it is just about running the VM and following the on-screen instructions.

### Virtual machine technical details
- operating system: Xubuntu
- username: seams
- password: seams
- password-less sudo - no security
- recommended 2+ core CPU
- recommended 4+ GiB of RAM

### Run instructions
1. Import VM (All major virtualization tools should support it)
2. Start the VM
3. Eclipse will start when XUbuntu starts
4. Run ServerRunner (in recent launches in the Eclipse menu)
5. Run GarbageCollectDemo (in recent launches in the Eclipse menu)
6. The demo should run
 1. Stage window should open showing the map and robots (colored circles)
 2. After a while roots should start moving
 3. This simulation runs for about 10 minutes
 4. On exit, some exceptions related to network connections may be logged in the console, this is normal and does not indicate error.
 5. "results" directory in demo project will contain report for the completed run.
 6. If the demo is to be run again it is not necessary to restart the ServerRunner.

## 2. Native simulation
The setup similar to the one in the VM is described here. Although it may be beneficial to run the simulation and ROS stack on server, while keeping the example application running locally in Eclipse.

1. Install Xubuntu (Most probably another Debian based distribution will work the same way)
2. Setup ROS indigo repository [manual](http://wiki.ros.org/indigo/Installation/Ubuntu)
	**sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'**
	**sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116**
	**sudo apt-get update**
3. Install ROS and Stage
	**sudo apt-get install ros-indigo-turtlebot-stage**
4. Install rviz (optional)
	**sudo  apt-get install ros-indigo-desktop**
	**sudo  apt-get install ros-indigo-rviz**
4. Compile OMNeT++ and INET from source attached in the artifact (or use upstream version once 802.15.4 is mature enough)
 1. Install dependecnies
	**sudo apt-get install bison**
	**sudo apt-get install flex**
	**sudo apt-get install g++**
 2. Compile sources
	**cd /path/to/omnet**
	**source setenv**
	**./configure**
	**make**
	**cd /path/to/inet**
	**make makefiles**
	**make**
9. Configure environment
 - Add this to .bashrc if you are using VM
	**LIBGL_ALWAYS_SOFTWARE=1**
 - Add this to .bashrc
	**source /opt/ros/indigo/setup.bash**
	**source ~/path/to/catkin_ws/devel/setup.bash**
	**export ROS_MASTER_URI=http://localhost:11311**
 - Reopen console (apply new .bashrc)
5. Compile ROSOmnet++
	**cd /path/to/omnet**
	**source setenv**
	**cd /path/to/rosomnet**
	**make makefiles**
	**make**
	**ln –s /media/sf_ROSOMNeT\+\+ \`pwd\`**
6. Install java 8
	**sudo add-apt-repository ppa:webupd8team/java**
	**sudo apt-get update**
	**sudo apt-get install oracle-java8-installer**
7. Install eclipse
 - Mars is recommended as it contains maven and git
8. Running eclipse
 - Run eclipse from command-line because of the environment variables
9. Import ROSRemove and seams2016-adaptation-testbed projects
 - Dependencies are downloaded from maven
11. Run ServerRunner (in recent launches)
12. Run GarbageCollectDemo (in recent launches)
13. The demo should run
 1. Stage window should open showing the map and robots (colored circles)
 2. After a while roots should start moving
 3. This simulation runs for 10 minutes
 4. On exit some exceptions related to network connections may be logged in the console, this is normal and does not indicate error.
 5. "results" directory in demo project will contain report for the completed run.
 6. If the demo should be run again it is not necessary to restart the ServerRunner.
 
## Real robot deployment
First it is important to note that working with real robots is a lot different from aforementioned simulation. First a real hardware is needed. Second a deployment for a single robot is described here. The deployment of the whole system is to be handled manually. This section is split into description of the robot hardware and software stack.

### Robot hardware
Even when it is expected that the experiment can be adjusted for most [ROS](http://www.ros.org) compatible robots, the tested [Turtlebot 2](http://www.turtlebot.com) bases setup is described here.
The robot consists of two parts. The first one it the main body composed of Turtlebot 2 base with 3D camera and a laptop. The second part is the sensor and communication board. It is a bit more custom part, but still it is a commodity hardware witch can be easily obtained.

#### Main body
 - [Turtlbot 2 (base with platforms)](http://www.robotnik.eu/mobile-robots/turtlebot-ros)
 - [Asus Xtion 3D camera](https://www.asus.com/3D-Sensor/Xtion_PRO)
 - Core 2 Duo based laptop with 6 GiB of RAM (3 USB ports are required. Can be supplied by a hub, but Xtion needs to be connected directly. WiFi connection is a nice tool for debugging.)
 
In order to assemble main body, just follow instructions shipped with robot. No special technical skills, but mounting screws and connecting connectors are required.
 
#### Communication and sensor board
 - [STM32F4 discovery shield](http://www.mikroe.com/stm32/stm32f4-discovery-shield) - Board base
 - [STM32F4](http://www.st.com/stm32f4) - MCU unit
 - [802.15.4 network interface](http://www.mikroe.com/click/bee) - bee click
 - [Temperature and humidity sensor](http://www.mikroe.com/click/sht) - SHT1x click
 - [GPS](http://www.mikroe.com/click/gps2) - GPS2 click
 
Assembly of communication and sensor board is a bit more complex. In general it is necessary to assemble all parts described earlier. Some of the *click* extension boards may require soldering connector pins to PCB which require soldering iron, but almost no technical skills. Due to limitations in firmware described in following section of this document it is necessary to mount all click to defined slots on the *discovery shield* even when some of these will not be used. *SHT1x click* must be connected to Slot 1, *Bee click* to slot 3, and *GPS click* to slot 4. Finally the board needs to be connected to the on-board computer via a **mini** USB cable via the mini USB port located on discovery shield marked as *USB UART*. Remember, the other mini USB is for flashing the firmware and the micro USB port will also not work.  

### Robot software
As in previous section a tested setup is described here. It is assumed that a slight modification to the used software can extend supported hardware or mode of operation if required. The hardware of the robot consists of a Linux operating system, standard ROS modules, custom ROS module driving the *communication and sensor board*, communication and sensor board firmware, and finally example application. The robot software stack will be described part by part in following subsections.

#### On-board computer operating system
X86_64 XUbuntu Linux was used as an on-board operating system. If required any Ubuntu, or Debian based Linux distribution should be enough. If required it is possible to compile everything from source giving compatibility with virtually any Linux distribution, but this approach will not be explained. Using 32bit system may also be possible, but not tested.

#### Turtlebot ROS modules
In order to use ROS it is necessary to add ROS as external package repository to XUbuntu, install Turtlebot related packages, install 3D camera packages and verify that the robot stack is working as expected. As most of the steps described is already covered in ROS tutorials, these will not be described here in detail.

##### Instructions:
1. Complete [ROS Turtlebot tutorial including adding ROS into Ubuntu](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation)
2. Verify it is possible to control the robot using the keyboard tele-operation. It is much more fun with SSH using a WiFi connection.
3. Install [RViz](http://wiki.ros.org/rviz)
4. Make 3D camera working. [Related ROS module description](http://wiki.ros.org/openni_camera)
5. Verify that *RViz* can show depth image captured by on-board camera. If network is configured properly, it should be possible to run RViz on different computer and let remotely controlled robot move around while seeing live depth feed from it.
6. As a first step to localization it is necessary to make Simultaneous Localization and Mapping (SLAM) running. [There is a tutorial on this.](http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM) Depth image will be used to obtain laser-scan like data. 
7. Build a map of the environment in which the experiment will take place.
8. Enhance the map. Most probably the map will be a bit stretched in some direction. It is wise to fix map dimensions and errors as well as removing all temporary/movable objects.
9. Make [Autonomous Navigation of a Known Map with TurtleBot](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map) working. Use map obtained earlier.
10. Verify that it is possible to see the map in *RViz* and it is possible to order robot to move to a chosen location on the map autonomously.

Congratulations a main robot body is now fully operational as required for the experiment.

#### *Communication and sensor board* firmware
Before proceeding to connection of the *Communication and sensor board* to the ROS, it is necessary to build and flash firmware for it. The firmware source code is located in the [Github repository](https://github.com/d3scomp/beeclickarm/tree/robot-additions). It is necessary to take the code from *robot-additions* branch of the repository.
Building the source code is straightforward. A Linux system with *armv7m-hardfloat-eabi* tool-chain, cross-compiler, and [OpenOCD](http://openocd.org) is required to proceed. To compile the firmware fillow these instructions:

1. Clone the *robot-additions* branch of the [project](https://github.com/d3scomp/beeclickarm/tree/robot-additions).
2. Open shell and navigate to the root of the cloned repository.
3. Switch to beeclickarm directory.
4. Execute *make* command (Fixing paths to the compiler in a Makefile may be necessary)
5. Connect the *STM32F4* board with MCU using the *miniUSB* cable. Do **NOT** use the port marked as USB UART on the discovery shield, nor the micro USB port.
6. Execute *make flash* command. (Again, it may be necessary to fix paths to OpenOCD in the Makefile)

Congratulations, now the *Communication and sensor board* board should be operational. Brave ones can strip the SHT1x and GPS driver from the code as these are not required. Doing so will remove necessity to have the *SHT1x click* and *GPS 2 click* installed.

#### Communication and sensor board ROS integration
Running the ROS integration modules is a bit easier than building the firmware. The module is written in Java 1.8 and uses Maven to obtain its dependencies. In order to run the modules clone a [beeclickarmROS](https://github.com/d3scomp/beeclickarmROS) project from Github. It is possible to build and execute the module from command-line using these commands:

1. *mvn compile*
2. *java EquipmentProviderRunner /dev/ttyUSB0* # Assuming the /dev/ttyUSB0 is serial interface from robot to *Communication and sensor board*.

In the graphical environment it is recommended to use Eclipse with Maven integration to build the project and export the *jar* with *EquipmentProviderRunner.main* as entry point.

Now *Communication and sensor board* should be exported as ROS topics and services. It is possible to check using the *rostopic* and *rosservice* tools. But in order to get full output from these tools it is necessary to build the [beeclickarmROS](https://github.com/d3scomp/beeclickarmROS) using *catkin* as a ROS project and include resulting binaries in the environment variables. This is optional and out of scope of this document. 

#### Example application
Example application is using [jDEECo framework](http://d3s.mff.cuni.cz/software/deeco) as an architectural base for its adaptation and business logic. As all the dependencies and jDEECo plugins are resolved using Maven, running single node of example application should be straightforward. It is necessary to run the application using a modified version of *DeployRealRobot.main* method replacing robot id, garbage locations, and robot initial location with desired values.

Single robot should start visiting assigned garbage locations. More robots on the scene should communicate using 802.15.4 packets and resolve problems using local and collective adaptation techniques described as *Processes* in *CleanerRobot* class and *Ensembles* such as one described in *DestinationAdoptionEnsemble* class.

**Congratulations, now it is possible to proceed to experiments with adaptation techniques for real robots.**

 

## Appendix A: Simulation configuration 

### Simulation parameters
Naturally all the parameters of the simulation can be changed in the source code. The most important file for changing simulation parameters is GarabgeCollectionDemo.java. GarbageCollectDemo class defines several constants that can be used to tune simulation. For instance it is possible change robot count, colors and initial positions. It is also possible to change count of intial goals. In theory it might be also interesting to change how the goals are generated. Moreover it is possible to switch DEECo simulation plugins. One of possible changes is commented out in the source code. By toggling comments on lines 93 and 94 it is possible to switch network device implementation from OMNeT++ based to even less realistic fake networking. This may be of good use as it saves performance and provides more deterministic communication. Such less realistic communication can serve well for early development stages when the application cannot cope well with network failures. Finally it is possible to change constant which says how long the simulation should run.

### Processing collected data
Each simulation run results in creation of a file with statistic data in *results* subfolder in demo project. In order to process the data into figures please use python script located in the same directory. The output images will be stored in the same directory. The script can be executed this way:

	seams2016-adaptation-testbed/results$ python3 process.py
    
The output are two boxplots for number of reached targets and time until last reached goal.

### Inspecting simulation at runtime
It is possible to inspect simulation progress at runtime. The map and location of the robots are displayed in the Stage simulator graphical output. Moreover it allows to drag and drop robots.
More advanced diagnostics can be achieved using *rviz* tool which is part of ROS installation. The tool can display almost all important data managed by ROS in nice graphical way. For instance costmaps and laserscans can be shown as overlay of static map. Unfortunately usage of this tool requires graphical acceleration which is not available in example virtual machine. Without acceleration the tool tends to work very slowly and often crashes.

## Appendix B: Notes on adaptation architecture
This is just an ultra brief introduction into the topic. Many more reading is recommended to get some insight into the architecture and runtime used:
- [Model problem and testbed for experiments with adaptation in smart cyber-physical systems](http://dx.doi.org/10.1145/2897053.2897065)
- [DEECo: an ecosystem for cyber-physical systems](http://dx.doi.org/10.1145/2591062.2591140)
- [Development of smart cyber-physical systems](http://dx.doi.org/10.1145/2602458.2611458)
- [The E-mobility Case Study](http://dx.doi.org/10.1007/978-3-319-16310-9_17)
- [Gossiping Components for Cyber-Physical Systems](http://dx.doi.org/10.1007/978-3-319-09970-5_23)

### Component knowledge
Component knowledge can be extended/changed by modifications to CollectorRobot.java source file. Non-static fields which are public are considered component knowledge. The knowledge can be added just by introducing new field. Knowledge annotated as @Local never leaves the node. Thus cannot be used in ensembles evaluated on remote nodes.

### Component local adaptations and processes
Local adaptation strategy implemented in the demo application can be easily extended or changed by modifications to CollectorRobot.java source file. Process is just a static method in the class. Processes are annotated with scheduling parameters and the parameters are passed using @In, @Out and @InOut annotations. See current processes for detailed information on how to write processes.

### Adaptation working with knowledge of remote components
This kind of adaptation requires ensemble for knowledge mapping. AdoptedGoalRemoveEnsemble.java and BlockedGoalAdoptEnsemble.java include implementation if this mechanism for goal exchange adaptation described in the paper.
