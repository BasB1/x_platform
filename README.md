# X platform project
This project researches possible solutions for cross platfrom solutions. Examples are: communication, localization, collision avoidance etc. Because there are many different types of Autonomous Mobile Robots (AMR) of different vendors the need for cross platform systems are increasing. Creating a system which can interface via different types of communicaiton busses can allow for a more complex system which combines different AMRs in one robotic application. This project looked spicifcally in localization and communcation using Ultra-wide Band (UWB) as the upper level system.

## Localization
This package utilizes for UWB nodes to localize from one robot to another. The localizing robot is called the local robot and the localized the external robot. Both carry two nodes. As the system is ranging between the nodes, for distances are aquired. These distances are interpreted as radiuses/circles which can interm be used to calculate intercesction points in these circles. For example: if one node of the external robot is being localized, the local robot will generate one range with both its nodes resulting in two distances (whic are assumed to be radiuses of circles). Using these two circles, intersection points can be calculated (using the math [here](http://www.ambrsoft.com/TrigoCalc/Circles2/circle2intersection/CircleCircleIntersection.htm)). By localizing two external nodes, orientation can be determined and result in localization being achieved.

## Communication
Using the UWB to communicate was challanging since the Pozyx system si limited in communication functionalities. This project succeded in communicating limited odometry data via UWB. By only sneding essential data, it is possible to track the external robot after localizing it. The odometry data is being compressed using Zlib to a string containing less than 100 bytes. 

## How it works
In the launch file it is possible to specify at what distances the system will localize and at what distance it will communicate. Localization is done when the distance is below the specified range, same goes for communication. During localization, the system updates itself continuously with the most rescent localization data. When the system transitions from localization to communication, the data is used to create an acnhor point which is then used to transform the first received odometry data to the starting position of the external robot. This starting position is then used to continue the tranformation of the odometry data. This way, the external robot is tracked.

### Installation software
The package depends on the Pozyx libray which can be installed using pip:
```
$ pip install pypozyx
```
Then go to the src directory of your ROS ws and clone the repository:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/BasB1/x_platform
```
And that is it. You can now configure the launch file to your liking. It is important to note that the physical position of the nodes are configured and are measured as precisely as possible for the best results. The least amount of distance that there has to be between the two nodes are 40 centimeters.

After cofiguring the launchfile it is important to update the yaml in the src directory. Here it is important to match the node ID's to the robots which you will be using. Also, there is an paramater which is called "do_ranging". When the value of 1 is given, the normal registered ranging is executed by the Pozyx system. Due to limitation of the Pozyx system it is almost impossible to do ranging on two different nodes (it was not meand to be used liek this). To give the second robot the posibility to also range, the "do_ranging" needs to be set to 0. When set to 0, unregistered ranging tasks are executed.

### Installation nodes
When installing the UWB nodes to the robot, it is important to keep atleast 40 centimeter distnce between the two. When you go bellow this value, interference is to high and the localization results are poor. It is also recommended to alternate the antena's height.

### Limitations
The UWB is limited in ranging tasks. As described under installation software, only two robots can be used for this system. It has hower been programmed to support more robots.
