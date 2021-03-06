
# Project 
Building a drone and developing its control system.  

<!-- TABLE OF CONTENTS -->
## Table of Contents

- [Project](#project)
  - [Table of Contents](#table-of-contents)
  - [About The Project](#about-the-project)
    - [Tech Stack](#tech-stack)
    - [File Structure](#file-structure)
  - [Getting Started](#getting-started)
    - [Prerequisites and installlation](#prerequisites-and-installlation)
    - [Installation](#installation)
    - [Execution](#execution)
  - [Algorithm Flowchart](#algorithm-flowchart)
  - [Results and Demo](#results-and-demo)
  - [Future Work](#future-work)
  - [Contributors](#contributors)
  - [Acknowledgements and Resources](#acknowledgements-and-resources)
  - [License](#license)

<!--ABOUT THE PROJECT -->
## About The Project
Drone aviation is an emerging industry. With possibilities for its applications in agriculture, healthcare, e-commerce as well as traffic control. We wanted to get first hand experience with how a drone is designed as well as how it flies to get a firm grasp on the principles needed to work with drones in the future.  
Our full project report can be found [here](https://github.com/Jash-Shah/Eklavya---Drone/blob/main/report/Eklavya%20Report.pdf)


### Tech Stack
- [Solidworks](https://www.solidworks.com/)
- [ROS Noetic](http://wiki.ros.org/noetic)
- [Gazebo](http://gazebosim.org/)
- [Python 3](https://www.python.org/downloads/)


### File Structure
```
📦Eklavya---Drone
 ┣ 📂assets                           #contains gifs, videos and images of the results
 ┣ 📂cfg                              #config files for the sensors
 ┣ 📂include                          #include files for the plugins
 ┃ ┗ 📂vitarana_drone
 ┃ ┃ ┣ 📜gazebo_edrone_propulsion.h   #propulsion plugin include
 ┃ ┃ ┣ 📜gazebo_ros_gps.h             #gps plugin include
 ┣ 📂launch                           #launch files
 ┃ ┗ 📜drone.launch
 ┣ 📂models                           #files and meshes used to render the model
 ┃ ┗ 📂edrone
 ┃ ┃ ┣ 📂materials
 ┃ ┃ ┣ 📂meshes
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┣ 📂msg                              #contains custom messages which are used to control drone functions
 ┣ 📂scripts                          #python programs used to run the drone
 ┃ ┣ 📂__pycache__
 ┃ ┣ 📜control.py                     #brain of the drone, this file needs to be executed
 ┃ ┣ 📜pid.py                         #contains the math needed to stabilise the drone
 ┣ 📂src                              #contains custom plugins used with the drone
 ┃ ┣ 📜gazebo_edrone_propulsion.cpp
 ┃ ┗ 📜gazebo_ros_gps.cpp
 ┣ 📂worlds                           #world files
 ┃ ┣ 📜drone.world
 ┣ 📜CMakeLists.txt
 ┣ 📜README.md
 ┗ 📜package.xml
 ```

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites and installlation
* Tested on [Ubuntu 20.04](https://ubuntu.com/download/desktop)
* [ROS Noetic](http://wiki.ros.org/noetic/Installation)
* [Gazebo Sim](http://gazebosim.org/)
* Do visit these websites for the installation steps of the above mentioned software. It is recommended to install Gazebo along with ROS and not seperately

### Installation

```sh
git clone https://github.com/Jash-Shah/Eklavya---Drone.git
```
Add this folder in the src directory of your catkin workspace
Create the src folder if it doesn't already exist by
```sh
mkdir src
```
Initialise the project with
```sh
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Execution
Open two terminal windows and run the following commands
- Terminal 1
```sh
source ~/catkin_ws/devel/setup.bash
roslaunch drone vitarana_drone drone.launch
```
- Terminal 2
```sh
source ~/catkin_ws/devel/setup.bash
rosrun vitarana_drone control.py
```


<img src="assets/terminal_start.gif" width="640" height="480" />


<!--Flowchart -->
## Algorithm Flowchart
Simplified code structure  
<img src="assets/PID_Diagram.png">

<!-- RESULTS AND DEMO -->
## Results and Demo


Drone at start of the program:  

<img src="assets/drone_start.png">

Drone when target co-ordinates are given in control.py:  




https://user-images.githubusercontent.com/82902712/138563488-c9e4268a-3700-4b34-b2bd-1795de4a06f7.mp4





<!-- FUTURE WORK -->
## Future Work
- [x] Create a control system for the drone using PID
- [x] Stabilise the Roll, Pitch and Yaw of the Drone 
- [x] Get the drone to fly at any arbitrary altitude
- [x] Have the drone fly to given co-ordinates and stabilise itself
- [ ] Implement obstacle avoidance 

<!-- CONTRIBUTORS -->
## Contributors
* [Toshan Luktuke](https://github.com/toshan-luktuke)
* [Jash Shah](https://github.com/Jash-Shah)


<!-- ACKNOWLEDGEMENTS AND REFERENCES -->
## Acknowledgements and Resources
* [SRA VJTI](http://sra.vjti.info/) Eklavya 2021  
* [E-Yantra IIT-B](https://new.e-yantra.org/) for the plugins as well as the model of the drone. 
* [Nishanth Rao](https://github.com/NishanthARao/ROS-Quadcopter-Simulation) for the template of the PID controllers
* [Tim Wescott](http://wescottdesign.com/articles/pid/pidWithoutAPhd.pdf) for the paper PID without PhD which was extremely illuminating for beginners in PID
* Our mentors [Saad Hashmi](https://github.com/hashmis79), [Karthik Swaminathan](https://github.com/kart1802) and [Dhruvi Doshi](https://github.com/dhruvi29) for their guidance throughout the whole project

<!-- -->
## License
[MIT License](https://opensource.org/licenses/MIT)
