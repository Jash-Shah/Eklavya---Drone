# Pain and PIDs: Our Journey in Drone Design

## Introduction

Strange title isn't it? It manages to sum up our experience with this project quite well though. For the Eklavya Mentorship program, my teammate and I had the task of building a drone from scratch and designing it's control system.

Quite an intimidating task for the both of us, given that we were complete beginners in almost all areas of robotics. Thankfully we had the guidance of some great mentors and an amazing community thanks to our college club: [SRA](https://sravjti.in/).

> Q: So how does one go about building a drone?

> A: You design it first

## **Trial 1** : Solidworks and URDFs

We started out at the same place everyone does when they need to do something but don't know how to: **YouTube**

![youtube search results](assets/search-results.png)

Incidentally we went for the **fourth video** in this list, mainly because it looked cool

Then came the fun but arduous task of actually making the Drone in SolidWorks. It involved three steps:
1. Making each indicidual part and saving it as a SolidWorks part file(.sldprt).
2. Adding all the parts in an Assembly file and liniking them together using different joints(but mostly prismatic) to create the Drone assembly file.
3. Then came the most crucial part, making the drone look sleek & sexy(actually not that important).

> Tip : Pick a nice 2 hr long album, couple of cups of Mocha and designing in SolidWorks turns into a meditative act.

After a lot of hours in Solidworks, our finished model was ready.

Now, to actually be able to simulate this model and controlled by ROS we needed to convert our SolidWorks assembly into a URDF(Universal Robot Description Format). Fortunately, some kinf hearted soul had thought of adding an extension in SolidWorks where by we could natively export our assembly into an URDF(God Bless the resourcefullness of programmers!).

## **Trial 2** : Understanding ROS & Gazebo 

This step involves us trying to build our Drone a playground to frolick in. It uses two softwares which are going to be critical going forward : 

### 1. ROS(Robot Operating System)

ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.
Basically ROS enables us to control our Drone(and its specific parts) using Python/C++ code. ROS communication implements data transmission between **Nodes** using **Topics**. If the last sentence went over your head then dont worry, we've been there too so let us explain a bit more.
#### ROS Node
All processes in ROS run in a Node. For eg: In our Drone each Wing Link(the joint between the wing and the base),the camera, IMU sensors are all nodes. The Python script we write itself creates many nodes.

#### ROS Topics
Topics are the _named_ buses over which the nodes exchange messages. Topics can be either:
**subscribed to** - For sending data through the topic.
**published to** - For recieving data from the topic.
In general, nodes are not aware of who they are communicating with. Instead, nodes that are interested in data subscribe to the relevant topic; nodes that generate data publish to the relevant topic. There can be multiple publishers and subscribers to a topic.

So, suppose we want to change the speed of wing link 1 of our Drone from 10 to 100 using a Python script then in ROS terms that would look as follows:

(placeholder) 
Python Script creates Node---(pubs to pwm topic)----------------------------------(subs to pwm topic)----> Wing Link Node
<!-- Insert img here -->

### 2. Gazebo
