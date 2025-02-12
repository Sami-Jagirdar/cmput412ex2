
# Duckiebot Exercise 2 - Code Implementation

## Collaborators
Sami Jagirdar [ccid: jagirdar, sid: 1686267]
Basia Ofovwe [ccid: ofovwe, sid: 1667223]

## Customize and Annotate Camera Image

./packages/ex2p1/src/camera_reader_node.py for the Node implementation
./launchers/ex2p1-camera-reader.sh to launch the program

## Straight Line Task (1.25m Forwards & Backwards)

./packages/ex2p2/src/straight_line_task.py for the Node implementation
./launchers/ex2p1-straight-line.sh to launch the program

## Rotation Task (90 degrees clockwise and counter-clockwise)

./packages/ex2p2/src/rotation_task.py for the Node implementation
./launchers/ex2p2-rotate.sh to launch the program

## D-Shape Trajectory using LEDs to signal state

./packages/ex2p3/src/wheel_of_D_node.py for the main task logic of following the D-shaped path and publishing the states of the trajectory

./packages/ex2p3/src/led_service_node.py for subscribing to states topic and controlling the LED lights based on the state data received

./launchers/ex2p3-D.sh to launch the main task
./launchers/ex2p3-led-service.sh to launch the LED control service task

## Plot tracked trajectory from rosbag files

./bag_decode/plot_trajectory_from_velocity.py script to load the contents of the bag file, calculate the x and y positions in the world frame and plot the trajectory

./bag_decode/plot_trajectory_animate.py The same exact logic for plotting the trajectory, but included functions to animate the plotted trajectory in gif (the code for animating the plot was generated using the help of ChatGPT)

# Ensuring Nodes shutdown

For all nodes (except subscriber nodes that are required to persist), ensured that the task logic executed within a while loop is broken out of (using break statement) and also explicitly call rospy.signal_shutdown(). All tasks terminate once the statements are executed

## Full Write-up

You can read the full project write-up on my website:  
👉 [Project Page](https://sami-portfolio-xi.vercel.app/projects/CMPUT412/ex2)

___
# Template: template-ros

This template provides a boilerplate repository
for developing ROS-based software in Duckietown.

**NOTE:** If you want to develop software that does not use
ROS, check out [this template](https://github.com/duckietown/template-basic).


## How to use it

### 1. Fork this repository

Use the fork button in the top-right corner of the github page to fork this template repository.


### 2. Create a new repository

Create a new repository on github.com while
specifying the newly forked template repository as
a template for your new repository.


### 3. Define dependencies

List the dependencies in the files `dependencies-apt.txt` and
`dependencies-py3.txt` (apt packages and pip packages respectively).


### 4. Place your code

Place your code in the directory `/packages/` of
your new repository.


### 5. Setup launchers

The directory `/launchers` can contain as many launchers (launching scripts)
as you want. A default launcher called `default.sh` must always be present.

If you create an executable script (i.e., a file with a valid shebang statement)
a launcher will be created for it. For example, the script file 
`/launchers/my-launcher.sh` will be available inside the Docker image as the binary
`dt-launcher-my-launcher`.

When launching a new container, you can simply provide `dt-launcher-my-launcher` as
command.
