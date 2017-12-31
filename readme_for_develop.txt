# https://github.com/dougvk/tum_simulator

# https://www.youtube.com/watch?v=zwTnY-ZqNcM

source devel/setup.bash
roslaunch cvg_sim_gazebo ardrone_testworld.launch

other tab 
source devel/setup.bash
rosrun drone forward.py


# ###########################################
create simulate
# https://github.com/dougvk/tum_simulator
## install ros indigo

#create ws catkin
mkdir -p ~/tum_simulator_ws/src
cd  ~/tum_simulator_ws/src
catkin_init_workspace

#dependences
    git clone https://github.com/AutonomyLab/ardrone_autonomy.git   # The AR.Drone
    ROS driver
    git clone https://github.com/occomco/tum_simulator.git

# create drone pack
catkin_create_pkg drone std_msgs rospy roscpp
# create script forward.py in drone/src

    cd ..
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y

# Build the simulator
catkin_make

# Run
rosrun $package $node
