# DAPPer: Depth-Aware Prunning for memoryless Planner

<!-- [![IMAGE ALT TEXT HERE](docs/imgs/video.png)](https://youtu.be/LSu25NH6fW0) -->


<!-- Would you like to push the boundaries of drone navigation? Then participate in the dodgedrone competition!
You will get the chance to develop perception and control algorithms to navigate a drone in both static and dynamic environments. Competing in the challenge will deepen your expertise in computer vision and control, and boost your research.
You can find more information at the [competition website](https://uzh-rpg.github.io/icra2022-dodgedrone/). -->

<!-- This codebase provides the following functionalities:

1. A simple high-level API to evaluate your navigation policy in the Robot Operating System (ROS). This is completely independent on how you develop your algorithm.
2. Training utilities to use reinforcement learning for the task of high-speed obstacle avoidance.

All evaluation during the competition will be performed using the same ROS evaluation, but on previously unseen environments / obstacle configurations. -->

## Update

<!-- - **27 March 2022** Fix a static object rendering issue. Please download the new Unity Standalone using [this](https://github.com/uzh-rpg/agile_flight/blob/main/setup_py.bash#L32-L39). Also, git pull the project. -->

<!-- ## Flight API

This library contains the core of our testing API. It will be used for evaluating all submitted policies. The API is completely independent on how you build your navigation system. You could either use our reinforcement learning interface (more on this below) or add your favourite navigation system. -->

### Prerequisite

1 . Before continuing, make sure to have g++ and gcc to version 9.3.0. You can check this by typing in a terminal `gcc --version` and `g++ --version`. Follow [this guide](https://linuxize.com/post/how-to-install-gcc-compiler-on-ubuntu-18-04/) if your compiler is not compatible.

2. In addition, make sure to have ROS installed. Follow [this guide](http://wiki.ros.org/noetic/Installation/Ubuntu) and install ROS Noetic if you don't already have it.

3. Install catkin tools.
```
sudo apt install python3-catkin-tools 
```
4. Install [anaconda](https://www.anaconda.com/).

### Installation

We only support Ubuntu 20.04 with ROS noetic. Other setups are likely to work as well but not actively supported.

Start by creating a new catkin workspace.

```
cd     # or wherever you'd like to install this code
export ROS_VERSION=noetic
export CATKIN_WS=./dapper
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fdiagnostics-color

cd src
git clone https://github.com/thethaibinh/agile_flight
cd agile_flight
```

Run the `setup_ros.bash` and `./setup_py.bash` sequentially in the main folder of this repository, it will ask for sudo permissions. Then build the packages.

```bash
./setup_ros.bash
./setup_py.bash
catkin build
```

**Testing approaches in the simulator:**
Navigate to the workspace directory and run
```
source devel/setup.bash
cd src/agile_flight/
```
To run the the evaluation automatically, you can use the `./run.bash N` script provided in this folder. It will automatically perform `N` rollouts and then create an `evaluation.yaml` file which summarizes the rollout statistics.

**Visualize results**
Change the name for each result file as "result_easy", "result_medium", "result_hard" corresponding to the environment.
Then run plot script in /evaluation_results.