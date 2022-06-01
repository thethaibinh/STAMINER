## Agile Drone Racing & 3D path planning problem
### 1. Competition
- IROS: 2016 https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7886372
Team KIRD, KAIST, South Korea 
(First Place)
● Team members: Sunggoo Jung, Han-seob Lee, Sungwook Cho, Dasol Lee,

- 2017: Mexico
1st: https://www.youtube.com/watch?v=Qo48pRCxM40
2nd: https://www.youtube.com/watch?v=ImQ8izGQzuk

- 2018: UZH
https://www.iros2018.org/competitions
https://www.youtube.com/watch?v=8RILnqPxo1s
https://arxiv.org/abs/1806.08548


- 2019:
AlphaPilot — Lockheed Martin AI Drone Racing Innovation Challenge:
</br></br>1st Place - MAVLab: shortage path between gate center
</br></br>2nd: UZH-RPG: Dijkstra

- 2020:
The goal was to estimate the quadrotor motion as accurately as possible, utilizing any desired sensor combinations
IROS 2020 FPV Drone Racing VIO Competition
https://fpv.ifi.uzh.ch/iros-2020-fpv-drone-racing-vio-competition-results/
ICRA 2020 FPV Drone Racing VIO Competition
https://fpv.ifi.uzh.ch/previous-competitions/


- 2021: No competition due to Covid, one competition cancelled
Learning eye gaze, human pilot ROI, visual attention 
https://www.youtube.com/watch?v=jxYGxw4eauc

- 2022: Online competition
Benchmark learning
https://www.youtube.com/ watch?v=zqdfVq2uWUA
Visual attention: https://rpg.ifi.uzh.ch/docs/PlosOne22_Pfeiffer.pdf

### 2. Review
- AlphaPilot — Lockheed Martin AI Drone Racing Innovation Challenge:
</br></br>1st Place [1] - MAVLab: shortage path between gate center
</br></br>2nd: UZH-RPG [2]: Dijkstra

- High policies for MPC [3][4] (code) from UZH only apply for known-dynamic obstacles (known-dynamic gate).Policies learning for scenarios is not real-time

- Same for Minimum-Time[5] (code) & Time-Optimal[6] (code), not apply for drone racing &  dynamic environments.

- Real-time: HKUST - Fast[7] (code), UPenn - Minimum-snap[8] (code), Berkeley - motion-primitive[9] (code)
adapt only motion constraint, do not take actuators constraints (saturation) in to account;
do not guarantee collision free in dynamic environments.

### 3. Intended Contribution
- Motion-primitive planner with actuators constraints (saturation) for real-time trajectory generation in an unknown dynamic environment.
- A non-linear controller for agile 3D trajectory tracking 

### 4. Approach
1. Impelement HKUST - Fast, UPenn - Minimum-snap & Berkeley - Motion-primitive. Investigate [10], [11], [12], [13].
2. Implement actuators constraint then compare.
3. Implement non-linear guidance controller for agile 3D trajectory tracking then compare.

[1] The Artificial Intelligence behind the winning entry to the 2019 AI Robotic Racing Competition
</br>[2] AlphaPilot: autonomous drone racing
</br>[3] Policy Search for Model Predictive Control
</br>[4] Learning high-level policies for model predictive control
</br>[5] Minimum-Time Quadrotor Waypoint Flight in Cluttered Environments
</br>[6] Time-Optimal Planning for Quadrotor Waypoint Flight
</br>[7] RAPTOR: Robust and Perception-Aware Trajectory Replanning for Quadrotor Fast Flight
</br>[8] Minimum snap trajectory generation and control for quadrotors
</br>[9] A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation
</br>[10] Perception-aware receding horizon trajectory planning for multicopters with visual-inertial odometry
</br> [11] Rectangular Pyramid Partitioning Using Integrated Depth Sensors (RAPPIDS): A Fast Planner for Multicopter Navigation
</br> [12] Autonomous flight through cluttered outdoor environments using a memoryless planner
</br> [13] Motion Primitives-based Path Planning for Fast and Agile Exploration using Aerial Robots

### 5. Implementation
#### 5.1. Problem: 
Input:
1. Individual single-rotor thrusts (SRT)
2. Mass-normalized collective thrust and bodyrates (CTBR) 
3. Linear velocity commands and yawrate (LINVEL)

Overall, the more low-level you go, the more difficult is going to be to mantain stability, but the more agile your drone will be.

#### 5.2. What we have: 
- Thrust allocation algorithm that demand the SRT control modality.
- Simple linear cascaded control python source code.
- Sophiticated full-stack Ardupilot controller: need to adapt the python function interface or we have to create a ROS bridge for its communication. Beside, the autopilot need to simplified to adapt the procedure of the competition. --> too much load with this approach.
- C++ autopilot source code from ETH & UPenn. To use them, we also have to create a ROS bridge to connect python function interface --> just for reference.
- Source code motion primitive path planning from Berkeley in python. Source code path planning for dynamic, cluttered environment from HKUST, RAPPIDS from Berkeley & minimum-snap trajectory generation from UPenn.
#### 5.3. What we need to do
If we use Ardupilot, we inherit only the thrust allocation algorithm but modify too much, no need to mention connecting existing interfaces.
</br>If we use ETH or UPenn project, we have to build a bridge, creating system complexity.
- --> We need to build the autopilot ourselves in python as well as path planning function.
- --> We need to understand the ObstaclesArray structure and integrate it into the path planning function.

12/05/2022
UPenn - Minimum-snap & Berkeley - Motion-primitive (min average jerk) is just for free space (hard-constraints). They don't integrate collision checking.

Real-time replanning:
HKUST, Nevada & Berkeley:
- Global planning: start --> goal
- **Local planning:**
    - Trajectory generation: 
        - Nevada: motion primitive based on random search on acceleration space.
        - Berkeley: optimize with minimum jerk. The trajectory is optimal with respect to the integral of jerk squared.
        - HKUST: topological path & path-guide (for escaping local minima) gradient-based trajectories optimize from on ESDF data.
    - Collision checking: high computational cost 
        - Nevada: checking collision using volumetric metric map (Voxblox - asl_ETH)
        - Berkeley: Rectangular Pyramid Partitioning
        - HKUST: No checking
    - Optimization:
        - Nevada: Maximum volumetric gain for exploration.
        - Berkeley: Generation effortless control trajectories based of minimum jerk then dropout in-collision ones. Choose the trajectory that optimize explorarion direction. 
        - HKUST: PTO (mention above)
        - Our: shortage & risk-aware trajectory.



