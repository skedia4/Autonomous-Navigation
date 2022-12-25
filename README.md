#Motion Primitives Based Kinodynamic RRT

This package is the implementation of our work, "Motion Primitives Based Kinodynamic RRT for Autonomous Vehicle Navigation in Complex Environments", presented at IROS, [BADUE 2022] (https://gamma.umd.edu/workshops/badue22/). For a detailed description read our work at conference website or browse to reports folder.


## Examples
### 1.1 Reverse Navigation
<p align='center'>
<img width="100%" src="/figures/reverse_navigation.gif"/>
</p>

### 1.2 Parallel Parking
<p align='center'>
<img width="100%" src="/figures/parallel_parking.gif"/>
</p>

### 1.3 Perpendicular Parking
<p align='center'>
<img width="100%" src="/figures/perpendicular_parking.gif"/>
</p>

### Preliminaries
Coordinate system used with X along car heading,Y along the left of car, and theta (yaw) is positive counter-clocwise. Basically, if you steer left you will see positive Y and positive yaw. X, Y are in metres and theta in radians.

The car intial states (from where hector slam is intialized) is fixed global frame of reference.


### Setup

* Copy the folder 'CS588_project_final' to the desired workspace (it will be better if copied to `~/workspaces/group_11_ws`)

* (IF workspace not `~/workspaces/group_11_ws`) Install Hector SLAM following instructions on assignment 3 README

* Open 3 seperate terminals with each cd to `~/workspaces/group_11_ws/CS588_project_final/launch_files_for sensor_slam_pacmod`  (Advised to use Terminator application)

* `bash sensor_init.sh` on terminal 1
` bash hector_slam_init.sh` on terminal 2
`bash joystick_init.sh.sh` on terminal 3

* Initialize the PACMOD by pressing the two middle buttons on Joystick. 

* If all above steps are completed. You will see a RVIZ window with localization and mapping from HECTOR SLAM. You may be able to control the steering by joystick.


### Running visualization for offline created plans
A few offline created trajectories for parallel, perpendicular, and taking the vehicle out from highbay are avaialble.

* Open the python file `tracking3.py` in a text editor. Naviagate to main function and uncomment either line 285 or line 286. This is for obstacles  visualization. Since, virtual/ simulated obstactes are used to create these plans.
 Ex. Line 286 ` car1 = Car(-2, 2.5, 4, 2.5, 0)	; car2 = Car(8, 2.5, 4, 2.5, 0)   # Parallel parking`

* Uncomment only one line from 288-293: 
```   filename = './Live_plans/Trajectory_test_plan_virtual_0'    ### For visualzing the live plans after running RRT code with simulated obstacles
    #filename = './Live_plans/Trajectory_test_plan_live_map_0' ### For visualzing the live plans after running RRT code with Lidar mapped obstacles
    #filename = './Good_plans/Trajectory_test_plan_virtual_0_parallel_DEC_18' ### Parallel parking better
    #filename = './Good_plans/Trajectory_DEC_17_parallel_found_0' ### Parallel parking old 
    #filename ='./Good_plans/Trajectory_DEC_17_live_map_found_0'### Taking vehicle out from highbay
    #filename = './Good_plans/Trajectory_DEC_15_perpendicular_decent' ### perpendicular parking
```
These are currently avaliable offline plans. Later, If you create your own plan, you can follow the same strucure and add the filename by same method.
By default line 288 will be uncommented. This is to ensure the immediate visulization of the plan, after running RRT code.

* Open new terminal
```bash 
cd ~/workspaces/group_11_ws/CS588_project_final
source ~/demo_ws/devel/setup.bash ##### path to demo workspace 
python3 tracking3.py         
```

* You will see a trajectory plot with obstacles, vehicle initial state, and final state. Marker '*' is also shown which is live position of the vehicle. It will be randomly initialized. After you complete the next section, this GRAPH will track the SLAM localization.

### Running the vehicle with the available plans

* Open the python file `SLAM_pp_tracker_pid.py` in a text editor. Uncomment only one line from 128-134: 
```CSV_filename = traj_csv( './Live_plans/Trajectory_test_plan_virtual_0') #### Default. Directy running the live plan generated from RRT(virtual obstacles).
   #CSV_filename = traj_csv( './Live_plans/Trajectory_test_plan_live_map_0')### Directy running the live plan generated from RRT( mapping by SLAM).
   #CSV_filename = traj_csv( './Good_plans/Trajectory_test_plan_virtual_0_parallel_DEC_18')  ### Parallel parking better
        
   #CSV_filename = './Good_plans/Trajectory_DEC_15_perpendicular_decent.csv'  ### perpendicular parking
   #CSV_filename = './Good_plans/Trajectory_DEC_17_live_map_found_0.csv' ### Taking vehicle out from highbay
   #CSV_filename = './Good_plans/Trajectory_DEC_17_parallel_found_0.csv' ### Parallel parking old 
```
NOTE THE FOLLOWING CODE WILL START MOVING THE VEHICLE AUTONOMOUSLY
* Open new terminal
```bash 
cd ~/workspaces/group_11_ws/CS588_project_final
source ~/demo_ws/devel/setup.bash ##### path to demo workspace 
python3 SLAM_pp_tracker_pid.py        
```

### Creating your own plans with the created kinodynamic RRT planner (virtual obstacles)

* Open the python file `RRT_code_with_motion_primitive_virtual_obastacles.py` in a text editor.

* Change or create virtal obstaces as cars by editing line 146 or 147. For ex. car1 = Car(-2, 3, 4, 2.5, 0) shows a car centred at (-2,3) having length 4m, width 2.5 m, and orientation 0 rad.

* Define start node and goal node using Line 211 and 217. For ex.  N_goal=node(None, 4, 3,0, 0) shows goal with car centre at (4,3) and orientation 0 rad. [Keep last argument always 0]

* Control the RRT expansion by changing number of nodes (line 229, default 1000) and expansion area (x,y) in the state space line 234, 235. Basically, a small area with high number of nodes creates a dense tree (useful to naviage inside small opening of CSPACE)

* Run this script on a new terminal. No ROS dependencies are required for this script. You will see a RRT tree created and few feasible trajectories to your goal. All these trajectories will be saved as binary files in folder `.\Live_plans`

* Note that RRT is a random planner. If you run the same code again, it will show a different trajectory. IF you like a plan then move the binary file to the folder `.\Good_plans`. Otherwise, it wil be overwritten at every execution.

* If you want to run the live created plans.  Just run the scripts mentioned in above two sections (with default values). You will see the vehicle following the same trajectory.

### Creating your own plans with the created kinodynamic RRT planner (live map obstacles) 
[USE AT YOUR OWN DISCRETION, SINCE LIDAR WILL NOT DETECT SMALL OBSTACLES AND EVEN HUMANS!!]

* Open the python file `RRT_code_with_motion_primitive_virtual_obastacles.py` in a text editor.

* Define start node and goal node using Line 211 and 217. For ex.  N_goal=node(None, 4, 3,0, 0) shows goal with car centre at (4,3) and orientation 0 rad. [Keep last argument always 0]

* Control the RRT expansion by changing number of nodes (line 229, default 1000) and expansion area (x,y) in the state space line 234, 235. Basically, a small area with high number of nodes creates a dense tree (useful to naviage inside small opening of CSPACE)

* Run this script on a new terminal. IT has ROS dependecies. Before running source it [source ~/demo_ws/devel/setup.bash ##### path to demo workspace].  You will see a RRT tree created and few feasible trajectories to your goal. All these trajectories will be saved as binary files in folder `.\Live_plans`

* Refer to the previous sections to run the generated plan.

### Dependencies

Most the local dependencies are stored in the project folder. Few Python packages which may be required (if not already installed):

matplotlib.pyplot
numpy
pickle
rospy
sys
random
shapely
regex
scipy
os
CSV
std_msgs.msg
geometry_msgs.msg
tf.transformations
pacmod_msgs.msg

