# digit-jsonapi package
ROS package for using Digit JSON API with AR's PySDK

# Prerequisites
* Ubuntu 18.04 OS
* Python 3.6 (needed for asyncio, higher versions may cause errors)
* Install ROS Melodic
* Open terminal and navigate to catkin_ws
```bash
cd release_2021.06.01/catkin_ws/
```
* (Optional) Install PySDK and additional packages into virtual environment with pip. Assumes Python 3.6
```bash
python3 -m venv name_of_venv
source name_of_venv/bin/activate
python3 -m pip install ../docs/agility-pysdk-0.2.0/agility-0.2.0-py3-none-any.whl
python3 -m pip install pyyaml
python3 -m pip install rospkg
```

# Build
Build catkin workspace from the terminal (```catkin_make```) or the vscode task located in tasks.json

# Running nodes
Source environment with 
```bash
source devel/setup.bash
```
Activate virtual environment. For example to use the given virtual environment
```bash
source .virtualenvs/digit-venv/bin/activate
```
Use ```roslaunch``` to execute files. Some require arguments to be set on the commandline. Possible values are shown for string arguments
* action_shutdown.launch
  * args: exp_mode={'sim','real'}
* action_sit.launch
*   * args: exp_mode={'sim','real'}
* action_stand.launch
  * args: exp_mode={'sim','real'}
* action_start.launch
  * args: exp_mode={'sim','real'}
* apply_force.launch
  * args: mx, my, mz, fx, fy, fz, duration
  * all wrench values default to zero. duration defaults to 0.1 seconds
* query_kinematics.launch
  * args: exp_mode={'sim','real'}
* set_op_mode.launch
  * args: exp_mode={'sim','real'}, op_mode={'damping','disabled','locomotion','low-level-api'}

Some example roslaunch commands
```bash
roslaunch digit_jsonapi action_start.launch exp_mode:='sim'
roslaunch digit_jsonapi set_op_mode.launch exp_mode:='sim' op_mode:='damping'
roslaunch digit_jsonapi query_kinematics.launch exp_mode:='real'
```
To publish base pose run **query_kinematics.launch**
