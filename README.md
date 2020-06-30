# NNMPC
Intention-based NNMPC controller for CARLA simulator. NNMPC is a mathematical control framework based on Model Predictive Control (MPC) encompassing a state-of-the-art Recurrent Neural network (RNN) architecture. (SCRIPTS TO BE PUBLISHED)

![demo](demo/demo.gif)

S. Bae, A. Nakhaei, C. Choi, D. Saxena, K. Fujimura, and S. Moura, _Cooperation-Aware Lane Change Maneuver in Dense Traffic based on Model Predictive Control with Recurrent Neural Network_, [preprint](https://arxiv.org/pdf/1909.05665.pdf).
```
@article{bae2019cooperation,
  title={Cooperation-Aware Lane Change Maneuver in Dense Traffic based on Model Predictive Control with Recurrent Neural Network},
  author={Bae, Sangjae and Saxena, Dhruv and Nakhaei, Alireza and Choi, Chiho and Fujimura, Kikuo and Moura, Scott},
  journal={arXiv preprint arXiv:1909.05665},
  year={2019}
}
```


## Prerequisites
- Python 2.7 [anaconda](https://www.anaconda.com/distribution/)
- [Carla 0.9.6](https://github.com/honda-research-institute/carla-lane-change-setup/blob/master/doc/Carla-Simulator-Setup.md)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)


## Setup
Create a virtual environment
```bash
conda create -n carla-env python=2.7
conda activate carla-env
```

Install carla-lane-change-setup
```bash
git clone https://github.com/honda-research-institute/carla-lane-change-setup.git
cd carla-lane-change-setup
pip install -r requirements.txt
./setup_workspace.sh
cd catkin_ws
catkin_make
```

Install dependencies of SGAN
```bash    
pip install -r requirements.txt   # Install dependencies
```

Install rospy [ubuntu]
```bash
sudo apt-get update -y
sudo apt-get install -y python-rospy
```


## Project structure
- ```nnmpc_source```: contains NNMPC files
    - ```nnmpc.py```: MPC with SGAN
    - ```lane.py```: Lane class
    - ```vehicle.py```: Vehicle class
    - ```lane_keep_opt_planner.py```: MPC-based smooth trajectory planner
    - ```path_optimizer.py```: solve MPC for smooth trajectory
    - ```extended_kalman_filter.py```: extended Kalman Filter
    - ```utils.py```: helper functions
    - ```frenet_utils.py```: helper functions for frenet coordinates
- ```sgan_source```: contains source code of SGAN and trained model
    - ```predictor.py```: contains the _Predictor_ class, which loads trained SGAN model
- ```config.py```: hyper parameters. Descriptions are found [here](https://github.com/bsj1216/NNMPC_CARLA/blob/master/fields.md)
- ```init_ros_node.py```: run NNMPC as a ROS node
    

## Run NNMPC with Carla + scenario runner

**Step 1:** Start Carla Simulator (carla documentation: http://carla.readthedocs.io/en/latest/).

```bash
source ~/.bashrc
conda activate carla-env
source ~/carla-lane-change-setup/catkin_ws/devel/setup.bash
sudo nvidia-docker run -p 2000-2001:2000-2001 -it --rm carlasim/carla:0.9.6 ./CarlaUE4.sh
```

**Step 2:** Launch ROS Bridge.
In a new terminal
```bash
source ~/.bashrc
conda activate carla-env
source ~/carla-lane-change-setup/catkin_ws/devel/setup.bash
roslaunch demo_entrance carla_ros_bridge_with_ego_vehicle.launch
```
More information about the Carla ros bridge can be found on the [Official Repo](https://github.com/carla-simulator/ros-bridge/blob/master/README.md).


**Step 3:** Launch NNMPC.
In a new terminal
```bash
source ~/.bashrc
conda activate carla-env
source ~/carla-lane-change-setup/catkin_ws/devel/setup.bash
python ~/NNMPC_CARLA/init_ros_node.py
```

**Step 4:** Start lane change scenario under `scenario_runner` to test your lane change algorithm.

use the following parameters:
--dense: true or false (dense traffic vs spars traffice)
--cooperative: true or false (more cooperative drivers vs less cooperative drivers)
--repetitions: n
```bash
git clone git@github.com:honda-research-institute/scenario_runner.git
cd scenario_runner
source ~/.bashrc
conda activate carla-env
source ~/carla-lane-change-setup/catkin_ws/devel/setup.bash
python ~/scenario_runner/scenario_runner.py --scenario IntentionAwareLaneChange_3 --cooperative false --dense true --repetition 30 --waitForEgo
```

The above commands are tested with Carla version 0.9.6 with Python 2.7 kernel on Ubuntu 16.04. 

## Author 

Developed by Sangjae Bae as part of his internship at Honda Research Institute, USA.

## Acknowledgements

- Supervisor: Alireza Nakhaei, David Isele
- Collaborator: Peng Xu, Alexandre Miranda Anon, Kikuo Fujimura
