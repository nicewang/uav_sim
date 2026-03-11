[back](../README.md)
# UAV Sim
Simple Hybrid UAV Simulation

## Using `gym-pybullet-drones`
[gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones)

### Install
#### Prerequisites
[requirements.txt](requirements.txt)
#### Install
```bash
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
# PS. In this repo, we use the forked repo (as submoudule) to 
#     keep the fixed `gym-pybullet-drones` version for (running) stability. 

cd gym-pybullet-drones
pip install -e .
# pid example
python gym_pybullet_drones/examples/pid.py
```
### Cases
[Case 1: Waypoint Tracking](case-1-waypoint/)
