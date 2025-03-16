#!/bin/bash

# Wheeled robot training
source ~/cranfield-navigation-gym/cranavgym/tests/setup_env.sh
# python3 train.py -run-name test1 -algo TD3 -lr 0.03 -max-training-steps 1000

# Drone training
python3 train_drone_simple.py -run-name gdp-test -gym-env-id DroneNavigationGDP-v0 -max-training-steps 5000000 -lr 0.003 -obs "lidar" -algo TD3
# python3 train_drone_simple.py -run-name gdp-test -gym-env-id DroneNavigationGDP-v0 -max-training-steps 5001 -lr 0.0003 -obs "camera" -algo TD3
