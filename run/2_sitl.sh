#!/bin/bash
source "$(dirname "$0")/../env.sh"
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --out=127.0.0.1:14551 --out=127.0.0.1:14552
