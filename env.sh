# source this file:  source ~/nidar_airmouse/env.sh
export NIDAR_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH=$NIDAR_HOME/models:$NIDAR_HOME/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export PATH=$PATH:$HOME/.local/bin:$HOME/ardupilot/Tools/autotest
source /opt/ros/jazzy/setup.bash
