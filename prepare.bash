#!/bin/bash

CATKIN_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
export MAP_SAVE_FILE="$CATKIN_DIR/maps/autogenmap"   
echo "Will save maps at: $MAP_SAVE_FILE"