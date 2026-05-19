#!/bin/bash

# --- CONFIGURATION ---
PX4_DIR="/home/$USER/PX4-Autopilot"
QGC_PATH="/home/$USER/Desktop/QGroundControl-x86_64.AppImage"


# 1. Select Model
echo "--- Select the vehicle model ---"
options_model=("x500_vision" "x500_depth")
select opt_m in "${options_model[@]}"; do
    case $opt_m in
        "x500_vision") MODEL="x500_vision"; break;;
        "x500_depth")  MODEL="x500_depth"; break;;
        *) echo "Invalid option $REPLY";;
    esac
done

# 2. Select World
echo "--- Select the simulation world ---"
options_world=("roboverse" "aprilworld" "empty")
select opt_w in "${options_world[@]}"; do
    case $opt_w in
        "roboverse")  WORLD="roboverse"; break;;
        "aprilworld") WORLD="aprilworld"; break;;
        "empty")      WORLD="default"; break;;
        *) echo "Invalid option $REPLY";;
    esac
done

# 3. Ask to start QGroundControl
echo "--- Start QGroundControl? ---"
options_qgc=("Yes" "No")
select opt_q in "${options_qgc[@]}"; do
    case $opt_q in
        "Yes") START_QGC=true; break;;
        "No")  START_QGC=false; break;;
        *) echo "Invalid option $REPLY";;
    esac
done

# 4. Set Environment Variables
export PX4_HOME_LAT=47.397742
export PX4_HOME_LON=8.545594
export PX4_HOME_ALT=488.0

# --- EXECUTION ---

# Conditionally start QGroundControl
if [ "$START_QGC" = true ]; then
    if [ -f "$QGC_PATH" ]; then
        echo "Starting QGroundControl..."
        "$QGC_PATH" &
        sleep 2 # Give QGC a moment to initialize
    else
        echo "Warning: QGroundControl not found at $QGC_PATH"
    fi
fi

if [ -d "$PX4_DIR" ]; then
    cd "$PX4_DIR"
    echo "------------------------------------------"
    echo "Launching: $MODEL in $WORLD"
    echo "------------------------------------------"
    
    PX4_GZ_WORLD=${WORLD} make px4_sitl gz_${MODEL}
else
    echo "Error: PX4 directory not found at $PX4_DIR"
    read -p "Press enter to close..."
fi
