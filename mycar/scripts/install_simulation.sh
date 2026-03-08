#!/bin/bash
# ============================================================================
# Install COVAPSY controller into the Webots simulation
# This copies the controller into the existing simulation directory
# so Webots can find and use it.
#
# Usage: bash install_simulation.sh
# ============================================================================

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
SIM_DIR="$(dirname "$PROJECT_DIR")/my-simulation/Simulation_CoVAPSy_Webots"
CONTROLLER_SRC="$PROJECT_DIR/simulation/controllers/covapsy_controller"
CONTROLLER_DST="$SIM_DIR/controllers/covapsy_controller"

echo "=========================================="
echo "  Installing COVAPSY controller into Webots simulation"
echo "=========================================="

if [ ! -d "$SIM_DIR" ]; then
    echo "ERROR: Simulation directory not found at: $SIM_DIR"
    echo "Make sure my-simulation/Simulation_CoVAPSy_Webots exists"
    exit 1
fi

if [ ! -f "$CONTROLLER_SRC/covapsy_controller.py" ]; then
    echo "ERROR: Controller source not found at: $CONTROLLER_SRC"
    exit 1
fi

# Copy controller
echo "Copying controller to: $CONTROLLER_DST"
mkdir -p "$CONTROLLER_DST"
cp "$CONTROLLER_SRC/covapsy_controller.py" "$CONTROLLER_DST/"

echo ""
echo "Done! To use in Webots:"
echo "  1. Open: $SIM_DIR/worlds/Piste_CoVAPSy_2025a.wbt"
echo "  2. Click on the TT02 car in the scene tree"
echo "  3. Change 'controller' field to 'covapsy_controller'"
echo "  4. Press the play button"
echo "  5. Click on the 3D view, press 'A' to enable auto mode"
echo ""
echo "Keyboard controls:"
echo "  A - Enable auto mode"
echo "  N - Disable auto mode (stop)"
echo "  R - Reverse"
echo "  S - Switch algorithm (advanced/simple)"
echo "=========================================="
