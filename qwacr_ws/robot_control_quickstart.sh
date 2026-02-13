#!/bin/bash
# QWACR Robot Control Quick Start Script
# Run this on the Pi after SSH connection

set -e

echo "====================================="
echo "QWACR Robot Control Quick Start"
echo "====================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}➜ $1${NC}"
}

# Step 1: Check workspace
print_info "Checking ROS2 workspace..."
if [ -f ~/qwacr_ws/install/setup.bash ]; then
    source ~/qwacr_ws/install/setup.bash
    print_success "Workspace sourced"
else
    print_error "Workspace not found! Build it first with: cd ~/qwacr_ws && colcon build"
    exit 1
fi

# Step 2: Find Arduino serial port
print_info "Detecting Arduino serial port..."
ARDUINO_PORT=""

for port in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyUSB0; do
    if [ -e "$port" ]; then
        # Check if it's likely the Arduino (can improve detection later)
        print_info "Found serial device: $port"
        
        # Ask user to confirm
        read -p "Is this the Arduino Mega? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            ARDUINO_PORT=$port
            break
        fi
    fi
done

if [ -z "$ARDUINO_PORT" ]; then
    print_error "No serial devices found!"
    print_info "Check USB connection: ls -l /dev/ttyACM* /dev/ttyUSB*"
    exit 1
fi

print_success "Arduino port: $ARDUINO_PORT"

# Step 3: Check permissions
print_info "Checking serial port permissions..."
if [ -r "$ARDUINO_PORT" ] && [ -w "$ARDUINO_PORT" ]; then
    print_success "Serial port permissions OK"
else
    print_error "No permission to access $ARDUINO_PORT"
    print_info "Run: sudo usermod -a -G dialout $USER"
    print_info "Then logout and login again"
    exit 1
fi

# Step 4: Prepare serial port
print_info "Preparing serial port..."
bash ~/qwacr_ws/src/qwacr_build/scripts/prepare_serial.sh $ARDUINO_PORT
if [ $? -eq 0 ]; then
    print_success "Serial port ready"
else
    print_error "Failed to prepare serial port"
    exit 1
fi

# Step 5: Show instructions
echo ""
echo "====================================="
echo "Setup Complete! Ready to launch."
echo "====================================="
echo ""
print_info "Arduino port: $ARDUINO_PORT"
echo ""
echo "To launch the robot control interface:"
echo ""
echo "  ros2 launch qwacr_build display.launch.py \\"
echo "      use_hardware:=true \\"
echo "      serial_port:=$ARDUINO_PORT \\"
echo "      baud_rate:=115200"
echo ""
echo "In another terminal, for keyboard control:"
echo ""
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard \\"
echo "      --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped"
echo ""
echo "====================================="
echo ""

# Step 6: Offer to launch now
read -p "Launch robot interface now? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    print_info "Launching robot interface..."
    echo ""
    ros2 launch qwacr_build display.launch.py \
        use_hardware:=true \
        serial_port:=$ARDUINO_PORT \
        baud_rate:=115200
else
    print_info "Launch manually when ready."
fi
