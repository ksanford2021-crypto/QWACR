#!/bin/bash
# Simple script to send velocity commands to Arduino via serial

PORT="/dev/ttyACM0"
BAUD=115200

# Function to send raw bytes
send_bytes() {
    local bytes="$@"
    echo -ne "$(echo "$bytes" | sed 's/0x/\\x/g')" > "$PORT"
}

# Function to create COBS-encoded command
# Command format: 0x01 + left_vel (4 bytes float) + right_vel (4 bytes float)
send_velocity() {
    local left=$1
    local right=$2
    
    # This is a simplified version - would need Python for proper float encoding
    echo "To send velocity command for left=$left, right=$right:"
    echo "Use Python script: python3 test_motor_command.py"
}

case "$1" in
    "test")
        echo "Sending test commands..."
        # Test: left motor 30 ticks/sec, right motor 0
        # This requires proper byte encoding (handled by Python script)
        echo "Use: python3 test_motor_command.py"
        ;;
    *)
        echo "Usage: $0 test"
        echo "For now, use: python3 test_motor_command.py"
        ;;
esac
