#!/bin/bash

# Define the vehicle type to build for
vehicle="copter" # Adjust this as necessary

# Obtain all targets, filter those starting with 'Q'
targets=$(./waf list_boards | grep '^Q')

# Check if targets variable is empty
if [ -z "$targets" ]; then
    echo "No targets starting with 'Q' found."
    exit 1
fi

# Convert string to array
readarray -t targetsArray <<< "$targets"

# Loop through each target starting with 'Q'
for target in "${targetsArray[@]}"
do
    echo "Processing target: $target"

    # Configure waf for the target board
    configure_output=$(./waf configure --board $target 2>&1)
    if ! echo "$configure_output" | grep -q 'Setting top to'; then
        echo "Configuration failed for target: $target"
        echo "$configure_output"
        continue # Skip to the next target
    fi

    # Attempt to build firmware
    build_output=$(./waf $vehicle 2>&1)
    
    # Check for bootloader error
    if echo "$build_output" | grep -q "Bootloader (.*) does not exist and AP_BOOTLOADER_FLASHING_ENABLED"; then
        # Extract the specific bootloader build command from the error message
        bootloader_build_command=$(echo "$build_output" | grep -oP "Please run: \K.*")

        # Check if a bootloader build command is extracted
        if [ ! -z "$bootloader_build_command" ]; then
            echo "Bootloader build required for $target. Executing: $bootloader_build_command"
            eval "$bootloader_build_command"

            # Retry building the firmware after building the bootloader
            echo "Retrying firmware build for $target after bootloader build."
            ./waf configure --board $target && ./waf $vehicle
        else
            echo "Bootloader build command extraction failed for target: $target"
        fi
    elif echo "$build_output" | grep -q 'Finished building'; then
        echo "Build successful for target: $target"
    else
        echo "Build failed for target: $target"
        echo "$build_output"
    fi
done

echo "Build process for all 'Q' targets completed."
