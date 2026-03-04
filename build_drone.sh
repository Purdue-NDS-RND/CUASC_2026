#!/bin/bash
# get all packages with the name starting with drone_*

packages=$(ls src | grep '^drone_')
echo "Building packages: \n $packages"

echo 


for package in $packages; do
    colcon build --packages-select $package
done


source install/setup.bash