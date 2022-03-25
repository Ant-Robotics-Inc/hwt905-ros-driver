#!/bin/bash

set -e
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

# Setup ROS environment variables if inside a container
key="$1"
case $key in
    -d|--docker)
        source /opt/ros/$ROS_DISTRO/setup.bash
        source /catkin_ws/devel/setup.bash
        ;;
*)
    ;;
esac

# Run tests
PACKAGE_NAME=inclinometer
catkin run_tests --no-deps $PACKAGE_NAME

# catkin_tools issue #245. catkin run_tests may return 0 when fail.
# https://github.com/catkin/catkin_tools/issues/245
# Check report directory to verify that tests are ok.
roscd && cd ../build/
CATKIN_TESTS_FAILURE_RESULTS_PATH=inclinometer/test_results/hwt905_test/MISSING-*
if [ ! -z "$(ls -A $CATKIN_TESTS_FAILURE_RESULTS_PATH)" ]; then
    echo "Failure results have been found. Exit with error."
    exit -1
fi
