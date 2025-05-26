#!/bin/bash

# ====Custom steps=================================================================================
PYTEST_FLAG=()

# ....Required flags...............................................................................
PYTEST_FLAG+=(--rootdir="${DN_PROJECT_PATH}/tests")

# ....Optional flags...............................................................................
PYTEST_FLAG+=(--config-file="${DN_PROJECT_PATH}/tests/pytest.ini")
# pytest-xdist is configured through the pytest.ini file

PYTEST_FLAG+=(--reruns 5 --reruns-delay 2.5)
#PYTEST_FLAG+=(--verbose)

# (CRITICAL) ToDo: on task end >> mute this line ↓
#PYTEST_FLAG+=(--exitfirst) # Exit instantly on first error or failed test

# ....Add per project specific flag................................................................
PYTEST_FLAG+=(--ignore="${DN_PROJECT_PATH}/src/ros2_packages")
PYTEST_FLAG+=(-k 'not tests_pipeline and not tests_launcher') # Skip those tests

# ====Execute pytest command=======================================================================
n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}pytest ${PYTEST_FLAG[*]} ${DN_PROJECT_PATH}/tests${MSG_END_FORMAT}\n"

pytest "${PYTEST_FLAG[@]}" "${DN_PROJECT_PATH}/tests" || exit 1
exit $?

