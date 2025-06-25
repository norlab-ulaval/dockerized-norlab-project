#!/bin/bash

# ====Custom steps=================================================================================
PYTEST_FLAG=()

# ....Required flags...............................................................................
PYTEST_FLAG+=(--rootdir="${DN_PROJECT_PATH}/tests")

# ....Optional flags...............................................................................
PYTEST_FLAG+=(--config-file="${DN_PROJECT_PATH}/tests/pytest.dna_no_xdist.ini")

# pytest-xdist manual setting
# ref: https://pytest-xdist.readthedocs.io
PYTEST_FLAG+=(--dist loadgroup)
PYTEST_FLAG+=(--numprocesses auto --maxprocesses=2) # Limit CPU process to prevent GPU overload.

PYTEST_FLAG+=(--reruns 5 --reruns-delay 2.5)
#PYTEST_FLAG+=(--verbose)
#PYTEST_FLAG+=(--exitfirst) # Exit instantly on first error or failed test

# ....Add per project specific flag................................................................
#PYTEST_FLAG+=(--ignore="${DN_PROJECT_PATH}/src/ros2_packages")
PYTEST_FLAG+=(-k "test_try_pytorch") # Only run test matching this expression

# ====Execute pytest command=======================================================================
n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}pytest ${PYTEST_FLAG[*]} ${DN_PROJECT_PATH}/tests${MSG_END_FORMAT}\n"

pytest "${PYTEST_FLAG[@]}" "${DN_PROJECT_PATH}/tests" || exit 1
exit $?
