#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container runtime entrypoint callback.
# Is executed each time a shell is attach to a project-develop or project-deploy container.
#
# Usage:
#   Add project wide logic that need to be executed by each shell.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================

# ....DN-project internal logic....................................................................
source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.global.common.bash || exit 1

# ====DN-project user defined logic================================================================
# Add your code here

# ....Examples: source ROS2 environment variables..................................................
#dn::source_ros2_underlay_only
#dn::source_ros2_overlay_only
dn::source_ros2
