#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container runtime entrypoint callback.
# Is executed only once on container initialisation, at the end of the project-develop or
# project-deploy entrypoints.
#
# Usage:
#   Add project wide logic that need to be executed only on startup.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================

# ....DNA-project internal logic...................................................................
source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.global.common.bash || exit 1
source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.global.init.bash || exit 1

# ====DNA-project user defined logic===============================================================
# Add your code here

# ....DNA-project optional logic...................................................................
source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.show_info.bash || exit 1

# To check available N2ST lib and DN lib functions, un-comment the following lines
#dn::show_dn_and_n2st_available_functions

# ....Examples: source ROS2 environment variables..................................................
#dn::source_ros2_underlay_only
#dn::source_ros2_overlay_only
dn::source_ros2
