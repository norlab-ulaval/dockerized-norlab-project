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

# ....Examples: source ROS2 environment variables..................................................
#dn::source_ros2_underlay_only
#dn::source_ros2_overlay_only
dn::source_ros2

# ....Show N2ST and DN librairy available functions................................................
# To check available N2ST lib and DN lib functions, un-comment te following lines
#n2st::print_msg "Show in container available N2ST functions...\n${MSG_DIMMED_FORMAT}"
#for func in $(compgen -A function | grep -e n2st::); do
#  # shellcheck disable=SC2163
#  echo "   ${func}"
#done
#echo -e "${MSG_END_FORMAT}"
#n2st::print_msg "Show in container available DN functions...\n${MSG_DIMMED_FORMAT}"
#for func in $(compgen -A function | grep -e dn::); do
#  # shellcheck disable=SC2163
#  echo "   ${func}"
#done
