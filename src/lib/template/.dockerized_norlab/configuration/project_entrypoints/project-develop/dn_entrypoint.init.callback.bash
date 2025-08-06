#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container runtime entrypoint callback.
# Is executed only once on container initialisation, at the end of the project-develop entrypoints.
#
# Usage:
#   Add only project-develop specific logic that need to be executed only on startup.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================

# ....DN-project internal logic....................................................................
source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.common.bash || exit 1

# ====DN-project user defined logic================================================================
# Add your code here

# ....Example......................................................................................
if [[ $( n2st::which_architecture_and_os ) == "l4t\arm64" ]]; then
  n2st::print_msg "Is running on a Jetson..."
  # Add Jetson logic e.g., cat /proc/device-tree/model
fi
# .................................................................................................
