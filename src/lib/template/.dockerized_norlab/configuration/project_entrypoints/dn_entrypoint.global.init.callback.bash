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

# ....DN-project internal logic....................................................................
source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.global.init.bash || exit 1
source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.show_info.bash || exit 1

# ====DN-project user defined logic================================================================

# ....Project specific info........................................................................
echo -e "Project ${DN_PROJECT_GIT_NAME:?err} specific information: ${MSG_DIMMED_FORMAT}
\n$(
  SP="    " \
  && pip --disable-pip-version-check list | grep -i -e hydra -e omegaconf | sed "s;hydra;${SP}hydra;" | sed "s;omega;${SP}omega;" \
  && pip --disable-pip-version-check list --exclude hydra-optuna-sweeper | grep -i -e optuna | sed "s;^optuna;${SP}optuna;"
)
${MSG_END_FORMAT}"

# ....Examples: source ROS2 environment variables..................................................
#dn::source_ros2_underlay_only
#dn::source_ros2_overlay_only
dn::source_ros2
