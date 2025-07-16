#!/bin/bash
# =================================================================================================
# Helper script executed by all dn_entrypoint.*.callback.bash.
#
# Usage:
#   source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.common.bash
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e

if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  n2st::print_msg "Execute ${BASH_SOURCE[0]}"
fi

if [[ -n "${ROS_DISTRO}" ]]; then
  source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bashrc_non_interactive.bash

  # Should be executed before sourcing ROS2
  if ! ros2 pkg list | grep -q "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"; then
      echo "Warning: ${RMW_IMPLEMENTATION} not found, falling back to default RMW"
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  fi
fi
