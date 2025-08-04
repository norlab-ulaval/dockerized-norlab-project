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
#set -e

# ....Sanity check.................................................................................
test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DNA error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f dn::source_ros2 )" || { echo -e "\033[1;31m[DNA error]\033[0m The DN lib is not loaded!" 1>&2 && exit 1; }

# ....Debug logic..................................................................................
if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  n2st::print_msg "Execute ${BASH_SOURCE[0]}"
fi

## (CRITICAL) ToDo: validate >> deleting DN lib import ↓ (ref task NMO-770)
## ....Load DN lib explicitly.......................................................................
#source /dockerized-norlab/dockerized-norlab-images/container-tools/bash_run_config/.bashrc.dn_common

# ....ROS2 logic...................................................................................
# ToDo: assess refactoring out to a dedicated 'entrypoint_helper.ros2.bash' ⬇︎
if [[ -n "${ROS_DISTRO}" ]]; then
  # Should be executed before using ROS2
  pkg_list=$(dn::source_ros2 && ros2 pkg list 2>/dev/null) || pkg_list=""
  if ! echo "$pkg_list" | grep -q "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"; then
      n2st::print_msg_warning "${RMW_IMPLEMENTATION} not found, falling back to default RMW"
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  fi
fi
