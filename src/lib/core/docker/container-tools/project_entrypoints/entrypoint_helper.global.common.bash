#!/bin/bash
# =================================================================================================
# Helper script executed by both "dn_entrypoint.globals.*.callback.bash".
#
# Usage:
#   source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.common.bash
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
#set -e

#DN_SHOW_DEBUG_INFO=false # (CRITICAL) ToDo: on task end >> mute this line ←

function _show_debug_info() {
  local caller="$1"
  echo -e "\033[1;33m[DN trace]\033[0m Info from $1 \033[1;2m
  BASH_SOURCE: ${BASH_SOURCE[*]}
  realpath: $(realpath .)
  \$0: $0
  \033[0m"
}

# ....Sanity check.................................................................................
test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f dn::source_ros2 )" || { echo -e "\033[1;31m[DN error]\033[0m The DN lib is not loaded!" 1>&2 && exit 1; }

# ....Debug logic..................................................................................
if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  echo -e "\033[1;33m[DN trace]\033[0m Execute entrypoint_helper.global.common.bash from ${BASH_SOURCE[1]}"
  if [[ "${DN_SHOW_DEBUG_INFO}" == true ]]; then
    _show_debug_info "script entrypoint_helper.common.bash"
  fi
fi

# ....ROS2 logic...................................................................................
# ToDo: assess refactoring out to a dedicated 'entrypoint_helper.ros2.bash' ⬇︎
if [[ -n "${ROS_DISTRO}" ]]; then
  # Should be executed before using ROS2
  pkg_list=$(dn::source_ros2 && ros2 pkg list 2>/dev/null) || pkg_list=""
  if ! echo "$pkg_list" | grep -q "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"; then
      n2st::print_msg_warning "${RMW_IMPLEMENTATION} not found, falling back to default RMW"
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  else
    if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
      echo -e "\033[1;33m[DN trace]\033[0m Found ${RMW_IMPLEMENTATION}"
    fi
  fi
else
  if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
    echo -e "\033[1;33m[DN trace]\033[0m Missing env var ROS_DISTRO=${ROS_DISTRO} -> skipping RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} check!"
  fi
fi
