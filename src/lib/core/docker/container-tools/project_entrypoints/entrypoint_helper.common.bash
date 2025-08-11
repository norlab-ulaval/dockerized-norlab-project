#!/bin/bash
# =================================================================================================
# Helper script executed by all "project-*/dn_entrypoint.*.callback.bash", but not global ones.
#
# Usage:
#   source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.common.bash
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================

# ....Debug logic..................................................................................
if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  echo -e "\033[1;33m[DN trace]\033[0m Execute $(basename "${BASH_SOURCE[1]}") -> entrypoint_helper.common.bash"
fi

# ....Sanity check.................................................................................
test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f dn::source_ros2 )" || { echo -e "\033[1;31m[DN error]\033[0m The DN lib is not loaded!" 1>&2 && exit 1; }
