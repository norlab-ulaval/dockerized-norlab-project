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
  echo -e "\033[1;33m[DN trace]\033[0m Execute entrypoint_helper.common.bash from ${BASH_SOURCE[1]}"
fi
