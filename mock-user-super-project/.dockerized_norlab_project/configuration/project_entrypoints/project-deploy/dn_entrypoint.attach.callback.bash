#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container up_and_attach.bash entrypoint callback.
# Is executed each time a shell is attach to a project-deploy container.
#
# Usage:
#   Add only project-deploy specific logic that need to be executed by each shell.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e

# ====DN-project internal logic====================================================================
if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  n2st::print_msg "Execute ${BASH_SOURCE[0]}"
fi

# ====DN-project user defined logic================================================================
# Add your code here
