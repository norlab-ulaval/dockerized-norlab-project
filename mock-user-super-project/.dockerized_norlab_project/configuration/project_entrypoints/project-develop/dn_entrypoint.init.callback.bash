#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container up_and_attach.bash entrypoint callback.
# Is executed only once on container initialisation, at the end of the project-develop entrypoints.
#
# Usage:
#   Add only project-develop specific logic that need to be executed only on startup.
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
