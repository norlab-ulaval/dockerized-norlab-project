#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container runtime entrypoint callback.
# Is executed only once on container initialisation, at the end of the project-deploy entrypoints.
#
# Usage:
#   Add only project-deploy specific logic that need to be executed only on startup.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================

# ....DNA-project internal logic...................................................................
source /dna-lib-container-tools/entrypoints/entrypoint_helper.common.bash || exit 1

# ====DNA-project user defined logic===============================================================
# Add your code here
