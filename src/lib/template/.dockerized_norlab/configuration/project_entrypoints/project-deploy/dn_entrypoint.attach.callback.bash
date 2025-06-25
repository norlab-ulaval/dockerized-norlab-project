#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container runtime entrypoint callback.
# Is executed each time a shell is attach to a project-deploy container.
#
# Usage:
#   Add only project-deploy specific logic that need to be executed by each shell.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================

# ....DN-project internal logic....................................................................
source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.common.bash

# ====DN-project user defined logic================================================================
# Add your code here
