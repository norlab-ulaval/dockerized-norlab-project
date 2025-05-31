#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container runtime entrypoint callback.
# Is executed each time a shell is attach to a project-develop or project-deploy container.
#
# Usage:
#   Add project wide logic that need to be executed by each shell.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================

# ....DN-project internal logic....................................................................
source /dnp-lib-container-tools/project_entrypoints/entrypoint_helper.global.init.bash

# ====DN-project user defined logic================================================================
# Add your code here

