#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container project-ci-tests service entrypoint callback (native and multiarch).
# Is executed only once on container initialisation, at the begining of the project-ci-tests entrypoints.
#
# Usage:
#   Add only project-ci-tests specific logic that need to be executed only on startup.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================

# ....DN-project internal logic....................................................................
source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.common.bash

# ====DN-project user defined logic================================================================
# Add your code here
