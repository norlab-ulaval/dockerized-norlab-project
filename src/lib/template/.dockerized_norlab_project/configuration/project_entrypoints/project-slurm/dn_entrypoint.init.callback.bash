#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container project-slurm service entrypoint callback.
# Is executed only once on container initialisation, at the begining of the project-slurm entrypoints.
#
# Usage:
#   Add only project-slurm specific logic that need to be executed only on startup.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================

# ....DN-project internal logic....................................................................
source /dnp-lib-container-tools/project_entrypoints/entrypoint_helper.common.bash


# ====DN-project user defined logic================================================================
# Add your code here
