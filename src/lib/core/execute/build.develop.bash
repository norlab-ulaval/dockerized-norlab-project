#!/bin/bash
# =================================================================================================
# Build develop images specified in docker-compose.project.build.native.yaml
#
# Remark:
#   Develop images are build localy and are single architecture so they are not pushed to dockerhub
#
# Usage:
#   $ bash build.develop.bash  [<any-build.all-argument>]
#
# =================================================================================================
ADD_DOCKER_FLAG=("--service-names" "project-core,project-develop")

# ....Path resolution..............................................................................
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"

# ====Begin========================================================================================
bash "${SCRIPT_PATH_PARENT}"/build.all.bash "${ADD_DOCKER_FLAG[@]}" "$@"
exit $?
