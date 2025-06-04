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
# Positional argument:
#   <any-build.all-argument>               (Optional) Any build.all.bash arguments
#
# =================================================================================================
add_docker_flag=("--service-names" "project-core,project-develop")

# ....Path resolution..............................................................................
script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
script_path_parent="$(dirname "${script_path}")"

# ====Begin========================================================================================
bash "${script_path_parent}"/build.all.bash "${add_docker_flag[@]}" "$@"
exit $?
