#!/bin/bash
# =================================================================================================
# Build continuous integration tests images specified in docker-compose.project.build.native.yaml.
# Use in conjonction with `run.ci_tests.bash`.
#
# Usage:
#   $ bash build.ci_tests.bash  [<any-build.all-argument>]
#   $ bash run.ci_tests.bash [<any-docker-argument>]
#
# Positional argument:
#   <any-build.all-argument>               (Optional) Any build.all.bash arguments
#   <any-docker-flag>                      (Optional) Any docker flag
#
# Notes:
#   The difference with `build.ci_tests.multiarch.bash` is that tests are only executed for the
#   host architecture, either x86 or arm64 and are executed at runtime (using `run.ci_tests.bash`)
#   instead of at build time.
#
# =================================================================================================
# Note: force push project core is set to false by default

build_all_flag=()
build_all_flag+=("--service-names" "project-core,project-ci-tests,project-ci-tests-no-gpu")

# ....Path resolution..............................................................................
script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
script_path_parent="$(dirname "${script_path}")"

# ====Begin========================================================================================
bash "${script_path_parent}"/build.all.bash "${build_all_flag[@]}" "$@" || exit 1
exit $?
