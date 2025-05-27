#!/bin/bash
# =================================================================================================
# Build continuous integration tests images specified in docker-compose.project.build.native.yaml.
# Use in conjonction with `run.ci_tests.bash`.
#
# Usage:
#   $ bash build.ci_tests.bash  [<any-build.all-argument>]
#   $ bash run.ci_tests.bash [<any-docker-argument>]
#
# Notes:
#   The difference with `build.ci_tests.multiarch.bash` is that tests are only executed for the
#   host architecture, either x86 or arm64 and are executed at runtime (using `run.ci_tests.bash`)
#   instead of at build time.
#
# =================================================================================================
BUILD_ALL_FLAG=()
BUILD_ALL_FLAG+=("--service-names" "project-core,project-ci-tests,project-ci-tests-no-gpu")
# Note: force push project core is set to false by default

# ....Path resolution..............................................................................
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"

# ====Begin========================================================================================
bash "${SCRIPT_PATH_PARENT}"/build.all.bash "${BUILD_ALL_FLAG[@]}" "$@" || exit 1
exit $?
