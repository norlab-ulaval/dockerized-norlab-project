#!/bin/bash
# =================================================================================================
# Build continuous integration test image specified in docker-compose.project.build.multiarch.yaml
# and execute tests on both x86 and arm64 architecture.
#
# Usage:
#   $ bash build.ci_tests.multiarch.bash [<any-build.all.multiarch-argument>]
#
# Positional argument:
#   <any-build.all.multiarch-argument>     (Optional)  build.all.multiarch.bash
#
# Notes:
#   The differences with executing `bash build.ci_tests.bash && bash run.ci_tests.bash` is that:
#     1. tests are executed at build time instead of at runtime;
#     2. the test image and its dependent images are rebuild;
#     3. tests are executed on both x86 and arm64 architecture.
#
# =================================================================================================
add_docker_flag=()
add_docker_flag+=("--service-names" "project-core,project-ci-tests,project-ci-tests-no-gpu")

# ....Path resolution..............................................................................
script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
script_path_parent="$(dirname "${script_path}")"

# ====Begin========================================================================================
bash "${script_path_parent}"/build.all.multiarch.bash "${add_docker_flag[@]}" "$@"
exit $?
