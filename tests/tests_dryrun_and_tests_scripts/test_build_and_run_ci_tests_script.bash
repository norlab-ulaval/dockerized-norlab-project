#!/bin/bash

# ....path resolution logic........................................................................
SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
cd "${SUPER_PROJECT_ROOT}"/.dockerized_norlab_project/execute

# ====begin========================================================================================
bash build.ci_tests.bash

bash run.ci_tests.bash
exit $?
