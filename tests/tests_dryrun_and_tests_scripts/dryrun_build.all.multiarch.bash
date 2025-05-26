#!/bin/bash

# ....path resolution logic........................................................................
SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
cd "${SUPER_PROJECT_ROOT}"/.dockerized_norlab_project/execute

# ====begin========================================================================================
bash build.all.multiarch.bash --no-force-push-project-core -- --dry-run
exit $?
