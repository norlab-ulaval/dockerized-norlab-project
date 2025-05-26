#!/bin/bash

# ....path resolution logic........................................................................
SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
cd "${SUPER_PROJECT_ROOT}"/.dockerized_norlab_project/execute

# ====begin========================================================================================
bash dryrun_and_config_test.slurm.bash
exit $?
