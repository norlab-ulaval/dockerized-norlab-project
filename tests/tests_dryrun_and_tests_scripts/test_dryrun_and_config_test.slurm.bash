#!/bin/bash

# ....Setup........................................................................................
DNP_ROOT=$(git rev-parse --show-toplevel)
DNP_LIB_EXEC_PATH="${DNP_ROOT}/src/lib/core/execute"
SUPER_PROJECT_ROOT="${DNP_ROOT}/mock-user-super-project"
cd "$SUPER_PROJECT_ROOT" || exit 1

# ====begin========================================================================================
export DNP_LIB_EXEC_PATH
bash "${DNP_LIB_EXEC_PATH}"/dryrun_and_config_test.slurm.bash ".dockerized_norlab_project/slurm_jobs"
EXIT_CODE=$?

# ....Teardown.....................................................................................
rm -rf "${DNP_ROOT}/mock-user-super-project/artifact/mock_experiment_tmp/"
rm -f "${DNP_ROOT}/mock-user-super-project/artifact/optuna_storage/mock_experiment_tmp.db"

exit $EXIT_CODE
