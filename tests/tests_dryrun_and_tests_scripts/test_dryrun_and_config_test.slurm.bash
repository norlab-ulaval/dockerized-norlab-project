#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash" || exit 1
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
bash "${DNP_LIB_EXEC_PATH:?err}"/dryrun_and_config_test.slurm.bash ".dockerized_norlab_project/slurm_jobs"
EXIT_CODE=$?

# ....Teardown.....................................................................................
rm -rf "${DNP_ROOT:?err}/dockerized-norlab-project-mock/artifact/mock_experiment_tmp/"
rm -f "${DNP_ROOT}/dockerized-norlab-project-mock/artifact/optuna_storage/mock_experiment_tmp.db"

exit $EXIT_CODE
