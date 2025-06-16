#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
bash "${DNP_ROOT:?err}/tests/setup_mock.bash"
function dnp::test_teardown_callback() {
  exit_code=$?
  cd "${DNP_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dnp::test_teardown_callback EXIT

cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
export PATH="$PATH:${DNP_PATH:?err}"

bash "${DNP_LIB_EXEC_PATH:?err}"/project_validate.slurm.bash ".dockerized_norlab_project/slurm_jobs"

## ....Teardown.....................................................................................
#rm -rf "${DNP_ROOT:?err}/dockerized-norlab-project-mock/artifact/mock_experiment_tmp/"
#rm -f "${DNP_ROOT}/dockerized-norlab-project-mock/artifact/optuna_storage/mock_experiment_tmp.db"
