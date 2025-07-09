#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"
function dna::test_teardown_callback() {
  exit_code=$?
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dna::test_teardown_callback EXIT

cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
bash "${DNA_LIB_EXEC_PATH:?err}"/project_validate.slurm.bash --include-multiarch "slurm_jobs"

## ....Teardown.....................................................................................
#rm -rf "${DNA_ROOT:?err}/dockerized-norlab-project-mock/artifact/mock_experiment_tmp/"
#rm -f "${DNA_ROOT}/dockerized-norlab-project-mock/artifact/optuna_storage/mock_experiment_tmp.db"
