#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"
function dna::test_teardown_callback() {
  local exit_code=$?
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:-1}
}
trap dna::test_teardown_callback EXIT

# ====begin========================================================================================

# Re-build slurm image (required on TC to prevent ownership error related to agent switching)
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNA_LIB_EXEC_PATH:?err}"/build.all.bash --service-names project-slurm -- --no-cache

# Execute project validate slurm script
# Note: the "--include-multiarch" flag affect only the dry-run config check, not the slurm job check
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNA_LIB_EXEC_PATH:?err}"/project_validate.slurm.bash --include-multiarch "slurm_jobs"

## ....Teardown.....................................................................................
#rm -rf "${DNA_ROOT:?err}/dockerized-norlab-project-mock/artifact/mock_experiment_tmp/"
#rm -f "${DNA_ROOT}/dockerized-norlab-project-mock/artifact/optuna_storage/mock_experiment_tmp.db"
