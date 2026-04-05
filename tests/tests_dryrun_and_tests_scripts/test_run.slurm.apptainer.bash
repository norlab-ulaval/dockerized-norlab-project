#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"
function dna::test_teardown_callback() {
  exit_code=$?
  if [[ ${exit_code} != 0 ]]; then
    # Make sure there is no slurm container running
    bash "${DNA_LIB_EXEC_PATH:?err}"/down.slurm.bash >/dev/null
  fi
  echo "test_run.slurm.apptainer.bash DONE"
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:-1}
}
trap dna::test_teardown_callback EXIT

cd "${DNA_ROOT:?err}/tests/tests_containerized_apptainer" || exit 1

# ====begin========================================================================================
unset BUILDX_BUILDER

bash run_containerized_apptainer_tests.bash

