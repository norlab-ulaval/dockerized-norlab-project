#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
bash "${DNP_ROOT:?err}/tests/setup_mock.bash"
function dnp::trap_teardown_callback() {
  exit_code=$?
  cd "${DNP_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dnp::trap_teardown_callback EXIT

cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
bash "${DNP_LIB_EXEC_PATH:?err}"/build.ci_tests.multiarch.bash -- --dry-run

