#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNP_PATH:?err}"
bash "${DNP_ROOT:?err}/tests/setup_mock.bash"

function dnp::test_teardown_callback() {
  exit_code=$?
  cd "${DNP_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dnp::test_teardown_callback EXIT

# ====begin========================================================================================
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNP_LIB_EXEC_PATH:?err}"/build.ci_tests.bash -- --no-cache

# ....Teardown.....................................................................................
# Handle by the trap command



