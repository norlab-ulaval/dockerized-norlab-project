#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash"
bash "${DNP_ROOT:?err}/tests/setup_mock.bash"

function dnp::execute_on_exit() {
  cd "${DNP_ROOT:?err}"
  bash tests/teardown_mock.bash
  exit ${EXIT_CODE:1}
}
trap dnp::execute_on_exit EXIT

# ====begin========================================================================================
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNP_LIB_EXEC_PATH:?err}"/build.develop.bash
EXIT_CODE=$?

# ....Teardown.....................................................................................
# Handle by the trap command
