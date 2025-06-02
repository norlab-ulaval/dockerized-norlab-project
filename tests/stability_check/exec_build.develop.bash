#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash" || exit 1
bash "${DNP_ROOT:?err}/tests/setup_mock.bash"

function dnp::teardown() {
  EXIT_CODE=$?
  cd "${DNP_ROOT:?err}"
  bash tests/teardown_mock.bash
  exit ${EXIT_CODE:1}
}
trap dnp::teardown EXIT

# ====begin========================================================================================
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNP_LIB_EXEC_PATH:?err}"/build.develop.bash

# ....Teardown.....................................................................................
# Handle by the trap command
