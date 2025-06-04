#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash" || exit 1
bash "${DNP_ROOT:?err}/tests/setup_mock.bash"

function dnp::teardown() {
  exit_code=$?
  cd "${DNP_ROOT:?err}"
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dnp::teardown EXIT

# ====begin========================================================================================
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNP_LIB_EXEC_PATH:?err}"/build.deploy.bash

# ....Teardown.....................................................................................
# Handle by the trap command


