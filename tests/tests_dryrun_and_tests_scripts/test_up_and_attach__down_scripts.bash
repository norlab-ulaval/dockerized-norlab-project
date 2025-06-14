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
bash "${DNP_LIB_EXEC_PATH:?err}"/build.develop.bash


bash "${DNP_LIB_EXEC_PATH:?err}"/up_and_attach.bash --service project-develop -- bash -c "echo -e \"\nExecute up and attach test command\n\" && tree -L 2 -a \$(pwd)"

bash "${DNP_LIB_EXEC_PATH:?err}"/down.bash


