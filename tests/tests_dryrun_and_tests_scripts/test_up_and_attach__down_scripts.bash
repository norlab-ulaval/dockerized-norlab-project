#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
bash "${DNP_LIB_EXEC_PATH:?err}"/build.develop.bash
exit_code=$?
if [[ ${exit_code} != 0 ]]; then
  exit $exit_code
fi

bash "${DNP_LIB_EXEC_PATH:?err}"/up_and_attach.bash --service project-develop -- tree -L 1
exit_code=$?
if [[ ${exit_code} != 0 ]]; then
  exit $exit_code
fi

bash "${DNP_LIB_EXEC_PATH:?err}"/down.bash
exit_code=$?
if [[ ${exit_code} != 0 ]]; then
  exit $exit_code
fi

