#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash" || exit 1
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
bash "${DNP_LIB_EXEC_PATH:?err}"/build.develop.bash
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

bash "${DNP_LIB_EXEC_PATH:?err}"/up_and_attach.bash --service project-develop -- tree -L 1
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

bash "${DNP_LIB_EXEC_PATH:?err}"/down.bash
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

