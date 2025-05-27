#!/bin/bash

# ....Setup........................................................................................
DNP_ROOT=$(git rev-parse --show-toplevel)
DNP_LIB_EXEC_PATH="${DNP_ROOT}/src/lib/core/execute"
SUPER_PROJECT_ROOT="${DNP_ROOT}/mock-user-super-project"
cd "$SUPER_PROJECT_ROOT" || exit 1

# ====begin========================================================================================
bash "${DNP_LIB_EXEC_PATH}"/build.develop.bash
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

bash "${DNP_LIB_EXEC_PATH}"/up_and_attach.bash --service project-develop -- tree -L 1
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

bash "${DNP_LIB_EXEC_PATH}"/down.bash
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

