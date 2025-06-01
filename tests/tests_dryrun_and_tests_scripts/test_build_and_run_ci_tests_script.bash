#!/bin/bash

# ....Setup........................................................................................
DNP_ROOT=$(git rev-parse --show-toplevel)
DNP_LIB_EXEC_PATH="${DNP_ROOT}/src/lib/core/execute"
SUPER_PROJECT_ROOT="${DNP_ROOT}/dockerized-norlab-project-mock"

# ====begin========================================================================================
cd "$SUPER_PROJECT_ROOT" || exit 1
bash "${DNP_LIB_EXEC_PATH}"/build.ci_tests.bash
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

cd "$SUPER_PROJECT_ROOT" || exit 1
bash "${DNP_LIB_EXEC_PATH}"/run.ci_tests.bash
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi
