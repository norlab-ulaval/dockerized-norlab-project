#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash" || exit 1

# ====begin========================================================================================
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNP_LIB_EXEC_PATH:?err}"/build.ci_tests.bash
exit_code=$?
if [[ ${exit_code} != 0 ]]; then
  exit $exit_code
fi

cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNP_LIB_EXEC_PATH:?err}"/run.ci_tests.bash
exit_code=$?
if [[ ${exit_code} != 0 ]]; then
  exit $exit_code
fi
