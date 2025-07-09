#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"

function dna::test_teardown_callback() {
  exit_code=$?
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dna::test_teardown_callback EXIT

# ====begin========================================================================================
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

if [[ ${TEAMCITY_VERSION} ]] ; then
  bash "${DNA_LIB_EXEC_PATH:?err}"/build.all.bash "$@" -- --no-cache
else
  bash "${DNA_LIB_EXEC_PATH:?err}"/build.all.bash "$@"
fi

# ....Teardown.....................................................................................
# Handle by the trap command



