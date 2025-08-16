#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"

function dna::test_teardown_callback() {
  local exit_code=$?

  bash "${DNA_LIB_EXEC_PATH:?err}"/down.bash

  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash

  exit ${exit_code:-1}
}
trap dna::test_teardown_callback EXIT

cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
echo "Set port to non default value to mitigate test time collision..."
export DN_SSH_SERVER_PORT=2299
export DN_GDB_SERVER_PORT=7799
export DN_TENSORBOARD=6099
export DN_OPTUNA_DASHBOARD=8899
echo "
  DN_SSH_SERVER_PORT: $DN_SSH_SERVER_PORT
  DN_GDB_SERVER_PORT: $DN_GDB_SERVER_PORT
  DN_TENSORBOARD: $DN_TENSORBOARD
  DN_OPTUNA_DASHBOARD: $DN_OPTUNA_DASHBOARD
"
bash "${DNA_LIB_EXEC_PATH:?err}"/build.develop.bash

bash "${DNA_LIB_EXEC_PATH:?err}"/up_and_attach.bash --service project-develop -- bash -c "echo -e \"\nExecute up and attach test command\nWe are in! Execute tree command...\n\" && tree -L 2 -a \$(pwd) && echo \$(printenv | grep -e DN_SSH_ -e DN_GDB_)"

unset DN_SSH_SERVER_PORT
unset DN_GDB_SERVER_PORT
unset DN_TENSORBOARD
unset DN_OPTUNA_DASHBOARD
