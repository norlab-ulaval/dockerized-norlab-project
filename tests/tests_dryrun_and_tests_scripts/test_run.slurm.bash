#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================

# Script locals env var
declare -a flags
declare -a hydra_flags

# Exported env var
declare -x SJOB_ID

SJOB_ID="default"
# ....Debug flags..................................................................................
#flags+=("--hydra-dry-run")
#flags+=(--register-hydra-dry-run-flag "+dev@_global_=math_env_slurm_job_dryrun")
flags+=("--skip-core-force-rebuild")
hydra_flags+=("--version")

bash "${DNP_LIB_EXEC_PATH:?err}"/run.slurm.bash "${SJOB_ID}" "${flags[@]}" "${hydra_flags[@]}"
exit_code=$?

if [[ ${exit_code} != 0 ]]; then
  # Make sure there is no slurm container running
  bash "${DNP_LIB_EXEC_PATH:?err}"/down.slurm.bash >/dev/null
  exit $exit_code
fi

echo "test_run.slurm.bash DONE"
