#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"
function dna::test_teardown_callback() {
  local exit_code=$?
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:-1}
}
trap dna::test_teardown_callback EXIT

# ====begin========================================================================================

# Create a mock apptainer in a temporary directory and add it to PATH
# This satisfies the version check in the Slurm templates (Apptainer >= 1.1.0)
mock_bin_dir=$(mktemp -d)
cat > "${mock_bin_dir}/apptainer" <<'EOF'
#!/bin/bash
if [[ "$1" == "version" ]]; then
  echo "apptainer version 1.1.0"
else
  # Do nothing for other commands
  exit 0
fi
EOF
chmod +x "${mock_bin_dir}/apptainer"
export PATH="${mock_bin_dir}:${PATH}"

function dna::cleanup_mock_apptainer() {
  rm -rf "${mock_bin_dir}"
}
trap "dna::test_teardown_callback; dna::cleanup_mock_apptainer" EXIT

# Re-build slurm image (required on TC to prevent ownership error related to agent switching)
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNA_LIB_EXEC_PATH:?err}"/build.all.bash --service-names project-slurm -- --no-cache

# Execute project validate slurm script
# Note: the "--include-multiarch" flag affect only the dry-run config check, not the slurm job check
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
bash "${DNA_LIB_EXEC_PATH:?err}"/project_validate.slurm.bash --include-multiarch "slurm_jobs"

## ....Teardown.....................................................................................
#rm -rf "${DNA_ROOT:?err}/dockerized-norlab-project-mock/artifact/mock_experiment_tmp/"
#rm -f "${DNA_ROOT}/dockerized-norlab-project-mock/artifact/optuna_storage/mock_experiment_tmp.db"
