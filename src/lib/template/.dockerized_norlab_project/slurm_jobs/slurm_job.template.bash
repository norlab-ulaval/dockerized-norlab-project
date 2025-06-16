#!/bin/bash
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=12
#SBATCH --time=7-00:00
#SBATCH --output=out/%x-%j.out
#SBATCH --mail-type=ALL
#SBATCH --mail-user=luc.coupal@norlab.ulaval.ca

# Note: Flag time format --time=D-HH:MM ->  D=day, HH=hours, MM=minutes

# =================================================================================================
# Execute slurm job
#
# Usage:
#   $ bash slurm_job.template.bash [<any-python-args>]
#
# =================================================================================================
declare -x SJOB_ID
declare -a dnp_run_slurm_flags=()
declare -a python_arguments=()

# ====Setup========================================================================================
# ....Custom setup (optional)......................................................................
function dnp::job_setup_callback() {
  # Add any instruction that should be executed before 'dnp run slurm' command
  :
}

# ....Custom teardown (optional)...................................................................
function dnp::job_teardown_callback() {
  local exit_code=$?
  # TODO: Add any instruction that should be executed after 'dnp run slurm' exit.

  # Note: Command 'dnp run slurm' already handle stoping the container in case the slurm command
  #  `scancel` is issued.
  exit ${exit_code:1}
}

# ....Set job name.................................................................................
# TODO: Set SJOB_ID
SJOB_ID="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an SJOB_ID.

# ....Python module................................................................................
# TODO: Set python module to launch
python_arguments+=("launcher/example.py")
# Note: assume container workdir is `<super-project>/src/`

# ....Debug flags..................................................................................
#dnp_run_slurm_flags+=("--skip-core-force-rebuild")
#dnp_run_slurm_flags+=("--dry-run")

# ====DNP internal=================================================================================
dnp_run_slurm_flags+=("--log-name" "$(basename -s .bash $0)")
dnp_run_slurm_flags+=("--log-path" "artifact/slurm_jobs_logs")
python_arguments+=("$@")
export SJOB_ID
dnp::job_setup_callback
trap dnp::job_teardown_callback EXIT

# ====Launch slurm job=============================================================================
dnp run slurm "${SJOB_ID:?err}" "${dnp_run_slurm_flags[@]}" "${python_arguments[@]}"
