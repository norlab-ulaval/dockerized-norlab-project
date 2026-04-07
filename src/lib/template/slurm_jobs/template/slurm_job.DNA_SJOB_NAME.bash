#!/bin/bash
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=12
#SBATCH --time=0-24:00
#SBATCH --output=artifact/slurm_jobs_logs/%x-%j.out


# Note: Flag time format --time=D-HH:MM ->  D=day, HH=hours, MM=minutes

# =================================================================================================
# Execute slurm job
#
# Usage:
#   $ bash slurm_job.template.bash [<any-dna-argument>]
#
# =================================================================================================
declare -x DNA_SJOB_NAME
declare -a dna_run_slurm_flags=()
declare -a python_arguments=()

# ====Setup========================================================================================
# ....Custom setup (optional)......................................................................
function dna::job_setup_callback() {
  # Add any instruction that should be executed before 'dna run slurm' command
  :
}

# ....Custom teardown (optional)...................................................................
function dna::job_teardown_callback() {
  local exit_code=$?
  # TODO: Add any instruction that should be executed after 'dna run slurm' exit.

  # Note: Command 'dna run slurm' already handle stoping the container in case the slurm command
  #  `scancel` is issued.
  exit ${exit_code:-1}
}

# ....Python module................................................................................
# TODO: Set python module to launch
python_arguments+=("launcher/example.py")
# Note: assume container workdir is `<super-project>/src/`

# ....Debug flags..................................................................................
#dna_run_slurm_flags+=("--skip-core-force-rebuild")
#dna_run_slurm_flags+=("--skip-slurm-force-rebuild")
#dna_run_slurm_flags+=("--hydra-dry-run")

# ====DNA internal=================================================================================
# ....Set job name.................................................................................
# Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as the DNA_SJOB_NAME.

# Auto-set DNA_SJOB_NAME from the script filename (slurm_job.<name>.bash → <name>)
DNA_SJOB_NAME="$( basename "${BASH_SOURCE[0]}" | sed 's/^slurm_job\.//;s/\.bash$//' )"
export DNA_SJOB_NAME

dna_run_slurm_flags+=("--log-name" "$(basename -s .bash $0)")
dna_run_slurm_flags+=("--log-path" "artifact/slurm_jobs_logs")
dna_run_slurm_flags+=("$@")
dna::job_setup_callback
trap dna::job_teardown_callback EXIT

# ====Launch slurm job=============================================================================
dna run slurm "${DNA_SJOB_NAME:?err}" "${dna_run_slurm_flags[@]}" "${python_arguments[@]}"
