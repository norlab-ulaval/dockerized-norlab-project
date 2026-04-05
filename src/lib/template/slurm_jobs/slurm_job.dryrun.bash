#!/bin/bash
#SBATCH --gres=gpu:0
#SBATCH --cpus-per-task=2
#SBATCH --time=0-01:00
#SBATCH --output=out/%x-%j.out


# Note: Flag time format --time=D-HH:MM ->  D=day, HH=hours, MM=minutes

# =================================================================================================
# Execute slurm job
#
# Usage:
#   $ bash slurm_job.dryrun.bash [<any-dna-argument>]
#
# =================================================================================================
declare -x DNA_SJOB_NAME
declare -a dna_run_slurm_flags=()
declare -a hydra_flags=()

# ====Setup========================================================================================
# ....Custom setup (optional)......................................................................
function dna::job_setup_callback() {
  # Add any instruction that should be executed before 'dna run slurm' command
  :
}

# ....Custom teardown (optional)...................................................................
function dna::job_teardown_callback() {
  local exit_code=$?
  # Add any instruction that should be executed after 'dna run slurm' exit.

  # Note: Command 'dna run slurm' already handle stoping the container in case the slurm command
  #  `scancel` is issued.
  exit ${exit_code:-1}
}

# ....Set job name.................................................................................
DNA_SJOB_NAME="dryrun"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an DNA_SJOB_NAME.

# ....Hydra app module.............................................................................
hydra_flags+=("launcher/example_app_hparm_optim.py")
# Note: assume container workdir is `<super-project>/src/`

# ....Optional hydra flags.........................................................................
# --config-path,-cp : Overrides the config_path specified in hydra.main(). (absolute or relative)
# --config-name,-cn : Overrides the config_name specified in hydra.main()
# --config-dir,-cd : Adds an additional config dir to the config search path

#hydra_flags+=("--config-path=")
#hydra_flags+=("--config-dir=")
#hydra_flags+=("--config-name=")

# ....Debug flags..................................................................................
dna_run_slurm_flags+=(--register-hydra-dry-run-flag "run_pytorch_check=true")

dna_run_slurm_flags+=("--skip-core-force-rebuild")
dna_run_slurm_flags+=("--skip-slurm-force-rebuild")
dna_run_slurm_flags+=("--hydra-dry-run")

# ====DNA internal=================================================================================
dna_run_slurm_flags+=("--log-name" "$(basename -s .bash $0)")
dna_run_slurm_flags+=("--log-path" "artifact/slurm_jobs_logs")
dna_run_slurm_flags+=("$@")
export DNA_SJOB_NAME
dna::job_setup_callback
trap dna::job_teardown_callback EXIT

# ====Launch slurm job=============================================================================
dna run slurm "${DNA_SJOB_NAME:?err}" "${dna_run_slurm_flags[@]}" "${hydra_flags[@]}"

