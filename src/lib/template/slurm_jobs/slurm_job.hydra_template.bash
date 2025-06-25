#!/bin/bash
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=12
#SBATCH --time=7-00:00
#SBATCH --output=out/%x-%j.out


# Note: Flag time format --time=D-HH:MM ->  D=day, HH=hours, MM=minutes

# =================================================================================================
# Execute slurm job
#
# Usage:
#   $ bash slurm_job.template.bash [<any-dna-argument>]
#
# =================================================================================================
declare -x SJOB_ID
declare -a dna_run_slurm_flags=()
declare -a hydra_flags=()

# ====Setup========================================================================================
# ....Custom setup (optional)......................................................................
function dna::job_setup_callback() {
  # TODO: Add any instruction that should be executed before 'dna run slurm' command
  :
}

# ....Custom teardown (optional)...................................................................
function dna::job_teardown_callback() {
  local exit_code=$?
  # TODO: Add any instruction that should be executed after 'dna run slurm' exit.

  # Note: Command 'dna run slurm' already handle stoping the container in case the slurm command
  #  `scancel` is issued.
  exit ${exit_code:1}
}

# ....Set job name.................................................................................
# TODO: Set SJOB_ID
SJOB_ID="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an SJOB_ID.

# ....Hydra app module.............................................................................
# TODO: Set python module to launch
hydra_flags+=("launcher/example_app.py")
# Note: assume container workdir is `<super-project>/src/`

# ....Optional hydra flags.........................................................................
# --config-path,-cp : Overrides the config_path specified in hydra.main(). (absolute or relative)
# --config-name,-cn : Overrides the config_name specified in hydra.main()
# --config-dir,-cd : Adds an additional config dir to the config search path

#hydra_flags+=("--config-path=")
#hydra_flags+=("--config-dir=")
#hydra_flags+=("--config-name=")

# ....Debug flags..................................................................................
dna_run_slurm_flags+=(--register-hydra-dry-run-flag "+new_key='fake-value'")

#dna_run_slurm_flags+=("--skip-core-force-rebuild")
#dna_run_slurm_flags+=("--dry-run")
#hydra_flags+=("--cfg" "all")

# ====DNA internal=================================================================================
dna_run_slurm_flags+=("--log-name" "$(basename -s .bash $0)")
dna_run_slurm_flags+=("--log-path" "artifact/slurm_jobs_logs")
dna_run_slurm_flags+=("$@")
export SJOB_ID
dna::job_setup_callback
trap dna::job_teardown_callback EXIT

# ====Launch slurm job=============================================================================
dna run slurm "${SJOB_ID:?err}" "${dna_run_slurm_flags[@]}" "${hydra_flags[@]}"

