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
#   $ bash slurm_job.template.bash [<any-hydra-argument>]
#
# =================================================================================================

# ====Custom steps=================================================================================
declare -a HYDRA_FLAGS
declare -a FLAGS
declare -x SJOB_ID

# ....Set job name.................................................................................
export SJOB_ID="default" # Open a YouTrack task and use the issue ID

# ....Hydra app module.............................................................................
# Note assume cwd is `src/`
HYDRA_FLAGS+=("launcher/example_app.py")

# ....Optional flags...............................................................................
# --config-path,-cp : Overrides the config_path specified in hydra.main(). (absolute or relative)
# --config-name,-cn : Overrides the config_name specified in hydra.main()
# --config-dir,-cd : Adds an additional config dir to the config search path

#HYDRA_FLAGS+=("--config-path=")
#HYDRA_FLAGS+=("--config-dir=")
#HYDRA_FLAGS+=("--config-name=")

# ....Debug flags..................................................................................
FLAGS+=(--register-hydra-dry-run-flag "+new_key='fake-value'")

# (CRITICAL) ToDo: on task end >> mute next bloc ↓↓
#FLAGS+=("--skip-core-force-rebuild")
#FLAGS+=("--dry-run")
#HYDRA_FLAGS+=("--cfg" "all")

# .................................................................................................
FLAGS+=("--log-name" "$(basename -s .bash $0)")
FLAGS+=("--log-path" "artifact/slurm_jobs_logs")
FLAGS+=("$@")

# (Priority) ToDo: NMO-666 refactor: update slurm_jobs template
bash "${DNP_LIB_EXEC_PATH:?err}"/run.slurm.bash "${SJOB_ID:?err}" "${FLAGS[@]}" "${HYDRA_FLAGS[@]}"

# ====Teardown=====================================================================================
exit $?
