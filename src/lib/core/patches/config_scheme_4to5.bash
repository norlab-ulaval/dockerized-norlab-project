#!/bin/bash
# =================================================================================================
# Configuration scheme patching script: v4 → v5.
#
# Usage:
#   source config_scheme_4to5.bash
#
# =================================================================================================

# ==== Patch logic starts here ====

# ....HPC server profile .env files................................................................
# Remove the "Apptainer cache configuration" section (APPTAINER_CACHEDIR / APPTAINER_TMPDIR)
# from all HPC server profile dotenv files. These variables are now handled directly in the
# slurm job scripts or set by the generated converter scripts (not in the shared env file).

hpc_profile_files=(
  ".dockerized_norlab/configuration/hpc_server_profile/.env.valeria"
  ".dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada"
  ".dockerized_norlab/configuration/hpc_server_profile/.env.mamba"
)

for hpc_profile_file in "${hpc_profile_files[@]}"; do
  target_file="${SUPER_PROJECT_ROOT}/${hpc_profile_file}"
  if [[ -f "${target_file}" ]]; then
    dna::patch_modify_content \
      "${hpc_profile_file}" \
      "# ....Apptainer cache configuration" \
      "" \
      "Remove 'Apptainer cache configuration' comment from $(basename "${hpc_profile_file}")"

    dna::patch_modify_content \
      "${hpc_profile_file}" \
      "APPTAINER_CACHEDIR=" \
      "" \
      "Remove APPTAINER_CACHEDIR from $(basename "${hpc_profile_file}")"

    dna::patch_modify_content \
      "${hpc_profile_file}" \
      "APPTAINER_TMPDIR=" \
      "" \
      "Remove APPTAINER_TMPDIR from $(basename "${hpc_profile_file}")"
  fi
done
unset hpc_profile_files
unset target_file

# ....Slurm job templates: update default --time limit.............................................
# Change the default SBATCH --time from 7-00:00 (7 days) to 0-24:00 (24 hours) to encourage
# users to explicitly set their required time limit and avoid inadvertently holding cluster
# resources for an excessively long default duration.

slurm_job_templates=(
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
)

for template in "${slurm_job_templates[@]}"; do
  if [[ -f "${SUPER_PROJECT_ROOT}/${template}" ]]; then
    dna::patch_modify_content \
      "${template}" \
      "#SBATCH --time=7-00:00" \
      "#SBATCH --time=0-24:00" \
      "Update default SBATCH --time from 7-00:00 to 0-24:00 in $(basename "${template}")"
  fi
done
unset slurm_job_templates

# ....Valeria apptainer slurm job template.........................................................
# (1) Update APPTAINER_TMPDIR / APPTAINER_CACHEDIR block: replace the old SLURM_TMPDIR-based
#     single-line assignment with the Valeria-recommended val-mktemp-dir calls.

valeria_template="slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
if [[ -f "${SUPER_PROJECT_ROOT}/${valeria_template}" ]]; then
  dna::patch_modify_content \
    "${valeria_template}" \
    "# Set APPTAINER_TMPDIR to SLURM_TMPDIR for best performance on Valeria" \
    "# Set Apptainer cache and tmp dirs using Valeria's val-mktemp-dir for best performance" \
    "Update APPTAINER_TMPDIR comment in valeria slurm template"

  dna::patch_modify_content \
    "${valeria_template}" \
    'APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"' \
    'export APPTAINER_CACHEDIR="$( val-mktemp-dir )"' \
    "Replace APPTAINER_TMPDIR=SLURM_TMPDIR with val-mktemp-dir in valeria slurm template"

  dna::patch_modify_content \
    "${valeria_template}" \
    "export APPTAINER_TMPDIR" \
    'export APPTAINER_TMPDIR="$( val-mktemp-dir )"' \
    "Update APPTAINER_TMPDIR export to use val-mktemp-dir in valeria slurm template"

  # (2) Update job_setup_callback to load required Valeria modules and profile utilities.
  dna::patch_modify_content \
    "${valeria_template}" \
    "  # Add any instruction that should be executed before the apptainer exec command
  :" \
    "  # Add any instruction that should be executed before the apptainer exec command
  module load apptainer

  # Required for wandb.ai
  module load httpproxy

  # Required because configurations under profile.d are not available by default for batch jobs
  # Ref https://doc.s3.valeria.science/fr/calcul/apptainer.html
  source /etc/profile.d/val-utils.sh" \
    "Add module load apptainer/httpproxy and val-utils.sh source to job_setup_callback in valeria template"

  # (3) Add optional hydra flags comment block (after python_arguments launcher line, before HPC server config).
  dna::patch_add_content_if_missing \
    "${valeria_template}" \
    "# ....Optional hydra flags" \
    '
# ....Optional hydra flags.........................................................................
# --config-path,-cp : Overrides the config_path specified in hydra.main(). (absolute or relative)
# --config-name,-cn : Overrides the config_name specified in hydra.main()
# --config-dir,-cd : Adds an additional config dir to the config search path
#python_arguments+=("--config-path=")
#python_arguments+=("--config-dir=")
#python_arguments+=("--config-name=")' \
    "Add optional hydra flags comment block to valeria apptainer slurm template"
fi
unset valeria_template

# ....Compute Canada apptainer slurm job template..................................................
# Add optional hydra flags comment block (mirroring the valeria template update).

compute_canada_template="slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
if [[ -f "${SUPER_PROJECT_ROOT}/${compute_canada_template}" ]]; then
  dna::patch_add_content_if_missing \
    "${compute_canada_template}" \
    "# ....Optional hydra flags" \
    '
# ....Optional hydra flags.........................................................................
# --config-path,-cp : Overrides the config_path specified in hydra.main(). (absolute or relative)
# --config-name,-cn : Overrides the config_name specified in hydra.main()
# --config-dir,-cd : Adds an additional config dir to the config search path
#python_arguments+=("--config-path=")
#python_arguments+=("--config-dir=")
#python_arguments+=("--config-name=")' \
    "Add optional hydra flags comment block to compute_canada apptainer slurm template"
fi
unset compute_canada_template

# ....Mamba apptainer slurm job template...........................................................
# Add optional hydra flags comment block (mirroring the valeria and compute_canada template update).

mamba_template="slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
if [[ -f "${SUPER_PROJECT_ROOT}/${mamba_template}" ]]; then
  dna::patch_add_content_if_missing \
    "${mamba_template}" \
    "# ....Optional hydra flags" \
    '
# ....Optional hydra flags.........................................................................
# --config-path,-cp : Overrides the config_path specified in hydra.main(). (absolute or relative)
# --config-name,-cn : Overrides the config_name specified in hydra.main()
# --config-dir,-cd : Adds an additional config dir to the config search path
#python_arguments+=("--config-path=")
#python_arguments+=("--config-dir=")
#python_arguments+=("--config-name=")' \
    "Add optional hydra flags comment block to mamba apptainer slurm template"
fi
unset mamba_template

# ==== Patch logic ends here ====
