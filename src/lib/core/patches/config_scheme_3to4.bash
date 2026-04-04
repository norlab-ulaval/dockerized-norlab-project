#!/bin/bash
# =================================================================================================
# Configuration scheme patching script: v3 → v4.
#
# Usage:
#   source config_scheme_3to4.bash
#
# =================================================================================================

# ==== Patch logic starts here ====

# Add new configuration directories for v4
dna::patch_add_directory_if_missing ".dockerized_norlab/configuration/hpc_server_profile" ".dockerized_norlab/configuration/hpc_server_profile" "HPC server profile configurations"
dna::patch_add_directory_if_missing ".dockerized_norlab/configuration/overrides" ".dockerized_norlab/configuration/overrides" "Docker Compose override configurations"
# Replace placeholder in HPC server profile files
hpc_profile_files=(
  ".dockerized_norlab/configuration/hpc_server_profile/.env.valeria"
  ".dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada"
  ".dockerized_norlab/configuration/hpc_server_profile/.env.mamba"
)
for hpc_profile_file in "${hpc_profile_files[@]}"; do
  target_file="${SUPER_PROJECT_ROOT}/${hpc_profile_file}"
  if [[ -f "${target_file}" ]]; then
    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_PROJECT_GIT_NAME" "${SUPER_PROJECT_REPO_NAME}" "${target_file}"
  fi
done
unset hpc_profile_files
unset target_file

# Add new slurm job template directory and templates
dna::patch_add_directory_if_missing "slurm_jobs/template" "slurm_jobs/template" "Slurm job templates directory"

slurm_job_templates=(
  "slurm_jobs/template/slurm_job.SJOB_ID.apptainer.compute_canada.bash"
  "slurm_jobs/template/slurm_job.SJOB_ID.apptainer.hpc_hydra.bash"
  "slurm_jobs/template/slurm_job.SJOB_ID.apptainer.mamba.bash"
  "slurm_jobs/template/slurm_job.SJOB_ID.apptainer.valeria.bash"
  "slurm_jobs/template/slurm_job.SJOB_ID.hydra.bash"
  "slurm_jobs/template/slurm_job.SJOB_ID.hydra_hparam_optim.bash"
  "slurm_jobs/template/slurm_job.SJOB_ID.bash"
)

for template in "${slurm_job_templates[@]}"; do
  if dna::patch_add_file_if_missing "${template}" "${template}" "Slurm job template: $(basename "${template}")"; then
    # Replace placeholder in the newly added template
    target_file="${SUPER_PROJECT_ROOT}/${template}"
    if [[ -f "${target_file}" ]]; then
      n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_PROJECT_IMAGE_NAME" "${SUPER_PROJECT_REPO_NAME}" "${target_file}"
    fi
  fi
done
unset slurm_job_templates
unset target_file

# ==== Patch logic ends here ====
