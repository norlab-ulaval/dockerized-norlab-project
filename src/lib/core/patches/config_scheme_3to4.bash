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
    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_CONTAINER_NAME" "IamDNA_${SUPER_PROJECT_REPO_NAME}" "${target_file}"
    # Add DN_CONTAINER_NAME line if not already present (for pre-existing files that predate this field)
    dna::patch_add_content_if_missing \
      "${hpc_profile_file}" \
      "DN_CONTAINER_NAME=" \
      "
# ....Container name configuration................................................................
# Container name used for identification. Set at runtime to: DN_CONTAINER_NAME-<sjob-name>
DN_CONTAINER_NAME=IamDNA_${SUPER_PROJECT_REPO_NAME}-slurm" \
      "Add DN_CONTAINER_NAME to $(basename "${hpc_profile_file}")"
  fi
done
unset hpc_profile_files
unset target_file

# Add new slurm job template directory and templates
dna::patch_add_directory_if_missing "slurm_jobs/template" "slurm_jobs/template" "Slurm job templates directory"

slurm_job_templates=(
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.hpc_hydra.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
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

# Update slurm_job.dryrun.bash: fix DNA_SJOB_NAME and remove TODO comments (only if file exists)
if [[ -f "${SUPER_PROJECT_ROOT}/slurm_jobs/slurm_job.dryrun.bash" ]]; then
  dna::patch_modify_content "slurm_jobs/slurm_job.dryrun.bash" \
    'DNA_SJOB_NAME="default"' \
    'DNA_SJOB_NAME="dryrun"' \
    "Fix DNA_SJOB_NAME from 'default' to 'dryrun' in slurm_job.dryrun.bash"

  dna::patch_modify_content "slurm_jobs/slurm_job.dryrun.bash" \
    '# TODO: Set DNA_SJOB_NAME' \
    '' \
    "Remove 'TODO: Set DNA_SJOB_NAME' comment from slurm_job.dryrun.bash"

  dna::patch_modify_content "slurm_jobs/slurm_job.dryrun.bash" \
    "  # TODO: Add any instruction that should be executed after 'dna run slurm' exit." \
    "  # Add any instruction that should be executed after 'dna run slurm' exit." \
    "Remove TODO comment from job_teardown_callback in slurm_job.dryrun.bash"

  dna::patch_modify_content "slurm_jobs/slurm_job.dryrun.bash" \
    '# TODO: Set python module to launch' \
    '' \
    "Remove 'TODO: Set python module to launch' comment from slurm_job.dryrun.bash"
fi

# ==== Patch logic ends here ====
