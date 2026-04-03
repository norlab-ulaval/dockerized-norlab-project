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

# Add new slurm job templates
dna::patch_add_file_if_missing "slurm_jobs/slurm_job.apptainer.compute_canada.template.bash" "slurm_jobs/slurm_job.apptainer.compute_canada.template.bash" "Compute Canada Apptainer slurm job template"
dna::patch_add_file_if_missing "slurm_jobs/slurm_job.apptainer.hpc_hydra.template.bash" "slurm_jobs/slurm_job.apptainer.hpc_hydra.template.bash" "HPC Hydra Apptainer slurm job template"
dna::patch_add_file_if_missing "slurm_jobs/slurm_job.apptainer.mamba.template.bash" "slurm_jobs/slurm_job.apptainer.mamba.template.bash" "Mamba Apptainer slurm job template"
dna::patch_add_file_if_missing "slurm_jobs/slurm_job.apptainer.valeria.template.bash" "slurm_jobs/slurm_job.apptainer.valeria.template.bash" "Valeria Apptainer slurm job template"
dna::patch_add_file_if_missing "slurm_jobs/slurm_job.hydra.template.bash" "slurm_jobs/slurm_job.hydra.template.bash" "Hydra slurm job template"
dna::patch_add_file_if_missing "slurm_jobs/slurm_job.hydra_hparam_optim.template.bash" "slurm_jobs/slurm_job.hydra_hparam_optim.template.bash" "Hydra hparam optimization slurm job template"

# ==== Patch logic ends here ====
