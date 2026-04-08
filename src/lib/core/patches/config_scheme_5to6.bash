#!/bin/bash
# =================================================================================================
# Configuration scheme patching script: v5 → v6.
#
# Usage:
#   source config_scheme_5to6.bash
#
# Changes applied:
#   1. All slurm job templates (base, hydra, hydra_hparam_optim, dryrun, valeria, compute_canada,
#      mamba) are replaced with the current DNA template versions.
#   2. The PLACEHOLDER_DN_PROJECT_IMAGE_NAME substitution is re-applied in each Apptainer template
#      (as performed by `dna init`), preserving the super project's image name.
#
# Rationale:
#   v6 introduces structural changes to all slurm job templates that are too numerous and fragile
#   to apply as individual find-and-replace operations (output path, DNA_SJOB_NAME auto-set,
#   HPC server configuration block, val-mktemp-dir → mktemp, module spider, APPTAINER_CACHEDIR).
#   Replacing templates wholesale and re-running init substitutions is simpler, more robust, and
#   easier to maintain.
#
# =================================================================================================

# ==== Patch logic starts here ====

# ....Determine the super project image name (same logic as dna init).............................
_super_project_image_name="$(echo "${SUPER_PROJECT_REPO_NAME:?err}" | tr '[:upper:]' '[:lower:]')"

# ....Replace non-apptainer slurm job templates...................................................

_non_apptainer_templates=(
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
  "slurm_jobs/slurm_job.dryrun.bash"
)

for _t in "${_non_apptainer_templates[@]}"; do
  dna::patch_replace_file \
    "${_t}" \
    "${_t}" \
    "slurm job template $(basename "${_t}")"
done
unset _non_apptainer_templates _t

# ....Replace apptainer slurm job templates and re-apply PLACEHOLDER substitution.................

_apptainer_templates=(
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
)

for _t in "${_apptainer_templates[@]}"; do
  if dna::patch_replace_file \
      "${_t}" \
      "${_t}" \
      "Apptainer slurm job template $(basename "${_t}")"; then
    # Re-apply PLACEHOLDER_DN_PROJECT_IMAGE_NAME substitution as dna init does
    _full_target="${SUPER_PROJECT_ROOT}/${_t}"
    if [[ -f "${_full_target}" ]]; then
      n2st::seek_and_modify_string_in_file \
        "PLACEHOLDER_DN_PROJECT_IMAGE_NAME" \
        "${_super_project_image_name}" \
        "${_full_target}"
    fi
  fi
done
unset _apptainer_templates _t _full_target _super_project_image_name

# ==== Patch logic ends here ====
