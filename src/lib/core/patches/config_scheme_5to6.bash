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
# Note on re-running this patch to recover from a corrupted super project:
#   An earlier (perl-based) version of this patch had two known bugs:
#     1. The "HPC server configuration" block removal searched for the literal string
#        PLACEHOLDER_DN_PROJECT_IMAGE_NAME, which is already substituted in initialized super
#        projects. This caused the block to appear twice in the apptainer templates.
#     2. Multi-line replacements in non-apptainer templates produced literal \n sequences
#        instead of actual newlines.
#   If your super project exhibits these symptoms, reset DNA_CONFIG_SCHEME_VERSION back to 5
#   in .dockerized_norlab/.env.<SUPER_PROJECT_REPO_NAME> and re-run `dna update` to apply
#   this corrected patch.
#
# =================================================================================================

# ==== Patch logic starts here ====

# ....Determine the super project image name (same logic as dna init).............................
_super_project_image_name="$(echo "${SUPER_PROJECT_REPO_NAME:?err}" | tr '[:upper:]' '[:lower:]')"

# ....Replace non-apptainer slurm job templates (now using .dna.bash naming convention)...........
# Mapping: <old-name-in-super-project> -> <new-template-source-path>
# The super project may have the old naming (v5) or the new naming (already patched); both handled.

# Use two parallel indexed arrays instead of an associative array so this patch remains
# compatible with bash 3.2 (the default /bin/bash shipped on macOS). Associative arrays
# (`declare -A`) require bash >= 4 and would otherwise cause the infamous
# "expression recursion level exceeded" error when the subsequent subscript assignment
# is parsed as an arithmetic expression.
_non_apptainer_old_templates=(
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
  "slurm_jobs/slurm_job.dryrun.bash"
)
_non_apptainer_new_templates=(
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.dna.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.dna.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.dna.bash"
  "slurm_jobs/slurm_job.dryrun.dna.bash"
)

for _i in "${!_non_apptainer_old_templates[@]}"; do
  _old_t="${_non_apptainer_old_templates[${_i}]}"
  _new_t="${_non_apptainer_new_templates[${_i}]}"
  # Remove old-named file if it exists (renamed to .dna.bash)
  if [[ -f "${SUPER_PROJECT_ROOT}/${_old_t}" ]]; then
    rm -f "${SUPER_PROJECT_ROOT}/${_old_t}"
    n2st::print_msg "Removed old template: ${_old_t}"
  fi
  # Copy new template to super project under the new name.
  # - If the new-named file already exists (e.g., left by an earlier, buggy version of this patch),
  #   replace it wholesale so any corruption (e.g., literal \n sequences) is corrected.
  # - If it doesn't exist yet (normal rename path), add it fresh.
  if [[ -f "${SUPER_PROJECT_ROOT}/${_new_t}" ]]; then
    dna::patch_replace_file \
      "${_new_t}" \
      "${_new_t}" \
      "slurm job template $(basename "${_new_t}")"
  else
    dna::patch_add_file_if_missing \
      "${_new_t}" \
      "${_new_t}" \
      "slurm job template $(basename "${_new_t}")"
  fi
done
unset _non_apptainer_old_templates _non_apptainer_new_templates _old_t _new_t _i

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
