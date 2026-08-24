#!/bin/bash
# =================================================================================================
# Configuration scheme patching script: v6 → v7.
#
# Usage:
#   source config_scheme_6to7.bash
#
# Changes applied:
#   1. The Apptainer slurm job templates (valeria, compute_canada, mamba) are replaced with the
#      current DNA template versions.
#   2. The PLACEHOLDER_DN_PROJECT_IMAGE_NAME substitution is re-applied in each Apptainer template
#      (as performed by `dna init`), preserving the super project's image name.
#
# Rationale:
#   v7 makes the slurm Apptainer artifacts target-aware so that building/pushing for one HPC target
#   (e.g. valeria) no longer overwrites the artifacts produced for another (e.g. compute_canada).
#   The generated Docker image tag and SIF filename now carry the target-platform suffix, the
#   generated helper scripts and tar archive are consolidated under artifact/apptainer/<target>/,
#   and the SIF is built into ${SCRATCH}/sif/ on the HPC server. The SIF filename is fully versioned
#   (<image>-slurm-<PROJECT_TAG>-<target>.sif) so different versions/targets never collide; because
#   the version is only known at build time, the Apptainer slurm job templates resolve the newest
#   matching versioned SIF at runtime via a glob (${SCRATCH}/sif/<image>-slurm-*-<target>.sif) instead
#   of artifact/apptainer/<image>-slurm.sif. These structural changes are replaced wholesale (as done
#   in v6) rather than as fragile find-and-replace operations.
#
# =================================================================================================

# ==== Patch logic starts here ====

# ....Determine the super project image name (same logic as dna init).............................
_super_project_image_name="$(echo "${SUPER_PROJECT_REPO_NAME:?err}" | tr '[:upper:]' '[:lower:]')"

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
