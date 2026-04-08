#!/bin/bash
# =================================================================================================
# Configuration scheme patching script: v5 → v6.
#
# Usage:
#   source config_scheme_5to6.bash
#
# Changes applied:
#   1. All slurm job templates: update #SBATCH --output from 'out/' to 'artifact/slurm_jobs_logs/'.
#   2. Non-apptainer slurm templates (base, hydra, hydra_hparam_optim, dryrun):
#      - Remove old 'Set job name' block (DNA_SJOB_NAME="default" / "dryrun") from Setup section.
#      - Add canonical auto-set DNA_SJOB_NAME block in DNA internal section.
#      - Remove duplicate 'export DNA_SJOB_NAME' that followed dna_run_slurm_flags+=("$@").
#   3. Apptainer slurm templates (valeria, compute_canada, mamba):
#      - Remove old 'Set job name' block (DNA_SJOB_NAME="default") from Setup section.
#      - Add auto-set DNA_SJOB_NAME + move 'HPC server configuration' block into DNA internal section.
#      - Replace val-mktemp-dir / APPTAINER_TMPDIR=SLURM_TMPDIR with version-aware module spider
#        block and APPTAINER_CACHEDIR+APPTAINER_TMPDIR using generic mktemp.
#      - Update job_setup_callback: remove 'module load apptainer' and val-utils.sh (valeria).
#
# Note: All multi-line replacements use 'perl -0777 -i -pe' (POSIX portable, no sed delimiter issues).
#       Replacement strings use $'...' ANSI-C quoting to embed actual newlines.
#
# =================================================================================================

# ==== Patch logic starts here ====

# -------------------------------------------------------------------------------------------------
# Internal helper: apply a multi-line perl search/replace to a file, guarded by prompt.
#
# Usage:
#   _dna_patch_perl_replace <target_file_rel_path> <grep_probe> \
#                            <perl_search_pattern> <replace_string> <description>
#
# Arguments:
#   grep_probe:         A short literal string to test if the pattern exists (grep -qF).
#   perl_search_pattern: Perl regex (multiline, /gms flags). Use \n for newlines in search.
#   replace_string:     The literal replacement string (use $'...' ANSI-C quoting with real \n).
# -------------------------------------------------------------------------------------------------
function _dna_patch_perl_replace() {
  local target_file="$1"
  local grep_probe="$2"
  local perl_search="$3"
  local perl_replace="$4"
  local description="$5"

  local full_target_path="${SUPER_PROJECT_ROOT}/${target_file}"

  if [[ ! -f "${full_target_path}" ]]; then
    n2st::print_msg_error "Target file not found for patching: ${target_file}"
    return 1
  fi

  if grep -qF "${grep_probe}" "${full_target_path}"; then
    n2st::print_msg "Modifying ${description} in ${target_file}"

    dna::patch_prompt_user "Apply modification?"
    local user_input="${REPLY}"

    if [[ "${user_input}" == "y" || "${user_input}" == "Y" ]]; then
      # Write replacement to a temp file to avoid shell quoting issues with complex multi-line strings
      local _replace_tmpfile
      _replace_tmpfile="$(mktemp)"
      printf '%s' "${perl_replace}" > "${_replace_tmpfile}"
      perl -0777 -i -pe \
        'BEGIN{ $s=shift; $rf=shift; open(FH,$rf) or die; local $/; $r=<FH>; close FH } s/$s/$r/gms' \
        "${perl_search}" \
        "${_replace_tmpfile}" \
        "${full_target_path}"
      local _perl_exit=$?
      rm -f "${_replace_tmpfile}"
      if [[ ${_perl_exit} -ne 0 ]]; then
        n2st::print_msg_error "Perl replace failed for ${target_file}"
        return 1
      fi
      added_resources+=("${target_file} (content modification)")
    else
      n2st::print_msg_warning "Skipping modification for ${target_file}. This might cause issues."
    fi
  fi
  return 0
}

# ....All slurm job templates: update #SBATCH --output path........................................
# Change from 'out/%x-%j.out' to 'artifact/slurm_jobs_logs/%x-%j.out' to align with the DNA
# artifact directory structure. This is a single-line change — safe to use dna::patch_modify_content.

_all_slurm_templates=(
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
  "slurm_jobs/slurm_job.dryrun.bash"
)
for _t in "${_all_slurm_templates[@]}"; do
  if [[ -f "${SUPER_PROJECT_ROOT}/${_t}" ]]; then
    dna::patch_modify_content \
      "${_t}" \
      "#SBATCH --output=out/%x-%j.out" \
      "#SBATCH --output=artifact/slurm_jobs_logs/%x-%j.out" \
      "Update #SBATCH --output path in $(basename "${_t}")"
  fi
done
unset _all_slurm_templates _t

# ....Non-apptainer slurm templates: DNA_SJOB_NAME auto-set restructuring.........................

# ---- Helper sub-function for the shared 3-step non-apptainer restructuring ----
# Arguments: <rel_path> <grep_probe_for_default> <old_Set_job_block_search> \
#             <auto_set_suffix_for_sed> <dna_internal_header_line>
function _dna_patch_restructure_nonapptainer() {
  local _tfile="$1"
  local _grep_probe="$2"       # e.g. 'DNA_SJOB_NAME="default"' or 'DNA_SJOB_NAME="dryrun"'
  local _old_block_search="$3" # perl regex to remove old Set job name block
  local _sed_suffix="$4"       # sed stripping suffix, e.g. ';s/\.bash$//'
  local _arrow_comment="$5"    # arrow comment for auto-set line, e.g. 'slurm_job.<name>.bash → <name>'

  # (1) Remove old Set job name block
  local _old_block_replace='# ....Python module'
  [[ "${_old_block_search}" == *'dryrun'* ]] && _old_block_replace='# ....Hydra app module'
  [[ "${_old_block_search}" == *'Hydra app module'* ]] && _old_block_replace='# ....Hydra app module'
  _dna_patch_perl_replace "${_tfile}" \
    "${_grep_probe}" \
    "${_old_block_search}" \
    "${_old_block_replace}" \
    "Remove old DNA_SJOB_NAME block from ${_tfile}"

  # (2) Add auto-set DNA_SJOB_NAME block into DNA internal section
  local _r2
  _r2=$'# ====DNA internal=================================================================================\n'"# ....Set job name.................................................................................\n"$'# Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)\n'"#  and use its issue ID as the DNA_SJOB_NAME.\n\n"$'# Auto-set DNA_SJOB_NAME from the script filename ('"${_arrow_comment}"$')\n'"DNA_SJOB_NAME=\"\$( basename \"\${BASH_SOURCE[0]}\" | sed 's/^slurm_job\\.//""${_sed_suffix}""' )\"\n"$'export DNA_SJOB_NAME\n'
  _dna_patch_perl_replace "${_tfile}" \
    '# ====DNA internal' \
    '# ====DNA internal=+' \
    "${_r2}" \
    "Add auto-set DNA_SJOB_NAME block to DNA internal section in ${_tfile}"

  # (3) Remove duplicate 'export DNA_SJOB_NAME' that follows dna_run_slurm_flags+=("$@")
  _dna_patch_perl_replace "${_tfile}" \
    'export DNA_SJOB_NAME' \
    'dna_run_slurm_flags\+=\("\$\@"\)\nexport DNA_SJOB_NAME\n' \
    $'dna_run_slurm_flags+=("$@")\n' \
    "Remove duplicate export DNA_SJOB_NAME from ${_tfile}"
}

# Base template
_t="slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
[[ -f "${SUPER_PROJECT_ROOT}/${_t}" ]] && _dna_patch_restructure_nonapptainer \
  "${_t}" \
  'DNA_SJOB_NAME="default"' \
  '# \.\.\.\.Set job name\.+\n# TODO: Set DNA_SJOB_NAME\nDNA_SJOB_NAME="default"\n# Note: Recommend opening an issue tracker task \(e\.g\., YouTrack, GitHub issue, Trello\)\n#  and use its issue ID as an DNA_SJOB_NAME\.\n# \.\.\.\.Python module' \
  ';s/\.bash$//' \
  'slurm_job.<name>.bash → <name>'
unset _t

# Hydra template
_t="slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
[[ -f "${SUPER_PROJECT_ROOT}/${_t}" ]] && _dna_patch_restructure_nonapptainer \
  "${_t}" \
  'DNA_SJOB_NAME="default"' \
  '# \.\.\.\.Set job name\.+\n# TODO: Set DNA_SJOB_NAME\nDNA_SJOB_NAME="default"\n# Note: Recommend opening an issue tracker task \(e\.g\., YouTrack, GitHub issue, Trello\)\n#  and use its issue ID as an DNA_SJOB_NAME\.\n\n# \.\.\.\.Hydra app module' \
  ';s/\.hydra\.bash$//' \
  'slurm_job.<name>.hydra.bash → <name>'
unset _t

# Hydra hparam optim template
_t="slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
[[ -f "${SUPER_PROJECT_ROOT}/${_t}" ]] && _dna_patch_restructure_nonapptainer \
  "${_t}" \
  'DNA_SJOB_NAME="default"' \
  '# \.\.\.\.Set job name\.+\n# TODO: Set DNA_SJOB_NAME\nDNA_SJOB_NAME="default"\n# Note: Recommend opening an issue tracker task \(e\.g\., YouTrack, GitHub issue, Trello\)\n#  and use its issue ID as an DNA_SJOB_NAME\.\n\n# \.\.\.\.Hydra app module' \
  ';s/\.hydra_hparam_optim\.bash$//' \
  'slurm_job.<name>.hydra_hparam_optim.bash → <name>'
unset _t

# Dryrun template
_t="slurm_jobs/slurm_job.dryrun.bash"
[[ -f "${SUPER_PROJECT_ROOT}/${_t}" ]] && _dna_patch_restructure_nonapptainer \
  "${_t}" \
  'DNA_SJOB_NAME="dryrun"' \
  '# \.\.\.\.Set job name\.+\nDNA_SJOB_NAME="dryrun"\n# Note: Recommend opening an issue tracker task \(e\.g\., YouTrack, GitHub issue, Trello\)\n#  and use its issue ID as an DNA_SJOB_NAME\.\n\n# \.\.\.\.Hydra app module' \
  ';s/\.bash$//' \
  'slurm_job.<name>.bash → <name>'
unset _t

# ....Valeria apptainer slurm job template.........................................................
_t="slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
if [[ -f "${SUPER_PROJECT_ROOT}/${_t}" ]]; then

  # (1) Update job_setup_callback: remove 'module load apptainer' and val-utils.sh
  _dna_patch_perl_replace "${_t}" \
    'source /etc/profile.d/val-utils.sh' \
    '  # Add any instruction that should be executed before the apptainer exec command\n  module load apptainer\n\n  # Required for wandb\.ai\n  module load httpproxy\n\n  # Required because configurations under profile\.d are not available by default for batch jobs\n  # Ref https://doc\.s3\.valeria\.science/fr/calcul/apptainer\.html\n  source /etc/profile\.d/val-utils\.sh' \
    $'  # TODO: Add any instruction that should be executed before the apptainer exec command\n\n  # Required for wandb.ai\n  module load httpproxy' \
    "Update job_setup_callback in valeria: remove module load apptainer and val-utils.sh"

  # (2) Remove old Set job name block from Setup section
  _dna_patch_perl_replace "${_t}" \
    'DNA_SJOB_NAME="default"' \
    '# \.\.\.\.Set job name\.+\n# TODO: Set DNA_SJOB_NAME\nDNA_SJOB_NAME="default"\n# Note: Recommend opening an issue tracker task \(e\.g\., YouTrack, GitHub issue, Trello\)\n#  and use its issue ID as an DNA_SJOB_NAME\.\n# \.\.\.\.Python module' \
    '# ....Python module' \
    "Remove old DNA_SJOB_NAME=default block from valeria template"

  # (3) Remove old HPC server configuration block from Setup section
  #     (It will be re-added inside DNA internal in step 4)
  _dna_patch_perl_replace "${_t}" \
    '/.env.valeria"' \
    '# \.\.\.\.HPC server configuration\.+\nSUPER_PROJECT_ROOT="\${SUPER_PROJECT_ROOT:-\$\(pwd\)}"\nSIF_PATH="\${SIF_PATH:-\${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm\.sif}"\nPROFILE_ENV_FILE="\${SUPER_PROJECT_ROOT}/\.dockerized_norlab/configuration/hpc_server_profile/\.env\.valeria"\n# ====DNA internal' \
    '# ====DNA internal' \
    "Remove old HPC server configuration block from valeria template Setup section"

  # (4) Restructure DNA internal: replace standalone export DNA_SJOB_NAME with
  #     auto-set block + HPC config block
  _dna_patch_perl_replace "${_t}" \
    'export DNA_SJOB_NAME' \
    '# ====DNA internal=+\nexport DNA_SJOB_NAME\n\n# Source HPC-specific env' \
    $'# ====DNA internal=================================================================================\n# ....Set job name.................................................................................\n# Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)\n#  and use its issue ID as the DNA_SJOB_NAME.\n\n# Auto-set DNA_SJOB_NAME from the script filename (slurm_job.<name>.apptainer.valeria.bash -> <name>)\nDNA_SJOB_NAME="$( basename "${BASH_SOURCE[0]}" | sed \'s/^slurm_job\\.//;s/\\.apptainer\\.valeria\\.bash$//\' )"\nexport DNA_SJOB_NAME\n\n# ....HPC server configuration.....................................................................\nSUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"\nSIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"\nPROFILE_ENV_FILE="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria"\n\n# Source HPC-specific env' \
    "Restructure DNA internal section in valeria: add auto-set DNA_SJOB_NAME + HPC config"

  # (5) Replace val-mktemp-dir with version-aware module spider + generic mktemp
  _dna_patch_perl_replace "${_t}" \
    'val-mktemp-dir' \
    "# Set Apptainer cache and tmp dirs using Valeria's val-mktemp-dir for best performance\nexport APPTAINER_CACHEDIR=\"\\\$\\( val-mktemp-dir \\)\"\nexport APPTAINER_TMPDIR=\"\\\$\\( val-mktemp-dir \\)\"" \
    $'# ====Load Apptainer module (HPC module system)====================================================\n# Try to load the highest available apptainer version; fallback to default.\nif command -v module &>/dev/null; then\n  _APPTAINER_LATEST_VERSION="$( module spider apptainer 2>&1 | grep -oE \'apptainer/[0-9]+\\.[0-9]+\\.[0-9]+\' | sed \'s|apptainer/||\' | sort -V | tail -1 )"\n  if [[ -n "${_APPTAINER_LATEST_VERSION}" ]]; then\n    echo "[info] Loading Apptainer module version: ${_APPTAINER_LATEST_VERSION}" 1>&2\n    module load "apptainer/${_APPTAINER_LATEST_VERSION}"\n  else\n    echo "[info] Loading default Apptainer module" 1>&2\n    module load apptainer\n  fi\nfi\n\n# Set APPTAINER_CACHEDIR and APPTAINER_TMPDIR to the local node scratch space.\n# Using SLURM_TMPDIR (fast local SSD allocated per job) avoids writing to the Lustre\n# home filesystem, which has quota limits and does not support atomic rename required\n# by Apptainer\'s cache. Falls back to /tmp if SLURM_TMPDIR is not set.\n# Ref: https://apptainer.org/docs/user/latest/build_env.html\n# Ref: https://doc.s3.valeria.science/fr/calcul/apptainer.html#bonnes-pratiques\nexport APPTAINER_CACHEDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"\nexport APPTAINER_TMPDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"' \
    "Replace val-mktemp-dir with version-aware module spider and generic mktemp in valeria"
fi
unset _t

# ....Compute Canada apptainer slurm job template..................................................
_t="slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
if [[ -f "${SUPER_PROJECT_ROOT}/${_t}" ]]; then

  # (1) Update job_setup_callback: add TODO and httpproxy
  _dna_patch_perl_replace "${_t}" \
    '  # Add any instruction that should be executed before the apptainer exec command' \
    '  # Add any instruction that should be executed before the apptainer exec command\n  :' \
    $'  # TODO: Add any instruction that should be executed before the apptainer exec command\n\n  # Required for wandb.ai\n  module load httpproxy' \
    "Update job_setup_callback in compute_canada: add TODO and httpproxy"

  # (2) Remove old Set job name block
  _dna_patch_perl_replace "${_t}" \
    'DNA_SJOB_NAME="default"' \
    '# \.\.\.\.Set job name\.+\n# TODO: Set DNA_SJOB_NAME\nDNA_SJOB_NAME="default"\n# Note: Recommend opening an issue tracker task \(e\.g\., YouTrack, GitHub issue, Trello\)\n#  and use its issue ID as an DNA_SJOB_NAME\.\n# \.\.\.\.Python module' \
    '# ....Python module' \
    "Remove old DNA_SJOB_NAME=default block from compute_canada template"

  # (3) Remove old HPC server configuration block from Setup section
  _dna_patch_perl_replace "${_t}" \
    '/.env.compute_canada"' \
    '# \.\.\.\.HPC server configuration\.+\nSUPER_PROJECT_ROOT="\${SUPER_PROJECT_ROOT:-\$\(pwd\)}"\nSIF_PATH="\${SIF_PATH:-\${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm\.sif}"\nPROFILE_ENV_FILE="\${SUPER_PROJECT_ROOT}/\.dockerized_norlab/configuration/hpc_server_profile/\.env\.compute_canada"\n# ====DNA internal' \
    '# ====DNA internal' \
    "Remove old HPC server configuration block from compute_canada Setup section"

  # (4) Restructure DNA internal: add auto-set DNA_SJOB_NAME + HPC config
  _dna_patch_perl_replace "${_t}" \
    'export DNA_SJOB_NAME' \
    '# ====DNA internal=+\nexport DNA_SJOB_NAME\n\n# Source HPC-specific env' \
    $'# ====DNA internal=================================================================================\n# ....Set job name.................................................................................\n# Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)\n#  and use its issue ID as the DNA_SJOB_NAME.\n\n# Auto-set DNA_SJOB_NAME from the script filename (slurm_job.<name>.apptainer.compute_canada.bash -> <name>)\nDNA_SJOB_NAME="$( basename "${BASH_SOURCE[0]}" | sed \'s/^slurm_job\\.//;s/\\.apptainer\\.compute_canada\\.bash$//\' )"\nexport DNA_SJOB_NAME\n\n# ....HPC server configuration.....................................................................\nSUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"\nSIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"\nPROFILE_ENV_FILE="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada"\n\n# Source HPC-specific env' \
    "Restructure DNA internal section in compute_canada: add auto-set DNA_SJOB_NAME + HPC config"

  # (5) Replace old APPTAINER_TMPDIR=SLURM_TMPDIR with version-aware module spider + mktemp
  _dna_patch_perl_replace "${_t}" \
    'APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"' \
    '# Set APPTAINER_TMPDIR to SLURM_TMPDIR for best performance on Compute Canada\n# \(SLURM_TMPDIR is high-speed local storage allocated per job\)\nAPPTAINER_TMPDIR="\${SLURM_TMPDIR:-/tmp}"\nexport APPTAINER_TMPDIR' \
    $'# ====Load Apptainer module (HPC module system)====================================================\n# Try to load the highest available apptainer version; fallback to default.\nif command -v module &>/dev/null; then\n  _APPTAINER_LATEST_VERSION="$( module spider apptainer 2>&1 | grep -oE \'apptainer/[0-9]+\\.[0-9]+\\.[0-9]+\' | sed \'s|apptainer/||\' | sort -V | tail -1 )"\n  if [[ -n "${_APPTAINER_LATEST_VERSION}" ]]; then\n    echo "[info] Loading Apptainer module version: ${_APPTAINER_LATEST_VERSION}" 1>&2\n    module load "apptainer/${_APPTAINER_LATEST_VERSION}"\n  else\n    echo "[info] Loading default Apptainer module" 1>&2\n    module load apptainer\n  fi\nfi\n\n# Set APPTAINER_CACHEDIR and APPTAINER_TMPDIR to the local node scratch space.\n# Using SLURM_TMPDIR (fast local SSD allocated per job) avoids writing to network\n# filesystems, which have quota limits and may not support atomic rename required\n# by Apptainer\'s cache. Falls back to /tmp if SLURM_TMPDIR is not set.\n# Ref: https://apptainer.org/docs/user/latest/build_env.html\nexport APPTAINER_CACHEDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"\nexport APPTAINER_TMPDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"' \
    "Replace APPTAINER_TMPDIR=SLURM_TMPDIR with module spider + mktemp in compute_canada"
fi
unset _t

# ....Mamba apptainer slurm job template...........................................................
_t="slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
if [[ -f "${SUPER_PROJECT_ROOT}/${_t}" ]]; then

  # (1) Remove old Set job name block
  _dna_patch_perl_replace "${_t}" \
    'DNA_SJOB_NAME="default"' \
    '# \.\.\.\.Set job name\.+\n# TODO: Set DNA_SJOB_NAME\nDNA_SJOB_NAME="default"\n# Note: Recommend opening an issue tracker task \(e\.g\., YouTrack, GitHub issue, Trello\)\n#  and use its issue ID as an DNA_SJOB_NAME\.\n# \.\.\.\.Python module' \
    '# ....Python module' \
    "Remove old DNA_SJOB_NAME=default block from mamba template"

  # (2) Remove old HPC server configuration block from Setup section
  _dna_patch_perl_replace "${_t}" \
    '/.env.mamba"' \
    '# \.\.\.\.HPC server configuration\.+\nSUPER_PROJECT_ROOT="\${SUPER_PROJECT_ROOT:-\$\(pwd\)}"\nSIF_PATH="\${SIF_PATH:-\${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm\.sif}"\nPROFILE_ENV_FILE="\${SUPER_PROJECT_ROOT}/\.dockerized_norlab/configuration/hpc_server_profile/\.env\.mamba"\n# ====DNA internal' \
    '# ====DNA internal' \
    "Remove old HPC server configuration block from mamba Setup section"

  # (3) Restructure DNA internal: add auto-set DNA_SJOB_NAME + HPC config
  _dna_patch_perl_replace "${_t}" \
    'export DNA_SJOB_NAME' \
    '# ====DNA internal=+\nexport DNA_SJOB_NAME\n\n# Source HPC-specific env' \
    $'# ====DNA internal=================================================================================\n# ....Set job name.................................................................................\n# Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)\n#  and use its issue ID as the DNA_SJOB_NAME.\n\n# Auto-set DNA_SJOB_NAME from the script filename (slurm_job.<name>.apptainer.mamba.bash -> <name>)\nDNA_SJOB_NAME="$( basename "${BASH_SOURCE[0]}" | sed \'s/^slurm_job\\.//;s/\\.apptainer\\.mamba\\.bash$//\' )"\nexport DNA_SJOB_NAME\n\n# ....HPC server configuration.....................................................................\nSUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"\nSIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"\nPROFILE_ENV_FILE="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.mamba"\n\n# Source HPC-specific env' \
    "Restructure DNA internal section in mamba: add auto-set DNA_SJOB_NAME + HPC config"

  # (4) Replace old APPTAINER_TMPDIR=SLURM_TMPDIR with version-aware module spider + mktemp
  _dna_patch_perl_replace "${_t}" \
    'APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"' \
    '# Set APPTAINER_TMPDIR to SLURM_TMPDIR for best performance on Mamba\nAPPTAINER_TMPDIR="\${SLURM_TMPDIR:-/tmp}"\nexport APPTAINER_TMPDIR' \
    $'# ====Load Apptainer module (HPC module system)====================================================\n# Try to load the highest available apptainer version; fallback to default.\nif command -v module &>/dev/null; then\n  _APPTAINER_LATEST_VERSION="$( module spider apptainer 2>&1 | grep -oE \'apptainer/[0-9]+\\.[0-9]+\\.[0-9]+\' | sed \'s|apptainer/||\' | sort -V | tail -1 )"\n  if [[ -n "${_APPTAINER_LATEST_VERSION}" ]]; then\n    echo "[info] Loading Apptainer module version: ${_APPTAINER_LATEST_VERSION}" 1>&2\n    module load "apptainer/${_APPTAINER_LATEST_VERSION}"\n  else\n    echo "[info] Loading default Apptainer module" 1>&2\n    module load apptainer\n  fi\nfi\n\n# Set APPTAINER_CACHEDIR and APPTAINER_TMPDIR to the local node scratch space.\n# Using SLURM_TMPDIR (fast local SSD allocated per job) avoids writing to network\n# filesystems, which have quota limits and may not support atomic rename required\n# by Apptainer\'s cache. Falls back to /tmp if SLURM_TMPDIR is not set.\n# Ref: https://apptainer.org/docs/user/latest/build_env.html\nexport APPTAINER_CACHEDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"\nexport APPTAINER_TMPDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"' \
    "Replace APPTAINER_TMPDIR=SLURM_TMPDIR with module spider + mktemp in mamba"
fi
unset _t

# ==== Patch logic ends here ====
