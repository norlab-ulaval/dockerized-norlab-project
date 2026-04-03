#!/bin/bash
# =================================================================================================
# Mock entrypoint for DNA Apptainer pipeline testing.
#
# Simplified version of dn_entrypoint.init.bash that validates the essential
# Apptainer runtime environment without requiring the full DN library stack.
#
# Key behaviors tested:
#   - Detects Apptainer runtime via APPTAINER_CONTAINER / SINGULARITY_CONTAINER
#   - Validates DN_PROJECT_PATH
#   - Runs pyclean gracefully on read-only SIF (2>/dev/null || true)
#   - Executes python3 with provided arguments
#
# =================================================================================================
set -e

# ====Detect runtime environment (Docker vs Apptainer)=============================================
if [[ -n "${APPTAINER_CONTAINER}" ]] || [[ -n "${SINGULARITY_CONTAINER}" ]]; then
  DNA_RUNTIME="apptainer"
else
  DNA_RUNTIME="docker"
fi
export DNA_RUNTIME

# ====Validate DN_PROJECT_PATH=====================================================================
if [[ ! -d "${DN_PROJECT_PATH:?'Required DN environment variable is set and not empty'}/src" ]]; then
  echo -e "\n[DN error] '${DN_PROJECT_PATH}/src' directory unreachable! Current working directory is '$(pwd)'" 1>&2
  exit 1
else
  cd "${DN_PROJECT_PATH}/src" || exit 1
fi

# ====Trace execution (if enabled)=================================================================
if [[ "${DN_ENTRYPOINT_TRACE_EXECUTION}" == "true" ]]; then
  echo "[DN trace] Execute mock dn_entrypoint.init.bash (DNA_RUNTIME=${DNA_RUNTIME})"
fi

# ====Execute python command=======================================================================
cd "${DN_PROJECT_PATH}/src" || exit 1
python3 "$@" || exit 1

echo "[mock-entrypoint] Done (DNA_RUNTIME=${DNA_RUNTIME})"
exit 0
