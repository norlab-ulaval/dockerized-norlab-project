#!/bin/bash
# =================================================================================================
# Override the DNA path environment variable to local repository instead of system wide install
# for tests executions so that we can directly use "dna COMMAND" in tests script on the TeamCity
# server e.g., See dockerized-norlab-project-mock/slurm_jobs/slurm_job.hydra_template.bash
#
# Usage:
#
#   Add the following to the first build step of all DNA repo TC build configurations
#   >>> #!/bin/bash
#   >>> source tests/teamcity_dna_path_override.bash
#
# Alternative:
#
#   Set DNA TeamCity project build configuration environment variable "env.PATH"
#   to "%system.teamcity.build.checkoutDir%/src/bin:%env.PATH%" in "Parameters" tab
#   "Environment Variables (env.)" section in each build configuration.
#
# Globals:
#   Read TEAMCITY_VERSION
#   Read/Write PATH
#
# Arguments:
#   None
#
# Outputs:
#   Writes status messages to stdout
#   Writes error messages to stderr
#
# Returns:
#   0 on success, 1 on error
# =================================================================================================
dna_error_prefix="\033[1;31m[dna error]\033[0m"
dna_done_prefix="\033[1;32m[dna done]\033[0m"

function dna::teamcity_dna_path_override() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  local dna_install_dir
  local dna_bin_dir
  local dna_entrypoint
  local exit_code

  # ....Begin......................................................................................
  # Determine the installation directory
  dna_install_dir="$( git rev-parse --show-toplevel )"
  dna_bin_dir="${dna_install_dir}/src/bin"
  dna_entrypoint="${dna_bin_dir}/dna"

  # Make the dna script executable
  chmod +x "${dna_entrypoint}"
  exit_code=$?

  # export dna entrypoint path
  export PATH="${dna_bin_dir}:$PATH"
  (( exit_code += $? ))

  # ....Sanity check...............................................................................
  echo -e "\n[dna] dna::teamcity_dna_path_override Sanity check...\n"
  dna version --all

  echo -e "\n[dna] dna::teamcity_dna_path_override path update..."
  echo -e"      PATH: ${PATH}\n"

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }

  if [[ ${exit_code} -eq 0 ]]; then
    echo -e "${dna_error_prefix} dna::teamcity_dna_path_override completed successfully."
    return 0
  else
    echo -e "${dna_done_prefix} dna::teamcity_dna_path_override exited with error!"
    return 1
  fi
}

if [[ ${TEAMCITY_VERSION} ]] ; then
  dna::teamcity_dna_path_override || exit 1
fi
