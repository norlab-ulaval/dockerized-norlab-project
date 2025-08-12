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
MSG_DIMMED_FORMAT_TEAMCITY="|[1;2m"
MSG_BASE_FORMAT_TEAMCITY="|[1m"
MSG_ERROR_FORMAT_TEAMCITY="|[1;31m"
MSG_WARNING_FORMAT_TEAMCITY="|[1;33m"
#MSG_STEP_FORMAT_TEAMCITY="|[1;104m"
MSG_STEP_FORMAT_TEAMCITY="|[30;107m"
MSG_END_FORMAT_TEAMCITY="|[0m"

dna_base_prefix="${MSG_BASE_FORMAT_TEAMCITY}|[dna|]${MSG_END_FORMAT_TEAMCITY}"
dna_error_prefix="${MSG_ERROR_FORMAT_TEAMCITY}|[dna error|]${MSG_END_FORMAT_TEAMCITY}"
dna_done_prefix="${MSG_STEP_FORMAT_TEAMCITY}|[dna done|]${MSG_END_FORMAT_TEAMCITY}"

function dna::teamcity_dna_path_override() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  local dna_install_dir
  local dna_bin_dir
  local dna_entrypoint
  declare -i exit_code

  # ....Begin......................................................................................
  echo
  echo -e "${dna_base_prefix} dna::teamcity_dna_path_override path update..."

  # Determine the installation directory
  dna_install_dir="$( git rev-parse --show-toplevel )"
  dna_bin_dir="${dna_install_dir}/src/bin"
  dna_entrypoint="${dna_bin_dir}/dna"

  # Make the dna script executable
  chmod +x "${dna_entrypoint}"
  exit_code=$?

  # export dna entrypoint path
  PATH="${dna_bin_dir}:${PATH}"

  # shellcheck disable=SC2028
  echo "##teamcity[setParameter name='env.PATH' value='${PATH}']"

  # ....Sanity check...............................................................................
  echo -e "\n${dna_base_prefix} TeamCity path update sanity check..."
  case ":$PATH:" in
    *":${dna_bin_dir}:"*)
        echo -e "${dna_base_prefix} DNA path is reachable in TC environment variables"
        ;;
    *)
        echo -e "${dna_error_prefix} DNA path is NOT reachable in TC environment variables"
        exit_code+=1
        ;;
  esac
  echo -e "${dna_base_prefix} path updated to PATH: ${PATH}"

  echo -e "\n${dna_base_prefix} DNA sanity check...\n"
  dna version --all
  exit_code+=$?
  echo

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }

  if [[ ${exit_code} -eq 0 ]]; then
    echo -e "${dna_done_prefix} dna::teamcity_dna_path_override completed successfully."
    return 0
  else
    echo -e "${dna_error_prefix} dna::teamcity_dna_path_override exited with error!"
    return 1
  fi
}

if [[ ${TEAMCITY_VERSION} ]] ; then
  dna::teamcity_dna_path_override || exit 1
fi
