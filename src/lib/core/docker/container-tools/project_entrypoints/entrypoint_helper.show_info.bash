#!/bin/bash
# =================================================================================================
#
# This script aggregate container info related logic so that it be less verbose for user.
#
# =================================================================================================

function dna::entrypoint_helper_show_info() {
  # ....Setup....................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Check pre-conditions.......................................................................
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DNA error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }

  # ....User feedback................................................................................
  source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

  # ....Show project specific alias..................................................................
  echo -e "Project ${DN_PROJECT_GIT_NAME:?err} specific in-container available aliases: ${MSG_DIMMED_FORMAT}
  \n$(
    SP="    " &&
      cd "/dockerized-norlab/dockerized-norlab-images/container-tools" &&
      sed "s;alias dna-${DN_PROJECT_ALIAS_PREFIX:?err}-;${SP}$ dna-${DN_PROJECT_ALIAS_PREFIX}-;" ./dn_bash_alias.bash | sed "s;='.*;;" | grep -e "dna-${DN_PROJECT_ALIAS_PREFIX}-"
  )
  ${MSG_END_FORMAT}"

  # ....Teardown.................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  dna_error_prefix="\033[1;31m[DNA error]\033[0m"
  echo -e "${dna_error_prefix} This script must be sourced! i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dna::entrypoint_helper_show_info || n2st::print_msg_error_and_exit "$0 script exited with error!"
fi
