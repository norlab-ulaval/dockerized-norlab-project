#!/bin/bash
# =================================================================================================
#
# This script aggregate global init entrypoint logic so that entrypoint callback script
# dn_entrypoint.global.init.callback.bash be less verbose for user.
#
# Helper script should be called in that order in dn_entrypoint.global.init.callback.bash:
#
#  # ....DNA-project internal logic................................................................
#  source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.global.common.bash || exit 1
#  source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.global.init.bash || exit 1
#
# =================================================================================================


function dna::show_container_dn_related_tree() {
  n2st::draw_horizontal_line_across_the_terminal_window "."
  echo -e "\033[1;33m[DN trace]\033[0m Dev tools to check in container available directories and files"
  echo
  tree -L 2 -a /dna-lib-container-tools
  tree -L 2 -a -I .git /dockerized-norlab
  tree -L 2 -a /home/redleader
  tree -L 2 -a /opt
  tree -L 2 -a /project_entrypoints
  tree -L 2 -a /ros2_ws
  tree -L 1 -a /
  echo "DN_PROJECT_PATH: ${DN_PROJECT_PATH:?err}"
  tree -L 2 -a -I .git  "${DN_PROJECT_PATH}"
  echo "PWD: $( pwd )"
  tree -L 2 -a -I .git
  echo
  n2st::draw_horizontal_line_across_the_terminal_window "."
}

function dna::entrypoint_helper_global_init() {
  local show_tree=false
  local tmp_cwd
  tmp_cwd=$(pwd)

  if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == 'true' ]]; then
    echo -e "\033[1;33m[DN trace]\033[0m Execute $(basename "${BASH_SOURCE[1]}") -> entrypoint_helper.global.init.bash"
    if [[ "${show_tree}" == true ]]; then
      dna::show_container_dn_related_tree
    fi
  fi

  # ....Sanity check.................................................................................
  if [[ "${DN_PROJECT_USER}" != "$(whoami)" ]]; then
    n2st::print_msg_error "Container login as user $(whoami) does not match project expected user DN_PROJECT_USER=${DN_PROJECT_USER}!\n
Trouble shooting procedure:
  1. Rebuild all images without using cache: ${MSG_DIMMED_FORMAT}dna build -- --no-cache${MSG_END_FORMAT}
  2. If it did not work, open a new terminal on the host machine, login as the desired user and rebuild all images
  3. If nothing work, open a bug ticket on https://github.com/norlab-ulaval/dockerized-norlab-project/issues"
    echo
    echo -e "${MSG_DIMMED_FORMAT}$(tree -L 1 -aug "${DN_PROJECT_PATH}")${MSG_END_FORMAT}"
    echo
    exit 1
  fi

  test -n "$(pgrep -x 'sshd')" || n2st::print_msg_warning "Be advised, ssh daemon is not running!\n" 1>&2

  # ....Remove byte-compiled files that could mess with tools on context/environment change..........
  pyclean "${DN_PROJECT_PATH}"
  # Remember the non-interactive-ros2 user path nightmare

  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "\033[1;31m[DN error]\033[0m This script must be sourced! i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }

  # This script is being sourced, ie: __name__="__source__"
  dna::entrypoint_helper_global_init || n2st::print_msg_error_and_exit "$0 script exited with error!"
fi
