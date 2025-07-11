#!/bin/bash
# =================================================================================================
#
# This aggregate global init entrypoint logic so that dn_entrypoint.global.init.callback.bash be
# less verbose for user.
#
# =================================================================================================
pushd "$(pwd)" >/dev/null || exit 1

function dna::show_container_dn_related_tree() {
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo "Dev tools to check in container available directories and files"
  echo
  tree -L 4 -a /dna-lib-container-tools
  tree -L 4 -a -I .git /dockerized-norlab
  tree -L 4 -a /home/redleader
  tree -L 2 -a /opt
  tree -L 4 -a /project_entrypoints
  tree -L 4 -a /ros2_ws
  tree -L 1 -a /
  echo "DN_PROJECT_PATH: ${DN_PROJECT_PATH:?err}"
  tree -L 4 -a "${DN_PROJECT_PATH}"
  echo "PWD: $( pwd )"
  tree -L 3 -a
  echo
  n2st::draw_horizontal_line_across_the_terminal_window "="
}

function dna::entrypoint_helper_global_init() {

  source /dna-lib-container-tools/project_entrypoints/entrypoint_helper.common.bash || exit 1

  if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == 'true' ]]; then
    dna::show_container_dn_related_tree
  fi

  # ....User feedback................................................................................
  source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

  # ....Show project specific alias..................................................................
  echo -e "Project ${DN_PROJECT_GIT_NAME:?err} specific in-container available alias: ${MSG_DIMMED_FORMAT}
  \n$(
    SP="    " &&
      cd "/dockerized-norlab/dockerized-norlab-images/container-tools" &&
      sed "s;alias dna-${DN_PROJECT_ALIAS_PREFIX:?err}-;${SP}$ dna-${DN_PROJECT_ALIAS_PREFIX}-;" ./dn_bash_alias.bash | sed "s;='.*;;" | grep -e "dna-${DN_PROJECT_ALIAS_PREFIX}-"
  )
  ${MSG_END_FORMAT}"

  # ....Sanity check.................................................................................
  if [[ "${DN_PROJECT_USER}" != "$(whoami)" ]]; then
    n2st::print_msg_error_and_exit "Container login as user $(whoami) does not match project expected user DN_PROJECT_USER=${DN_PROJECT_USER}!\n
Trouble shooting procedure:
  1. Rebuild all images. Make sure build option ${MSG_DIMMED_FORMAT}--skip-core-force-rebuild${MSG_END_FORMAT} is not enable.
  2. If it did not work, open a new terminal on the host machine, login as the desired user and rebuild project-core.
  3. If nothing work, open a bug ticket on https://github.com/norlab-ulaval/dockerized-norlab-project/issues\n
${MSG_DIMMED_FORMAT}$(tree -L 2 -aug "${DN_PROJECT_PATH}")${MSG_END_FORMAT}
"
  fi

  test -n "$(pgrep -x 'sshd')" || n2st::print_msg_warning "Be advised, ssh daemon is not running!\n" 1>&2

  # ....Remove byte-compiled files that could mess with tools on context/environment change..........
  pyclean "${DN_PROJECT_PATH}"
  # Remember the pycharm-debugger user path nightmare
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dna::entrypoint_helper_global_init || n2st::print_msg_error_and_exit "$0 script exited with error!"
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
