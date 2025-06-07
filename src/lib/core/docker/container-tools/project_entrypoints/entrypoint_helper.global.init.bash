#!/bin/bash
# =================================================================================================
#
# This aggregate global init entrypoint logic so that dn_entrypoint.global.init.callback.bash be
# less verbose for user.
#
# =================================================================================================
pushd "$(pwd)" >/dev/null || exit 1

function dnp::show_container_dn_related_tree() {
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo "Dev tools to check in container available directories and files"
  echo
  tree -L 4 -a /dnp-lib-container-tools
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

function dnp::entrypoint_helper_global_init() {

  source /dnp-lib-container-tools/project_entrypoints/entrypoint_helper.common.bash || exit 1

  if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == 'true' ]]; then
    dnp::show_container_dn_related_tree
  fi

  # ....User feedback................................................................................
  source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

  # ....Show project specific alias..................................................................
  echo -e "Project ${DN_PROJECT_GIT_NAME:?err} specific in-container available alias: ${MSG_DIMMED_FORMAT}
  \n$(
    SP="    " &&
      cd "/dockerized-norlab/dockerized-norlab-images/container-tools" &&
      sed "s;alias dnp-${DN_PROJECT_ALIAS_PREFIX:?err}-;${SP}$ dnp-${DN_PROJECT_ALIAS_PREFIX}-;" ./dn_bash_alias.bash | sed "s;='.*;;" | grep -e "dnp-${DN_PROJECT_ALIAS_PREFIX}-"
  )
  ${MSG_END_FORMAT}"

  # ....Sanity check.................................................................................
  test -n "$(pgrep -x 'sshd')" || n2st::print_msg_warning "Be advised, ssh daemon is not running!\n" 1>&2

  # ....Remove byte-compiled files that could mess with tools on context/environment change..........
  pyclean "${DN_PROJECT_PATH}"
  # Remember the pycharm-debugger user path nightmare

}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dnp::entrypoint_helper_global_init || n2st::print_msg_error_and_exit "$0 script exited with error!"
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
