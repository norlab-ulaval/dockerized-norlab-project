#!/bin/bash
# =================================================================================================
# Sanity check related related utility functions
#
# Usage:
#   From a DNA container
#   >>> COPY --from=context-dna-lib-container-tools . /dna-lib-container-tools/
#   >>> RUN source /dna-lib-container-tools/dn_sanity_checks.bash && dna::user_and_dir_content_sanity_check
#
# Global:
#   read DN_SHOW_DEBUG_INFO
#   read DN_PROJECT_USER
#   read DN_SSH_SERVER_USER
#   read DN_CONTAINER_TOOLS_LOADED
#   read DEBIAN_FRONTEND
#
# =================================================================================================

function dna::user_and_dir_content_sanity_check() {
  # ....Setup......................................................................................
  local executed_at=${1:-runtime}
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Begin......................................................................................
  if [[ ${DN_SHOW_DEBUG_INFO} == true ]]; then
    MSG_WARNING_FORMAT="\033[1;33m"
    MSG_END_FORMAT="\033[0m"
    echo -e "${MSG_WARNING_FORMAT}[DN ${executed_at} debug]${MSG_END_FORMAT} Check container DN env var..."
    echo
    printenv | grep -e DN_ -e DNA_ -e PATH -e PYTHONPATH
    echo
    echo -e "${MSG_WARNING_FORMAT}[DN ${executed_at} debug]${MSG_END_FORMAT} Check container available files..."
    echo
    tree -aguL 1  -I .git /
    echo
    tree -aguL 1  -I .git "$HOME"
    echo
    tree -aguL 3 -I .git -I utilities "${DN_PATH}"
    echo
    tree -aguL 3 /dna-lib-container-tools
    echo
    tree -aguL 3 /project_entrypoints
    echo
    echo -e "${MSG_WARNING_FORMAT}[DN ${executed_at} debug]${MSG_END_FORMAT} Users configuration sanity check..."
    echo
    echo "whoami: $(whoami)"
    echo "id ${DN_PROJECT_USER}: $(id "${DN_PROJECT_USER}")"
    if [[ -n ${DN_SSH_SERVER_USER}  ]]; then
      echo "id ${DN_SSH_SERVER_USER}: $(id "${DN_SSH_SERVER_USER}")"
      echo "DN_SSH_SERVER_USER: ${DN_SSH_SERVER_USER}"
    else
      echo "DN_SSH_SERVER_USER not set"
    fi
    echo "DN_CONTAINER_TOOLS_LOADED: ${DN_CONTAINER_TOOLS_LOADED}"
    echo "DEBIAN_FRONTEND: ${DEBIAN_FRONTEND}"
    echo
    echo -e "${MSG_WARNING_FORMAT}[DN ${executed_at} debug]${MSG_END_FORMAT} Check users setup..."
    echo "getent passwd root: $(getent passwd root)"
    echo "getent passwd ${DN_PROJECT_USER}: $(getent passwd "${DN_PROJECT_USER}")"
    if [[ -n ${DN_SSH_SERVER_USER}  ]]; then
      echo "getent passwd ${DN_SSH_SERVER_USER}: $(getent passwd "${DN_SSH_SERVER_USER}")"
    fi
    echo
  fi

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}
