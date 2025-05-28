#!/bin/bash
# =================================================================================================
# Setup host computer (i.e, workstation, embed computer, server) for using this
# Dockerized-NorLab-Project super project.
#
# Usage:
#   $ source setup_host_for_this_super_project.bash
#
# =================================================================================================
pushd "$(pwd)" >/dev/null || exit 1

# (Priority) ToDo: Make the script multi-arch and multi-os by calling specialize version
# see:
# - `libpointmatcher-build-system build_system/lpm_server_config.bash`
# - https://github.com/vaul-ulaval/f1tenth_controller/blob/272573b99855779428fccd122b5f84fc172e0767/setup_dockerized_norlab_for_this_repo.bash
# - https://github.com/norlab-ulaval/dockerized-norlab/blob/8975e05e69ddc57cb858a3413ea7c98920f5422b/jetson_xavier_install.bash

function dnp::setup_host_for_this_super_project() {
  local SCRIPT_PATH
  local SCRIPT_PATH_PARENT

  # Note: can handle both sourcing cases
  #   i.e. from within a script or from an interactive terminal session
  SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"

  # ....Source project shell-scripts dependencies..................................................
  source "${SCRIPT_PATH_PARENT}/import_dnp_lib.bash" || exit 1
  source "${SCRIPT_PATH_PARENT}/load_super_project_config.bash" || exit 1

  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
  echo -e "${MSG_DONE} The '$(basename "${SUPER_PROJECT_ROOT}")' dir is reachable. Ready to install alias"

  DN_PROJECT_ALIAS_PREFIX="dnp${DN_PROJECT_ALIAS_PREFIX:?err}"

  # ...aliasing dev................................................................................
  # ref:
  # - https://www.baeldung.com/linux/bash-alias-with-parameters
  # - https://unix.stackexchange.com/questions/3773/how-to-pass-parameters-to-an-alias

  (
    echo ""
    echo "#>>>>DNP ${SUPER_PROJECT_REPO_NAME:?err} aliases and env variable"
    echo "export ${DN_PROJECT_ALIAS_PREFIX:?err}_DIR_PATH=${SUPER_PROJECT_ROOT:?err}"
    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_cd='cd $SUPER_PROJECT_ROOT'"
    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_cdd='cd ${SUPER_PROJECT_ROOT}/.dockerized_norlab_project'"
    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_cddd='cd ${SUPER_PROJECT_ROOT}/src'"
#    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_up_and_attach='cd ${SUPER_PROJECT_ROOT}/src/lib/core/execute && bash up_and_attach.bash'"
#    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_down='cd ${SUPER_PROJECT_ROOT}/src/lib/core/execute && bash down.bash'"
#    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_build_all='cd ${SUPER_PROJECT_ROOT}/src/lib/core/execute && bash build.all.bash'"
#    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_build_ci_tests='cd ${SUPER_PROJECT_ROOT}/src/lib/core/execute && bash build.ci_tests.bash'"
#    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_build_and_run_ci_tests='cd ${SUPER_PROJECT_ROOT}/src/lib/core/execute && bash build.ci_tests.bash && bash run.ci_tests.bash'"
#    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_build_develop='cd ${SUPER_PROJECT_ROOT}/src/lib/core/execute && bash build.develop.bash'"
#    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_build_deploy='cd ${SUPER_PROJECT_ROOT}/src/lib/core/execute && bash build.deploy.bash'"
#    echo "alias ${DN_PROJECT_ALIAS_PREFIX}_attach_to_${DN_CONTAINER_NAME:?err}='clear && dn_attach ${DN_CONTAINER_NAME}'"
    echo "#<<<<DNP ${SUPER_PROJECT_REPO_NAME:?err} aliases and env variable end"
    echo ""
  ) >>~/.bashrc

  # --env=\"DISPLAY=:0\"

  # ...CUDA toolkit path...........................................................................
  # ref dusty_nv comment at https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068
  if [[ $(uname -s) == "Darwin" ]]; then
    echo -e "${MSG_ERROR} CUDA is not supported yet on Apple M1 computer"
  else
    if ! command -v nvcc -V &>/dev/null; then
      # nvcc command not working
      echo -e "${MSG_WARNING} Fixing CUDA path for nvcc"

      (
        echo ""
        echo "# CUDA toolkit related"
        echo "# ref dusty_nv comment at"
        echo "#    https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068"
        echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}"
        echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
        echo ""
      ) >>~/.bashrc

      source ~/.bashrc && echo -e "${MSG_DONE} CUDA path hack added to ~/.bashrc for nvcc"
    fi

    if [[ $(nvcc -V | grep 'nvcc: NVIDIA (R) Cuda compiler driver') == "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
      echo -e "${MSG_DONE} nvcc installed properly"
      nvcc -V
    else
      echo -e "${MSG_ERROR} Check your nvcc installation. It's NOT installed properly!"
    fi
  fi

  if [ -n "$ZSH_VERSION" ]; then
    # ToDo: validate >> appending .bashrc to .zshrc should let to the user choice
    #  echo "source ~/.bashrc" >> ~/.zshrc
    source ~/.zshrc
  elif [ -n "$BASH_VERSION" ]; then
    source ~/.bashrc
  else
    echo -e "${MSG_ERROR} Unknown shell! Check with the maintainer to add it to DS"
  fi

  echo -e "${MSG_DONE} Setup completed!

    New available alias added to ~/.bashrc:
      - ${DN_PROJECT_ALIAS_PREFIX}_cd
      - ${DN_PROJECT_ALIAS_PREFIX}_up_and_attach
      - ${DN_PROJECT_ALIAS_PREFIX}_down
      - ${DN_PROJECT_ALIAS_PREFIX}_build
      - ${DN_PROJECT_ALIAS_PREFIX}_attach_to_${DN_CONTAINER_NAME}

    New env variable
      - ${DN_PROJECT_ALIAS_PREFIX}_DIR_PATH
  "
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  dnp::setup_host_for_this_super_project
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
