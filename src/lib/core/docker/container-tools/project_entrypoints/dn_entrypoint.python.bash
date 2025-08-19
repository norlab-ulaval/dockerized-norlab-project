#!/bin/bash
# =================================================================================================
# Project aware python app entrypoint.
#   - add project to the PYTHONPATH in compliance with Pycharm setup
#   - cleanup byte-compiled files to prevent execution bug
#
# Usage:
#   $ bash dn_entrypoint.python.bash [<any-python-arg>]
#
# Globals:
#   Read DN_PROJECT_PATH
#   Read PYTHONPATH
#
# Notes:
#   - This setup is required for Pycharm based project where the run configuration setup was set to
#     add 'content root' and 'source root' to PYTHONPATH
#
# =================================================================================================
set -e

# ====Setup========================================================================================
if [[ ! -d "${DN_PROJECT_PATH:?'Required DN environment variable is set and not empty'}/src" ]]; then
  echo -e "\n\033[1;31m[DN error]\033[0m '${DN_PROJECT_PATH}/src' directory unreachable!\n Current working directory is '$(pwd)'" 1>&2
  exit 1
else
  cd "${DN_PROJECT_PATH}/src" || exit 1
fi

# Add the DN-project path to python path (see header Notes).
export PYTHONPATH="${DN_PROJECT_PATH:?err}:${PYTHONPATH:?err}"
# (NICE TO HAVE) ToDo: refactor PYTHONPATH logic as a fct. Either in DN container-tools or in DN-project

# Remove byte-compiled files that can mess with tools on context/environment change (Remember the
# non-interactive-ros2 user path nightmare)
pyclean "${DN_PROJECT_PATH}"

# ....Load library.................................................................................
if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  echo -e "\033[1;33m[DN trace]\033[0m Execute dn_entrypoint.python.bash"
fi

if [[ $- == *i* ]]; then
    if [[ "${DN_ENTRYPOINT_TRACE_EXECUTION}" == true ]]; then
      echo -e "\033[1;33m[DN trace]\033[0m Interactive shell. Sourcing DN lib is handled via .bashrc"
    fi
else
    if [[ "${DN_ENTRYPOINT_TRACE_EXECUTION}" == true ]]; then
      echo -e "\033[1;33m[DN trace]\033[0m Non-interactive shell. Sourcing DN lib"
    fi
    source /dockerized-norlab/dockerized-norlab-images/container-tools/bash_run_config/.bashrc.dn_non_interactive
fi

test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }

# ....source ROS2 environment variables............................................................
#dn::source_ros2_underlay_only
#dn::source_ros2_overlay_only
dn::source_ros2

# ====Execute python command=======================================================================
python3 "$@" || exit 1

# ....Release......................................................................................
n2st::print_msg_done "dn_entrypoint.python.bash done!"
exit 0
