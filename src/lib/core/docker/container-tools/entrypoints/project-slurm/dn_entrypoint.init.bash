#!/bin/bash
# =================================================================================================
# Project aware python app entrypoint.
#   - add project to the PYTHONPATH in compliance with Pycharm setup
#   - cleanup byte-compiled files to prevent execution bug
#
# Usage:
#   $ bash dn_entrypoint.init.bash [<any-python-arg>]
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

# Remove byte-compiled files that can mess with tools on context/environment change (Remember the
# non-interactive-ros2 user path nightmare)
pyclean "${DN_PROJECT_PATH}"

# ....Load library.................................................................................
if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  echo -e "\033[1;33m[DN trace]\033[0m Execute project-slurm/dn_entrypoint.init.bash"
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

# Add the DN-project path to python path if missing (see header Notes).
source /dna-lib-container-tools/entrypoints/dn_entrypoint_pythonpath_checks.bash "${DN_PROJECT_PATH:?err}/src"

# ====DNA-project user defined logic===============================================================

# ....Execute DN-project user callback.............................................................
# Sanity check
test -d "/entrypoints" || n2st::print_msg_error_and_exit "Dir /entrypoints is unreachable"
test -d "/entrypoints/project-slurm" || n2st::print_msg_error_and_exit "Dir /entrypoints/project-slurm is unreachable"

if [[ -f /entrypoints/dn_entrypoint.global.init.callback.bash ]]; then
  source /entrypoints/dn_entrypoint.global.init.callback.bash || exit 1
else
  n2st::print_msg_warning "dn_entrypoint.global.init.callback.bash unavailable"
fi

if [[ -f /entrypoints/project-slurm/dn_entrypoint.init.callback.bash ]]; then
  source /entrypoints/project-slurm/dn_entrypoint.init.callback.bash || exit 1
else
  n2st::print_msg_warning "project-slurm/dn_entrypoint.init.callback.bash unavailable"
fi

# ====Execute python command=======================================================================
cd "${DN_PROJECT_PATH}/src" || exit 1
python3 "$@" || exit 1

# ....Release......................................................................................
n2st::print_msg_done "project-slurm/dn_entrypoint.init.bash done!"
exit 0
