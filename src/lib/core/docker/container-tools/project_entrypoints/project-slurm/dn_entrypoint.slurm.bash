#!/bin/bash
# =================================================================================================
# Project aware python app entrypoint.
#   - add project to the PYTHONPATH in compliance with Pycharm setup
#   - cleanup byte-compiled files to prevent execution bug
#
# Usage:
#   $ bash dn_entrypoint.slurm.bash [<any-python-arg>]
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

MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

# ====Setup========================================================================================
if [[ ! -d "${DN_PROJECT_PATH:?'Required DN environment variable is set and not empty'}/src" ]]; then
  echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '${DN_PROJECT_PATH}/src' directory unreachable!\n Current working directory is '$(pwd)'" 1>&2
  exit 1
else
  cd "${DN_PROJECT_PATH}/src" || exit 1
fi

# Add the DN-project path to python path (see header Notes).
export PYTHONPATH="${DN_PROJECT_PATH:?err}:${PYTHONPATH:?err}"
# (NICE TO HAVE) ToDo: refactor PYTHONPATH logic as a fct. Either in DN container-tools or in DN-project

# Remove byte-compiled files that can mess with tools on context/environment change (Remember the
# pycharm-debugger user path nightmare)
pyclean "${DN_PROJECT_PATH}"

# ....Load library.................................................................................
source /import_dockerized_norlab_container_tools.bash
n2st::set_which_python3_version && test -n "${PYTHON3_VERSION}" || exit 1
if [[ -z "${PYTHON3_VERSION}" ]]; then
  echo -e "[\033[1;31mERROR\033[0m] $0 | Script import_dockerized_norlab_container_tools.bash failled" 1>&2
fi

if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  n2st::print_msg "Execute $0"
fi

# ====DN-project user defined logic================================================================

# ....Execute DN-project user callback.............................................................
# Sanity check
test -d "/project_entrypoints" || n2st::print_msg_error_and_exit "Dir /project_entrypoints is unreachable"
test -d "/project_entrypoints/project-slurm" || n2st::print_msg_error_and_exit "Dir /project_entrypoints/project-slurm is unreachable"

if [[ -f /project_entrypoints/dn_entrypoint.global.init.callback.bash ]]; then
  source /project_entrypoints/dn_entrypoint.global.init.callback.bash || exit 1
else
  n2st::print_msg_warning "dn_entrypoint.global.init.callback.bash unavailable"
fi

if [[ -f /project_entrypoints/project-slurm/dn_entrypoint.init.callback.bash ]]; then
  source /project_entrypoints/project-slurm/dn_entrypoint.init.callback.bash || exit 1
else
  n2st::print_msg_warning "project-slurm/dn_entrypoint.init.callback.bash unavailable"
fi

# (CRITICAL) ToDo: on task end >> delete next bloc ↓↓
echo "whoami: $(whoami)"
tree -L 2 -aug "${DN_PROJECT_PATH}"
tree -L 4 -aug "${DN_PROJECT_PATH}/artifact"
sudo mkdir -p "${DN_PROJECT_PATH}/artifact/mock_experiment_tmp/"
sudo chown -R "$(id -u "${DN_PROJECT_USER:?err}"):$(id -g "${DN_PROJECT_USER}")" "${DN_PROJECT_PATH}"
tree -L 4 -aug "${DN_PROJECT_PATH}/artifact"

# ====Execute python command=======================================================================
cd "${DN_PROJECT_PATH}/src" || exit 1
python3 "$@" || exit 1

# (CRITICAL) ToDo: on task end >> delete next bloc ↓↓
tree -L 2 -aug "${DN_PROJECT_PATH}"
tree -L 4 -aug "${DN_PROJECT_PATH}/artifact"

# ....Release......................................................................................
echo "$(basename $0) done!"
exit 0
