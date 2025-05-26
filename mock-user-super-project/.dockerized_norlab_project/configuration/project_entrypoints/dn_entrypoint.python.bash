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
#   Read 'DN_PROJECT_PATH'
#
# Notes:
#   - This setup is required for Pycharm based project where the run configuration setup was set to
#     add 'content root' and 'source root' to PYTHONPATH
#   - entrypoint is used by Dockerfile.run-slurm
#
# =================================================================================================
set -e

MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

# ====Setup========================================================================================
if [[ ! -d "${DN_PROJECT_PATH:?'Required DN environment variable is set and not empty'}/src" ]]; then
  echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} '${DN_PROJECT_PATH}/src' directory unreachable!\n Current working directory is '$(pwd)'" 1>&2
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


# ====Execute python command=======================================================================
python3 "$@" || exit 1

# ....Release......................................................................................
echo "$(basename $0) done!"
exit 0
