#!/bin/bash
# =================================================================================================
# Dockerized-NorLab container up_and_attach.bash entrypoint callback.
# Is executed only once on container initialisation, at the end of the project-develop or
# project-deploy entrypoints.
#
# Usage:
#   Add project wide logic that need to be executed only on startup.
#
# Globals:
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e

# ====DN-project internal logic====================================================================
if [[ ${DN_ENTRYPOINT_TRACE_EXECUTION} == true ]]; then
  n2st::print_msg "Execute ${BASH_SOURCE[0]}"
fi

# ....User feedback................................................................................
source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash

# ....Show project specific alias..................................................................
echo -e "Project ${DN_PROJECT_GIT_NAME:?err} specific in-container available alias: \033[1;37m
\n$(
  SP="    " &&
    cd "/dockerized-norlab/dockerized-norlab-images/container-tools" &&
    sed "s;alias dn${DN_PROJECT_ALIAS_PREFIX:?err}_;${SP}$ dn${DN_PROJECT_ALIAS_PREFIX}_;" ./dn_bash_alias.bash | sed "s;='.*;;" | grep -e "dn${DN_PROJECT_ALIAS_PREFIX}"_
)
\033[0m"

# ....Sanity check.................................................................................
test -n "$(pgrep -x 'sshd')" || echo "ssh daemon is not running!"

# ....Remove byte-compiled files that could mess with tools on context/environment change..........
pyclean "${DN_PROJECT_PATH}"
# Remember the pycharm-debugger user path nightmare

# ====DN-project user defined logic================================================================

# ....Project specific info........................................................................
echo -e "Project ${DN_PROJECT_GIT_NAME:?err} specific information: \033[1;37m
\n$(
  SP="    " \
  && pip --disable-pip-version-check list | grep -i -e hydra -e omegaconf | sed "s;hydra;${SP}hydra;" | sed "s;omega;${SP}omega;" \
  && pip --disable-pip-version-check list --exclude hydra-optuna-sweeper | grep -i -e optuna | sed "s;^optuna;${SP}optuna;"
)
\033[0m"

