#!/bin/bash
# =================================================================================================
# DNA shell requirements install
#
# Notes:
# - This file is used in .dockerized_norlab/configuration/Dockerfile via pip install
# - It is executed before python.requirements.txt
#
# =================================================================================================

# ....Example 1....................................................................................
apt-get update &&  \
  apt-get install -y vim && \
  apt-get autoremove -y &&  \
  apt-get clean \
   || exit 1
# .................................................................................................

# ....Example 2....................................................................................
# Install repository root public requirement file
if [[ -f "${DN_PROJECT_PATH}/requirements.txt"  ]]; then
  pip3 install --verbose -r "${DN_PROJECT_PATH}/requirements.txt" || exit 1
fi
# .................................................................................................
