#!/bin/bash
# =================================================================================================
# DNA shell requirements install
#
# Notes:
# - This file is used in .dockerized_norlab/configuration/Dockerfile via pip install
# - It is executed before python.requirements.txt
# - N2ST library is available in script i.e., shell script function prefixed 'n2st::'
#
# =================================================================================================

# ....Update pip to latest.........................................................................
python3 -m pip install --upgrade pip

# ....Example 1....................................................................................
{
  apt-get update \
  && apt-get install --assume-yes --no-install-recommends \
    vim \
    tree \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* ;
} || n2st::print_msg_error_and_exit "Failed apt-get package install!"
# .................................................................................................

# ....Example 2....................................................................................
if [[ -f "${DN_PROJECT_PATH:?err}/requirements.txt"  ]]; then
  n2st::print_msg "Execute pip install from repository root python requirement file..."
  pip3 install --verbose -r "${DN_PROJECT_PATH}/requirements.txt" \
    || n2st::print_msg_error_and_exit "Failed pip install from repository root requirement!"
fi
# .................................................................................................

# ....Example 3....................................................................................
if [[ $( n2st::which_architecture_and_os ) == "l4t\arm64" ]]; then
  n2st::print_msg "Is running on a Jetson..."
  # Add Jetson logic e.g., cat /proc/device-tree/model
fi
# .................................................................................................

# ....Example 4....................................................................................
if [[ $( n2st::which_python3_version ) == 3.10 ]]; then
  n2st::print_msg "Execute python 3.10 specialized install..."
  # Add python logic
fi
# .................................................................................................
