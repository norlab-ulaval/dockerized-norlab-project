#!/bin/bash

# =================================================================================================
# Function to check if the system has internet connectivity
#
# Usage:
#     $ dnp::is_online
#
# Arguments:
#   none
# Outputs:
#   none
# Returns:
#   0 if online, 1 if offline
# =================================================================================================
function dnp::is_online() {
    ping -c 1 -W 2 8.8.8.8 > /dev/null 2>&1
    return $?
}
