#!/bin/bash

# =================================================================================================
# Tests PYTHONPATH for a specific path, add it if missing
#
# Usage example:
#   $ source /dna-lib-container-tools/project_entrypoints/dn_entrypoint_pythonpath_checks.bash "TARGET_PATH"
#
# Arguments:
#   TARGET_PATH     The path to check (can be relative)
# =================================================================================================

# ToDo: implement minimal unit-test (ref task NMO-793)

# ....Sanity check.................................................................................
test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && exit 1; }

# ....Function.....................................................................................
dna::add_to_pythonpath_if_missing() {
    local dir_path="$1"

    # Check if directory exists
    if [[ ! -d "$dir_path" ]]; then
        n2st::print_msg_warning "Directory $dir_path does not exist! Skip"
        return 0
    fi

    # Convert to absolute path
    dir_path=$(realpath "$dir_path")

    # Check if already in PYTHONPATH
    if [[ ":$PYTHONPATH:" != *":$dir_path:"* ]]; then
        export PYTHONPATH="${dir_path}${PYTHONPATH:+:$PYTHONPATH}"
        n2st::print_msg "Added $dir_path to PYTHONPATH"
    else
        n2st::print_msg "$dir_path is already in PYTHONPATH"
    fi
    return 0
}

dna::add_to_pythonpath_if_missing "$@"
