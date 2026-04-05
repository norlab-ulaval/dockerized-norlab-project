#!/bin/bash
# =================================================================================================
# Configuration scheme patching script: v-1 → v0 (TEST ANALOGUE).
#
# Usage:
#   source config_scheme_-1to0.bash
#
# =================================================================================================

# ==== Patch logic starts here ====

# Example: adding a new recommended file from template
# This is also used by tests/tests_bats/test_patch_helper.bats
if [[ -f "${DNA_LIB_PATH}/template/dummy_test_patch.txt" ]]; then
    dna::patch_add_file_if_missing "dummy_test_patch.txt" "dummy_test_patch.txt" "Dummy test patch file"
fi

# ==== Patch logic ends here ====
