#!/bin/bash
# =================================================================================================
# Configuration scheme patching script: v<FROM> → v<TO>.
#
# This script should be named 'config_scheme_<FROM>to<TO>.bash' and placed in 'src/lib/core/patches/'.
#
# Usage:
#   source config_scheme_<FROM>to<TO>.bash
#
# =================================================================================================

# ==== Patch logic starts here ====

# Use the following helper functions to add missing files/directories/content:
#
# dna::patch_add_file_if_missing <template_source> <target_dest> <description>
# dna::patch_add_directory_if_missing <template_source> <target_dest> <description>
# dna::patch_add_content_if_missing <target_file> <search_string> <content_to_add> <description>
# dna::patch_modify_content <target_file> <search_pattern> <replace_pattern> <description>
#
# Examples:
# dna::patch_add_file_if_missing ".gitignore" ".gitignore" "Git ignore file"
# dna::patch_add_directory_if_missing "src/launcher" "src/launcher" "Launcher directory"
# dna::patch_add_content_if_missing ".env" "NEW_VAR=" "NEW_VAR=value" "New environment variable"
# dna::patch_modify_content ".env" "OLD_VAR=v1" "OLD_VAR=v2" "Update environment variable"

# Example: adding a new recommended file from template
# dna::patch_add_file_if_missing "dummy_test_patch.txt" "dummy_test_patch.txt" "Dummy test patch file"

# ==== Patch logic ends here ====
