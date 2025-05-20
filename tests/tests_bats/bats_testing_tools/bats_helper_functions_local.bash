
# ====Docker related===============================================================================

function mock_docker_command_exit_ok() {
    function docker() {
      local tmp=$@
      return 0
    }
}

function mock_docker_command_exit_error() {
    function docker() {
      local tmp=$@
      echo "Error" 1>&2
      return 1
    }
}

function mock_docker_command_config_services() {
    function docker() {
      local tmp=$@
      echo "mock-service-one mock-service-two"
      return 0
    }
}

# ====bats-file utility============================================================================

# Temporary workaround for missing implementation of "assert_file_not_contains"
# ToDo: remove when bats-file PR #61 is merge
#   - PR #61 at https://github.com/bats-core/bats-file/pull/61
#
# Credit https://github.com/bats-core/bats-file/pull/61
#
# Fail and display path of the file (or directory) if it does contain a string.
# This function is the logical complement of `assert_file_contains'.
#
# Globals:
#   BATSLIB_FILE_PATH_REM
#   BATSLIB_FILE_PATH_ADD
# Arguments:
#   $1 - path
#   $2 - regex
# Returns:
#   0 - file does not contain regex
#   1 - otherwise
# Outputs:
#   STDERR - details, on failure
assert_file_not_contains() {
  local -r file="$1"
  local -r regex="$2"

  if [[ ! -f "$file" ]]; then
    local -r rem="${BATSLIB_FILE_PATH_REM-}"
    local -r add="${BATSLIB_FILE_PATH_ADD-}"
    batslib_print_kv_single 4 'path' "${file/$rem/$add}" 'regex' "$regex" \
      | batslib_decorate 'file does not exist' \
      | fail

  elif grep -q "$regex" "$file"; then
    local -r rem="${BATSLIB_FILE_PATH_REM-}"
    local -r add="${BATSLIB_FILE_PATH_ADD-}"
    batslib_print_kv_single 4 'path' "${file/$rem/$add}" 'regex' "$regex" \
      | batslib_decorate 'file contains regex' \
      | fail

  fi
}
