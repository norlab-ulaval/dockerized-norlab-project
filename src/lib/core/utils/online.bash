#!/bin/bash

# =================================================================================================
# Function to check if the system has internet connectivity
#
# Usage:
#     $ dna::is_online [reliable_dns_address]
#
# Arguments:
#   reliable_dns_address      Default to google 8.8.8.8
#
# Outputs:
#   none
#
# Returns:
#   0 if online, 1 if offline
# =================================================================================================
function dna::is_online() {
  local reliable_dns_address=${1:-8.8.8.8}
  ping -c 1 -W 2 "${reliable_dns_address}" > /dev/null 2>&1
  return $?
}

# =================================================================================================
# Check if a git branch exist on a remote git repository i.e., not the current one
#
# Usage:
#     $ dna::check_git_branch_exists_on_remote <remote-url> <branch-name>
#
# Arguments:
#   <remote-url>        The url of the remote
#   <branch-name>
#
# Outputs:
#   none
#
# Returns:
#   0 if branch exist, 1 otherwise
#
# Example in code:
#   if dna::check_git_branch_exists_on_remote "https://github.com/norlab-ulaval/dockerized-norlab" "dev" >/dev/null ; then
#      :  # do something
#   fi
#
# =================================================================================================
function dna::check_git_branch_exists_on_remote() {
  # Parameters: remote_url branch_name
  local remote_url="$1"
  local branch_name="$2"

  if [[ -n "$(git ls-remote --heads "${remote_url}" "${branch_name}")" ]]; then
    n2st::print_msg "Branch ${branch_name} exist on remote repository ${remote_url}"
    return 0  # Branch exists (success)
  else
    n2st::print_msg_warning "Branch ${branch_name} does not exist on remote repository ${remote_url}"
    return 1  # Branch doesn't exist (failure)
  fi
}
