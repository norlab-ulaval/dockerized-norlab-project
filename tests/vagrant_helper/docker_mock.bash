#!/bin/bash
# =================================================================================================
# Mock docker command. Used as a workaround for MacOsX nested virtualization limitations.
#
# Usage;
#   $ bash docker_mock.bash
#   $ docker
#
# =================================================================================================

# Mock docker commands
# - image ls
# - version
# - any arbitrary docker command
function docker() {
  if [[ "$1" == "image" && "$2" == "ls" ]]; then
    echo "test-image-deploy.latest"
  elif [[ "$1" == "version" ]]; then
    echo "Iam docker mock from tests/vagrant_helper/docker_mock.bash"
  else
    echo "Mock docker called with args: $*"
    return 0
  fi
}
export -f docker >/dev/null
