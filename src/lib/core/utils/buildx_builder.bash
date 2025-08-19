#!/bin/bash
# =================================================================================================
# Simple docker buildx builder instanciation script.
# Delete existing builder if it exist and create a new one.
#
# .................................................................................................
# Optional image store configuration (for non Docker Desktop user)
#
# Add the following to your '/etc/docker/daemon.json' configuration
#
#  {
#      "features": {
#        "containerd-snapshotter": true
#      }
#  }
#
# Restart docker daemon:
#   $ sudo systemctl restart docker
#
# =================================================================================================

# ToDo: minimal unit-test

function create_local_multiarch_docker_builder() {
  local builder_name=${1:-"local-builder-multiarch-virtual"}
  if docker buildx inspect --bootstrap "${builder_name}" &> /dev/null; then
    echo -e "\nPruning ${builder_name} cache..."
    docker buildx prune -f --builder "${builder_name}"

    echo -e "\nRemoving ${builder_name} builder..."
    docker buildx rm -f "${builder_name}"
  fi

  local buildx_extra_flags=()
  buildx_extra_flags+=(--name "${builder_name}")
  buildx_extra_flags+=(--driver docker-container)
  buildx_extra_flags+=(--platform "linux/amd64,linux/arm64" )
  buildx_extra_flags+=(--bootstrap)
  buildx_extra_flags+=( --buildkitd-flags '--allow-insecure-entitlement network.host' )
#  buildx_extra_flags+=( --buildkitd-flags '--allow-insecure-entitlement network.host --oci-worker-snapshotter=containerd --containerd-worker=true' )
#  buildx_extra_flags+=( --driver-opt="default-load=true" )

  echo -e "\nInstanciate new ${builder_name} builder with the following configuration:\n
    $ docker buildx create ${buildx_extra_flags[*]}\n"
  docker buildx create "${buildx_extra_flags[@]}" || return 1

  echo -e "Inspect new builder..."
  docker buildx inspect --builder "${builder_name}"
  echo

  return 0
}

create_local_multiarch_docker_builder "$@"

