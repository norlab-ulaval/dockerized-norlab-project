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
#    echo -e "\nPruning ${builder_name} cache..."
#    docker buildx prune -f --builder "${builder_name}"

    echo -e "\nRemoving ${builder_name} builder..."
    docker buildx rm -f "${builder_name}"
#    docker buildx rm -f "${builder_name}" --keep-state
  fi

  local buildx_extra_flags=()
  buildx_extra_flags+=(--name "${builder_name}")
  buildx_extra_flags+=(--driver docker-container)
  buildx_extra_flags+=(--platform "linux/amd64,linux/arm64" )
  buildx_extra_flags+=(--bootstrap)

  # Enable automatic loading to local image store for Docker Compose compatibility
  buildx_extra_flags+=(--driver-opt="default-load=true")

  # Use latest BuildKit image with containerd support
  buildx_extra_flags+=(--driver-opt="image=moby/buildkit:latest")

##  # ....Configuration for --buildkitd-config flag.................................................
#  local buildkitd_flags=()
#  mkdir -p "${DNA_ROOT:?err}/dna_buildx_config"
#  cat > "${DNA_ROOT:?err}/dna_buildx_config/buildkitd.toml" << EOF
#debug = true
## insecure-entitlements allows insecure entitlements, disabled by default.
#insecure-entitlements = [ "network.host", "security.insecure" ]
#
#[worker.oci]
#  enabled = true
#  # Use native snapshotter for OCI worker to avoid conflicts with containerd
#  snapshotter = "overlayfs"
#  # snapshotter = "native"
#
###[worker.containerd]
###  enabled = true
###
###  # Configure snapshotter for containerd worker to access local image store
###  snapshotter = "overlayfs"
###
#EOF

  # ....Begin........................................................................................
#  echo -e "\nInstanciate new ${builder_name} builder with the following configuration:\n
#    $ docker buildx create ${buildx_extra_flags[*]} --buildkitd-config=${DNA_ROOT:?err}/dna_buildx_config/buildkitd.toml\""
  echo -e "\nInstanciate new ${builder_name} builder with the following configuration:\n
    $ docker buildx create ${buildx_extra_flags[*]}"

  docker buildx create "${buildx_extra_flags[@]}" || return 1
#  docker buildx create "${buildx_extra_flags[@]}" --buildkitd-config="${DNA_ROOT:?err}/dna_buildx_config/buildkitd.toml" || return 1

  echo -e "Inspect new builder..."
  docker buildx inspect --builder "${builder_name}"
  echo

  return 0
}

create_local_multiarch_docker_builder "$@"

