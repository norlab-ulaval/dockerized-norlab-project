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

function dna::create_local_multiarch_docker_builder() {
  local builder_name=${1:-"local-builder-multiarch-virtual"}
  echo -e "Checking if ${builder_name} already exist..."
  if docker buildx inspect --bootstrap "${builder_name}" &> /dev/null; then
    #echo -e "\nPruning ${builder_name} cache..."
    #docker buildx prune -f --builder "${builder_name}"

    echo -e "Removing ${builder_name} builder..."
    local buildx_rm_flags=()
    buildx_rm_flags+=(--force)
    #buildx_rm_flags+=(--keep-state)
    docker buildx rm "${buildx_rm_flags[@]}" "${builder_name}"
  fi

  local buildx_extra_flags=()
  buildx_extra_flags+=(--name "${builder_name}")
  buildx_extra_flags+=(--driver docker-container)
  buildx_extra_flags+=(--platform "linux/amd64,linux/arm64" )
  buildx_extra_flags+=(--bootstrap)

  ## Enable automatic loading to local image store for Docker Compose compatibility
  #buildx_extra_flags+=(--driver-opt="default-load=true")

  # Use latest BuildKit image with containerd support
  buildx_extra_flags+=(--driver-opt="image=moby/buildkit:latest")

#  # ....Configuration for --buildkitd-config flag.................................................
#  local buildkitd_flags=()
#  mkdir -p "${DNA_ROOT:?err}/dna_buildx_config"
#  cat > "${DNA_ROOT:?err}/dna_buildx_config/buildkitd.toml" << EOF
#debug = false
## insecure-entitlements allows insecure entitlements, disabled by default.
#insecure-entitlements = [ "network.host", "security.insecure" ]
#
#[worker.oci]
#  enabled = true
#  snapshotter = "overlayfs"
#  # Use native snapshotter for OCI worker to avoid conflicts with containerd
#  # snapshotter = "native"
#
#[worker.containerd]
#  enabled = true
#
#  # Configure snapshotter for containerd worker to access local image store
#  snapshotter = "overlayfs"
#
#  namespace = "buildkit"
#EOF

  # ....Begin........................................................................................
#  echo -e "\nInstanciate new ${builder_name} builder with the following configuration:\n
#    $ docker buildx create ${buildx_extra_flags[*]} --buildkitd-config=${DNA_ROOT:?err}/dna_buildx_config/buildkitd.toml\""
  echo -e "\nInstanciate new ${builder_name} builder with the following configuration:\n$ docker buildx create ${buildx_extra_flags[*]}\n"

#  docker buildx create "${buildx_extra_flags[@]}" --buildkitd-config="${DNA_ROOT:?err}/dna_buildx_config/buildkitd.toml" || return 1
  docker buildx create "${buildx_extra_flags[@]}" || return 1

  echo -e "Inspect new builder...\n"
  docker buildx inspect --builder "${builder_name}"

  return 0
}

dna::create_local_multiarch_docker_builder "$@"

