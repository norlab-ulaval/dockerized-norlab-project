#!/bin/bash

if [[ ! $(docker buildx inspect --bootstrap "local-builder-multiarch-virtual" >/dev/null ) =~ "ERROR: no builder".*"found" ]]; then
  docker buildx rm local-builder-multiarch-virtual
fi

docker buildx create --name local-builder-multiarch-virtual \
    --driver=docker-container \
    --platform linux/amd64,linux/arm64 \
    --buildkitd-flags '--allow-insecure-entitlement network.host' \
    --bootstrap

    #--driver-opt="default-load=true" \

docker buildx inspect

# =================================================================================================
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
