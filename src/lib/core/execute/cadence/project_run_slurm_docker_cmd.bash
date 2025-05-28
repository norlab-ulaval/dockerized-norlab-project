#!/bin/bash

#
# (NICE TO HAVE) inprogress: RLRP-395 chore: add cadence support to dev workflow
#
# The following docker run command was generated from `docker-compose.project.run.slurm.yaml`
# via `https://www.decomposerize.com`.
#
#
docker run \
  --net host \
  -e DN_CONTAINER_NAME="${DN_CONTAINER_NAME:?err}-slurm${SJOB_ID}" \
  -e DN_ENTRYPOINT_TRACE_EXECUTION=false \
  -e DN_PROJECT_USER="${DN_PROJECT_USER:?err}" \
  -e DN_ACTIVATE_POWERLINE_PROMT=true \
  -e IS_TEAMCITY_RUN="${IS_TEAMCITY_RUN}" \
  -e IS_SLURM_RUN="${IS_SLURM_RUN}" \
  -e ROS_DOMAIN_ID=1 \
  -e CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES}" \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterface>${DN_DDS_NETWORK_INTERFACE:-eth0}</></></></>" \
  -v /etc/localtime:/etc/localtime:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
  -v ${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab_project/configuration/project_entrypoints/:/project_entrypoints/:ro \
  -v ${SUPER_PROJECT_ROOT}/.dockerized_norlab_project/dn_container_env_variable/:/dn_container_env_variable/:rw \
  -v ${SUPER_PROJECT_ROOT}/:/ros2_ws/src/"${DN_PROJECT_GIT_NAME:?err}"/:rw \
  -v ${SUPER_PROJECT_ROOT}/artifact:/ros2_ws/src/"${DN_PROJECT_GIT_NAME:?err}"/artifact/:rw \
  --security-opt seccomp=unconfined \
  --security-opt apparmor=unconfined \
  --cap-add SYS_PTRACE \
  --cap-add SYS_NICE \
  --pid host \
  --ipc host \
  --init \
  --runtime nvidia \
  "${DN_PROJECT_HUB:?err}/${DN_PROJECT_IMAGE_NAME:?err}-slurm:${PROJECT_TAG:?err}"
