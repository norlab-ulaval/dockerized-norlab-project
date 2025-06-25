# =================================================================================================
#
# Usage:
#   - 👍 You can change code in the 'user-project-custom-steps' stage.
#     See the line with the "↓ ↓ ↓ ..." character below.
#   - ⚠️ Dont change code the first stage (init-and-setup) or in the last stage (final) unless you
#     know what your doing.
#
# =================================================================================================
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS init-and-setup

# ====Dockerized-NorLab-project internal configuration step========================================
ARG DEBIAN_FRONTEND=noninteractive

ARG DN_PROJECT_GIT_NAME
ENV DN_PROJECT_GIT_NAME=${DN_PROJECT_GIT_NAME:?'Build argument needs to be set and non-empty.'}
ARG DN_PROJECT_GIT_DOMAIN
ENV DN_PROJECT_GIT_DOMAIN=${DN_PROJECT_GIT_DOMAIN:?'Build argument needs to be set and non-empty.'}
ARG DN_PROJECT_ALIAS_PREFIX
ENV DN_PROJECT_ALIAS_PREFIX=${DN_PROJECT_ALIAS_PREFIX:?'Build argument needs to be set and non-empty.'}
ARG DN_PROJECT_USER
ENV DN_PROJECT_USER=${DN_PROJECT_USER:?'Build argument needs to be set and non-empty.'}
ENV DN_PROJECT_USER_HOME=/home/${DN_PROJECT_USER}
ARG DN_PROJECT_UID
ENV DN_PROJECT_UID=${DN_PROJECT_UID:?'Build argument needs to be set and non-empty.'}
ARG DN_PROJECT_GID
ENV DN_PROJECT_GID=${DN_PROJECT_GID:?'Build argument needs to be set and non-empty.'}
ARG IS_TEAMCITY_RUN
ENV IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:-false}

# ====Set PATH related environment variable========================================================
ENV DN_PROJECT_PATH=${DN_DEV_WORKSPACE:?'Build argument needs to be set and non-empty.'}/src/${DN_PROJECT_GIT_NAME}
ENV PATH="${DN_PROJECT_PATH}:${PATH}"
ENV PYTHONPATH="${DN_PROJECT_PATH}/src:${DN_PROJECT_PATH}/tests:${DN_PROJECT_PATH}/external_data:${PYTHONPATH}"

# ====Copy init and setup files====================================================================
COPY .dockerized_norlab/configuration/project_requirements/python.requirements.txt /python.requirements.txt
COPY .dockerized_norlab/configuration/project_requirements/shell.requirements.bash /shell.requirements.bash
COPY .dockerized_norlab/configuration/project_entrypoints /project_entrypoints
COPY --from=context-dn-container-tools ./dn_project_core_init.bash /dn_project_core_init.bash
COPY --from=context-dna-lib-container-tools . /dna-lib-container-tools/

RUN <<EOF
    source /dn_project_core_init.bash || exit 1
    source /dna-lib-container-tools/dn_project_core.setup.bash || exit 1
    source /dna-lib-container-tools/dn_project_core.build.patch.bash || exit 1
EOF


# ====Matplotlib related===========================================================================
# See dn_project_core.setup.bash for details
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# ====User project custom steps====================================================================
FROM init-and-setup AS user-project-custom-steps
# USER NOTES: ADD YOUR CODE IN THIS STAGE
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓

# Example
RUN <<EOF
    {
        echo "..........................................." && \
        echo "Sanity check" && \
        python -c "import torch" && \
        python -c "import torchvision" && \
        python -c "import hydra" && \
        python -c "from omegaconf import DictConfig, OmegaConf" && \
        echo "..........................................." ;
    } || exit 1
EOF

# ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑
# ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑
# USER NOTES: ADD YOUR CODE BEFORE THIS LINE

# ====DN-project final=============================================================================
FROM --platform=${TARGETPLATFORM} user-project-custom-steps AS final
# ⚠️ USER NOTES: Dont change code in this stage unless you know what your doing.
ARG TARGETPLATFORM
ARG BUILDPLATFORM
WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}

RUN <<EOF
    source /dna-lib-container-tools/dn_project_core.build.aarch_aware_build_ros.bash ${TARGETPLATFORM} ${BUILDPLATFORM} ${DN_DEV_WORKSPACE:?err}/src || exit 1
    # Cleanup buidl script
    rm -f /dn_project_core_init.bash
    rm -f /dna-lib-container-tools/dn_project_core.setup.bash
    rm -f /dna-lib-container-tools/dn_project_core.build.aarch_aware_build_ros.bash
EOF
CMD [ "bash" ]








