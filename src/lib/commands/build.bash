#!/bin/bash
# lib/commands/build.bash

DOCUMENTATION_BUFFER_BUILD=$( cat <<'EOF'
# =================================================================================================
# Build project Docker images
#
# Usage:
#   $ dna build [OPTIONS] [SERVICE] [-- <any-docker-argument>]
#
# Options:
#   --multiarch                   Build services for multiple architectures
#   --rmab                        Re-create a local docker buildx multiarch builder
#   --online-build                Build image sequentialy by pushing/pulling intermediate images
#                                  from Dockerhub (requires Docker Hub authentication)
#   --save DIRPATH                Save built image to directory (develop, deploy or slurm services only)
#   --push                        Push image to Dockerhub (deploy or slurm services only,
#                                  requires Docker Hub authentication)
#   --apptainer <profile>         HPC Apptainer workflow for slurm service only.
#                                  <profile> selects .env.<profile> server configuration.
#                                  Must be combined with --save or --push:
#                                    --apptainer <profile> --save  : tar archive pipeline
#                                      Saves slurm image as linux/amd64 .tar archive and generates
#                                      dna_tar_to_apptainer_sif_converter.sh helper script for HPC.
#                                    --apptainer <profile> --push  : registry push pipeline
#                                      Pushes slurm image to Docker registry and generates
#                                      dna_registry_to_apptainer_sif_converter.sh helper script for HPC.
#                                  e.g., dna build slurm --apptainer valeria --save
#                                        dna build slurm --apptainer valeria --push
#                                  Note: apptainer is NOT executed locally (macOS compatible)
#   --squash                      Squash the built image to reduce its size.
#                                  For slurm (with or without --apptainer): squashes the slurm image
#                                  before saving the tar archive (or in-place without --apptainer).
#                                  For deploy/ci-tests: squashes the image in-place after building.
#                                  Collapses all image layers into one.
#                                  Preserves ENV, ENTRYPOINT/CMD, WORKDIR, LABEL, USER.
#                                  Removes intermediate layer history. Requires python3 on host.
#   --gs-only                     (Apptainer-only flag) Generate script only. Skip all docker
#                                  build/push/save steps and re-generate only the HPC converter
#                                  script. Must be used together with --apptainer <profile>
#                                  and either --save or --push. Useful to update
#                                  dna_tar_to_apptainer_sif_converter.sh (with --save) or
#                                  dna_registry_to_apptainer_sif_converter.sh (with --push)
#                                  without re-building the Docker image. Does not require internet.
#                                  e.g., dna build slurm --apptainer valeria --save --gs-only
#                                        dna build slurm --apptainer valeria --push --gs-only
#   --help, -h                    Show this help message
#
#
# SERVICE:
#   develop                       Build develop images only
#   deploy                        Build deploy images only
#   ci-tests                      Build CI tests images only
#   slurm                         Build slurm images only
#   release                       Build release images only
#   core                          Build core images only
#
# Notes:
#   - build all services for host native architecture by default
#   - build offline from the local image store by default
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }
test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

# =================================================================================================
# Check if user is logged into Docker Hub
#
# Usage:
#   $ dna::check_user_is_login_dockerhub
#
# Returns:
#   0 if user is logged in to Docker Hub
#   1 if user is not logged in to Docker Hub
# =================================================================================================
function dna::check_user_is_login_dockerhub() {
    # Check if docker is available
    if ! command -v docker &> /dev/null; then
        n2st::print_msg_error "Docker is not installed or not available in PATH"
        return 1
    fi

    # Try to get authentication info from docker config
    # This checks if there are any stored credentials for Docker Hub
    local docker_config_path="${HOME}/.docker/config.json"

    if [[ -f "${docker_config_path}" ]]; then
        # Check if there are auths for docker.io or index.docker.io (Docker Hub)
        if grep -q '"https://index.docker.io/v1/"' "${docker_config_path}" 2>/dev/null || \
           grep -q '"docker.io"' "${docker_config_path}" 2>/dev/null; then
            return 0
        fi
    fi

    # Alternative check: try to access Docker Hub API with stored credentials
    # This will fail silently if not logged in
    if docker system info 2>/dev/null | grep -q "Registry:" 2>/dev/null; then
        # Try a simple operation that requires authentication
        if docker search --limit 1 hello-world &>/dev/null; then
            return 0
        fi
    fi

    return 1
}

function dna::build_command() {

    # Pre-scan for --gs-only: it is a local-only operation that does not require internet
    local _gs_only_prescan=false
    for _arg in "$@"; do
      [[ "${_arg}" == "--gs-only" ]] && _gs_only_prescan=true && break
    done

    if [[ "${_gs_only_prescan}" == false ]] && ! dna::is_online; then
      n2st::print_msg_error "Be advised, you are currently offline. Executing ${MSG_DIMMED_FORMAT}dna build${MSG_END_FORMAT} require internet connection."
      return 1
    fi

    # ....Set env variables (pre cli)).............................................................
    local multiarch=false
    local re_create_multiarch_builder=false
    local force_push_project_core=false
    local service=""
    local push_deploy=false
    local save_dirpath=""
    local apptainer_profile=""
    local apptainer_pipeline=""  # "save" or "push" — required when --apptainer is set
    local squash_image=false
    local gs_only=false
    local remaining_args=()
    local original_command="$*"
    local line_format="${MSG_LINE_CHAR_BUILDER_LVL1}"
    local line_style="${MSG_LINE_STYLE_LVL2}"


    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --multiarch)
                multiarch=true
                shift
                ;;
            --rmab)
                re_create_multiarch_builder=true
                shift
                ;;
            --online-build)
                force_push_project_core=true
                shift
                ;;
            --push)
                push_deploy=true
                # When --apptainer is already set, --push selects the registry pipeline
                if [[ -n "${apptainer_profile}" ]]; then
                    apptainer_pipeline="push"
                fi
                shift
                ;;
            --save)
                # When --apptainer is already set, --save (no DIRPATH) selects the tar pipeline
                if [[ -n "${apptainer_profile}" ]]; then
                    apptainer_pipeline="save"
                    shift
                else
                    if [[ -z "$2" ]]; then
                        dna::illegal_command_msg "build" "${original_command}" "The --save flag requires a DIRPATH argument (or use with --apptainer for slurm tar pipeline).\n"
                        return 1
                    fi
                    save_dirpath="$2"
                    shift 2
                fi
                ;;
            --apptainer)
                if [[ -z "$2" ]]; then
                    dna::illegal_command_msg "build" "${original_command}" "The --apptainer flag requires a <profile> argument (e.g., valeria, compute_canada, mamba).\n"
                    return 1
                fi
                apptainer_profile="$2"
                shift 2
                ;;
            --squash)
                squash_image=true
                shift
                ;;
            --gs-only)
                gs_only=true
                shift
                ;;
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_BUILD:?err}"
                exit 0
                ;;
            --) # no more option
                shift
                remaining_args+=("$@")
                break
                ;;
            core|develop|deploy|ci-tests|slurm|release)
                # If service is already set, it's an error
                if [[ -n "${service}" ]]; then
                    dna::illegal_command_msg "build" "${original_command}" "Only one SERVICE can be specified.\n"
                    return 1
                fi
                service="$1"
                if [[ "${service}" == "release" ]]; then
                    n2st::print_msg "Command ${MSG_DIMMED_FORMAT}dna build release${MSG_END_FORMAT} is not released yet, stay tuned!\n ... yeah I know, an un-released release function" && exit 0 # (CRITICAL) ToDo: on task end >> delete this line <--
                    return 1
                fi
                shift
                ;;
            *)
                # Check if it starts with -- (unknown option)
                if [[ "$1" == --* ]]; then
                    dna::unknown_subcommand_msg "build" "$*"
                    exit 1
                fi
                # Otherwise it's an unknown service
                dna::illegal_command_msg "build" "${original_command}" "Unknown SERVICE: $1. Valid services are: core, deploy, develop, ci-tests, slurm, release.\n"
                return 1
                ;;
        esac
    done

    # ....Set env variables (post cli)...............................................................
    declare -a build_flag=()
    declare -a deploy_flag=()

    build_flag+=("--msg-line-level" "${line_format}")

    local architecture="native"
    if [[ "${multiarch}" == true ]]; then
      architecture="multiarch"
      # ToDo: on task (NMO-767) end >> delete next bloc ↓↓
      if [[ "${force_push_project_core}" == false  ]]; then
        if [[ $(docker info -f '{{ .DriverStatus }}') =~ .*"driver-type io.containerd.snapshotter".* ]]; then
          n2st::print_msg "Containerd snapshotter is enabled, Build multi-architecture localy."
        else
          n2st::print_msg_warning "Offline multi-architecture build requires that Docker containerd snapshotter be enabled.
Either run ${MSG_DIMMED_FORMAT}dna build${MSG_END_FORMAT} in online build mode i.e., ${MSG_DIMMED_FORMAT}$ dna build --multiarch --online-build [SERVICE]${MSG_END_FORMAT}
or enable containerd snapshotter local image store:
  - For Docker Desktop user: go to 'Setting / General'  and check the 'Use containerd for pulling and storing images' check box.
  - For non Docker Desktop user: add the following to your ${MSG_DIMMED_FORMAT}/etc/docker/daemon.json${MSG_END_FORMAT} configuration file
${MSG_DIMMED_FORMAT}
      {
          \"features\": {
            \"containerd-snapshotter\": true
          }
      }
${MSG_END_FORMAT}
    and restart the docker daemon: ${MSG_DIMMED_FORMAT}$ sudo systemctl restart docker${MSG_END_FORMAT}
"
          echo
          return 0
        fi
      fi
    fi

    if [[ "${force_push_project_core}" == true ]]; then
      build_flag+=("--force-push-project-core")
    fi

    if [[ "${service}" == "deploy" ]]; then
        # Case: Deploy
        header_footer_name="deploy images ${architecture} build procedure"
        if [[ "${multiarch}" == true ]]; then
          deploy_flag+=("--multiarch")
        fi
        if [[ "${push_deploy}" == true ]]; then
          deploy_flag+=("--push")
        fi
    else
      # Case: general
      if [[ "${service}" == "ci-tests" ]]; then
          header_footer_name="CI tests images ${architecture} build procedure"
          build_flag+=("--service-names" "project-core-pre,project-core-user,project-core,project-ci-tests")
      elif [[ "${service}" == "slurm" ]]; then
          header_footer_name="slurm images ${architecture} build procedure"
          build_flag+=("--service-names" "project-core-pre,project-core-user,project-core,project-slurm")
      elif [[ "${service}" == "develop" ]]; then
          header_footer_name="develop images ${architecture} build procedure"
          build_flag+=("--service-names" "project-core-pre,project-core-user,project-core,project-develop")
      elif [[ "${service}" == "core" ]]; then
          header_footer_name="core images ${architecture} build procedure"
          build_flag+=("--service-names" "project-core-pre,project-core-user,project-core")
      else
          header_footer_name="all images ${architecture} build procedure"
      fi
    fi

    # Splash type: small, negative or big
    n2st::norlab_splash "${DNA_SPLASH_NAME_SMALL}" "${DNA_GIT_REMOTE_URL}" "small"
    n2st::print_formated_script_header "${header_footer_name}" "${line_format}" "${line_style}"

    # ....Post-CLI flag resolution.................................................................
    # Handle the case where --push was parsed before --apptainer (arg order independent)
    if [[ -n "${apptainer_profile}" && "${push_deploy}" == true && -z "${apptainer_pipeline}" ]]; then
        apptainer_pipeline="push"
    fi

    # ....Flag check...............................................................................
    if [[ "${service}" != "deploy" ]] && [[ "${push_deploy}" == true ]] && [[ -z "${apptainer_profile}" ]]; then
      dna::illegal_command_msg "build" "${original_command}" "The ${MSG_DIMMED_FORMAT}--push${MSG_END_FORMAT} flag can only be used with SERVICE=deploy (or with --apptainer <profile> for the registry pipeline).\n"
      return 1
    fi

    if [[ -n "${save_dirpath}" ]]; then
      if [[ "${service}" != "develop" && "${service}" != "deploy" ]]; then
        dna::illegal_command_msg "build" "${original_command}" "The ${MSG_DIMMED_FORMAT}--save${MSG_END_FORMAT} flag can only be used with SERVICE=develop or SERVICE=deploy.\n"
        return 1
      fi
      if [[ ! -d "${save_dirpath}" ]]; then
        n2st::print_msg_error "The DIRPATH does not exist: ${MSG_DIMMED_FORMAT}${save_dirpath}${MSG_END_FORMAT}\n"
        return 1
      fi
    fi

    if [[ -n "${apptainer_profile}" ]]; then
      if [[ "${service}" != "slurm" ]]; then
        dna::illegal_command_msg "build" "${original_command}" "The ${MSG_DIMMED_FORMAT}--apptainer${MSG_END_FORMAT} flag can only be used with SERVICE=slurm.\n"
        return 1
      fi
      if [[ -z "${apptainer_pipeline}" ]]; then
        dna::illegal_command_msg "build" "${original_command}" "The ${MSG_DIMMED_FORMAT}--apptainer${MSG_END_FORMAT} flag requires either ${MSG_DIMMED_FORMAT}--save${MSG_END_FORMAT} (tar archive pipeline) or ${MSG_DIMMED_FORMAT}--push${MSG_END_FORMAT} (registry pipeline).
  e.g.: dna build slurm --apptainer ${apptainer_profile} --save
        dna build slurm --apptainer ${apptainer_profile} --push\n"
        return 1
      fi
    fi

    if [[ "${squash_image}" == true ]]; then
      if [[ -z "${apptainer_profile}" && "${service}" != "deploy" && "${service}" != "ci-tests" && "${service}" != "slurm" ]]; then
        dna::illegal_command_msg "build" "${original_command}" "The ${MSG_DIMMED_FORMAT}--squash${MSG_END_FORMAT} flag requires SERVICE=slurm, deploy, or ci-tests (or use --apptainer <profile> with slurm).\n"
        return 1
      fi
    fi

    if [[ "${gs_only}" == true ]]; then
      if [[ -z "${apptainer_profile}" || -z "${apptainer_pipeline}" ]]; then
        dna::illegal_command_msg "build" "${original_command}" "The ${MSG_DIMMED_FORMAT}--gs-only${MSG_END_FORMAT} flag requires ${MSG_DIMMED_FORMAT}--apptainer <profile> --save${MSG_END_FORMAT} or ${MSG_DIMMED_FORMAT}--apptainer <profile> --push${MSG_END_FORMAT}.\n  e.g.: dna build slurm --apptainer ${apptainer_profile:-valeria} --save --gs-only\n"
        return 1
      fi
    fi

    # ....Load dependencies........................................................................
    source "${DNA_LIB_PATH}/core/utils/load_super_project_config.bash" || return 1
    source "${DNA_LIB_EXEC_PATH}/build.all.bash" || return 1
    source "${DNA_LIB_EXEC_PATH}/build.all.multiarch.bash" || return 1
    source "${DNA_LIB_EXEC_PATH}/build.deploy.bash" || return 1
    if [[ -n "${apptainer_profile}" ]]; then
      source "${DNA_LIB_PATH}/core/utils/apptainer_tools.bash" || return 1
      # Override DN_PROJECT_USER with the HPC server username from the profile env file
      dna::load_apptainer_profile_env "${apptainer_profile}" || return 1
      # Enforce target platform for cross-architecture build (e.g., arm64 Mac → amd64 HPC)
      export DOCKER_DEFAULT_PLATFORM="${APPTAINER_TARGET_PLATFORM:-linux/amd64}"
      n2st::print_msg "Enforcing build platform: ${DOCKER_DEFAULT_PLATFORM} (from profile: ${apptainer_profile})"
    elif [[ "${squash_image}" == true ]]; then
      source "${DNA_LIB_PATH}/core/utils/apptainer_tools.bash" || return 1
    fi

    # When --gs-only is set, skip all docker build/push/save steps and jump directly to script generation
    if [[ "${gs_only}" == true ]]; then
      n2st::print_msg "--gs-only flag set: skipping docker build/push/save, regenerating HPC converter script only"
      local apptainer_save_dir="${SUPER_PROJECT_ROOT:?err}/artifact/apptainer"
      mkdir -p "${apptainer_save_dir}" || return 1
      local sif_name_gs="${DN_PROJECT_IMAGE_NAME:?err}-slurm.sif"
      local image_name_gs="${DN_PROJECT_HUB:?err}/${DN_PROJECT_IMAGE_NAME}-slurm:${PROJECT_TAG:?err}"

      dna::check_apptainer_profile_env_file "${apptainer_profile}" || return 1

      if [[ "${apptainer_pipeline}" == "save" ]]; then
        local tar_filename_gs="${DN_PROJECT_IMAGE_NAME}-slurm.${PROJECT_TAG}.tar"
        dna::generate_apptainer_build_sif_script \
            "${tar_filename_gs}" \
            "${sif_name_gs}" \
            "${apptainer_save_dir}" \
            "${apptainer_profile}" || return 1
        n2st::print_msg_done "dna_tar_to_apptainer_sif_converter.sh regenerated in: ${apptainer_save_dir}"
      elif [[ "${apptainer_pipeline}" == "push" ]]; then
        dna::generate_registry_to_apptainer_sif_script \
            "${image_name_gs}" \
            "${sif_name_gs}" \
            "${apptainer_save_dir}" \
            "${apptainer_profile}" || return 1
        n2st::print_msg_done "dna_registry_to_apptainer_sif_converter.sh regenerated in: ${apptainer_save_dir}"
      fi

      n2st::print_formated_script_footer "${header_footer_name}" "${line_format}" "${line_style}"
      return 0
    fi

    # ....Docker Hub login check..................................................................
    # Check if Docker Hub login is required and user is logged in
    local dockerhub_login_required=false

    # Check if --online-build flag is used (requires Docker Hub access for pushing/pulling)
    local login_hub_check_flag
    if [[ "${force_push_project_core}" == true ]]; then
        dockerhub_login_required=true
        login_hub_check_flag="--online-build"
        n2st::print_msg "Online build mode detected (${login_hub_check_flag} flag)"
    fi

    # Check if deploy service with --push flag is used (requires Docker Hub access for pushing)
    if [[ "${service}" == "deploy" && "${push_deploy}" == true ]]; then
        dockerhub_login_required=true
        login_hub_check_flag="deploy --push"
        n2st::print_msg "Deploy push mode detected (${login_hub_check_flag} flag)"
    fi

    # Check if slurm with --apptainer --push is used (requires Docker Hub access for pushing)
    if [[ "${service}" == "slurm" && "${apptainer_pipeline}" == "push" ]]; then
        dockerhub_login_required=true
        login_hub_check_flag="slurm --apptainer --push"
        n2st::print_msg "Slurm apptainer registry push pipeline detected (${login_hub_check_flag} flag)"
    fi

    # Perform Docker Hub login check if required
    if [[ "${dockerhub_login_required}" == true ]]; then
        n2st::print_msg "Checking Docker Hub authentication..."
        if ! dna::check_user_is_login_dockerhub; then
            n2st::print_msg_error "Build flag ${MSG_DIMMED_FORMAT}${login_hub_check_flag}${MSG_END_FORMAT} require Docker Hub authentication but user is not logged in!"
            echo -e "Please run ${MSG_DIMMED_FORMAT}docker login${MSG_END_FORMAT} to authenticate with Docker Hub before using this command."
            return 1
        fi
        n2st::print_msg_done "Docker Hub authentication verified"
    fi

    # ....Begin....................................................................................
    if [[ "${multiarch}" == true ]] && [[ "${re_create_multiarch_builder}" == true ]]; then
        local builder_name='local-builder-multiarch-virtual'
        n2st::print_msg "Re-create buildx builder ${MSG_DIMMED_FORMAT}${builder_name}${MSG_END_FORMAT}...\n"
        local dimmed_style
        local resset_style
        dimmed_style=$(tput dim)
        resset_style=$(tput sgr0)
        
        # Create a temporary file to capture the exit code
        local buildx_status_file=$(mktemp)
        {
          n2st::draw_horizontal_line_across_the_terminal_window "."
          if source "${DNA_ROOT:?err}/src/lib/core/utils/buildx_builder.bash" "${builder_name}"; then
            echo "0" > "$buildx_status_file"
          else
            echo "1" > "$buildx_status_file"
          fi
          n2st::draw_horizontal_line_across_the_terminal_window "."
          echo
        } | sed "s/.*/${dimmed_style}&${resset_style}/"
        
        # Check the status and handle accordingly
        if [[ "$(cat "$buildx_status_file")" != "0" ]]; then
            rm -f "$buildx_status_file"
            n2st::print_msg_error "Failed to re-create docker buildx builder ${builder_name}!"
            return 1
        fi
        rm -f "$buildx_status_file"
        
        n2st::print_msg_done "New builder ${MSG_DIMMED_FORMAT}${builder_name}${MSG_END_FORMAT} created successfully."
    fi


    if [[ "${service}" == "deploy" ]]; then
        dna::build_project_deploy_service "${deploy_flag[@]}" "${build_flag[@]}" "${remaining_args[@]}"
        fct_exit_code=$?
    else
      if [[ "${multiarch}" == true ]]; then
          dna::build_services_multiarch "${build_flag[@]}" "${remaining_args[@]}"
          fct_exit_code=$?
      else
          dna::build_services "${build_flag[@]}" "${remaining_args[@]}"
          fct_exit_code=$?
      fi
    fi

    # ....Post-build squash if requested (slurm/deploy/ci-tests without --apptainer)...........
    if [[ "${squash_image}" == true && -z "${apptainer_profile}" && $fct_exit_code -eq 0 ]]; then
        local squash_image_name="${DN_PROJECT_HUB:?err}/${DN_PROJECT_IMAGE_NAME:?err}-${service}:${PROJECT_TAG:?err}"
        n2st::print_msg "Squashing ${service} image: ${squash_image_name}"
        dna::squash_docker_image "${squash_image_name}" || {
            n2st::print_msg_error "Failed to squash Docker image"
            return 1
        }
        n2st::print_msg_done "Image squashed successfully: ${squash_image_name}"
    fi

    # ....Post-build save if requested.............................................................
    if [[ -n "${save_dirpath}" && $fct_exit_code -eq 0 ]]; then
        n2st::print_msg "Executing save command as requested"
        source "${DNA_LIB_PATH}/commands/save.bash" || {
            n2st::print_msg_error "Failed to load save command"
            return 1
        }
        dna::save_command "${save_dirpath}" "${service}" || {
            n2st::print_msg_error "Failed to save image"
            return 1
        }
    fi

    # ....Post-build apptainer artifacts if requested..............................................
    if [[ -n "${apptainer_profile}" && $fct_exit_code -eq 0 ]]; then
        n2st::print_msg "Generating Apptainer artifacts for profile: ${apptainer_profile} (pipeline: ${apptainer_pipeline})"
        dna::check_apptainer_profile_env_file "${apptainer_profile}" || return 1

        local apptainer_save_dir="${SUPER_PROJECT_ROOT:?err}/artifact/apptainer"
        mkdir -p "${apptainer_save_dir}" || {
            n2st::print_msg_error "Failed to create apptainer artifact directory: ${apptainer_save_dir}"
            return 1
        }

        local sif_name="${DN_PROJECT_IMAGE_NAME:?err}-slurm.sif"
        local image_name="${DN_PROJECT_HUB:?err}/${DN_PROJECT_IMAGE_NAME}-slurm:${PROJECT_TAG:?err}"

        # Squash image if requested (reduces size before save/push)
        if [[ "${squash_image}" == true ]]; then
            dna::squash_docker_image "${image_name}" || {
                n2st::print_msg_error "Failed to squash Docker image"
                return 1
            }
        fi

        if [[ "${apptainer_pipeline}" == "save" ]]; then
            # ....Save pipeline: save tar archive + generate dna_tar_to_apptainer_sif_converter.sh.
            local tar_filename="${DN_PROJECT_IMAGE_NAME}-slurm.${PROJECT_TAG}.tar"

            n2st::print_msg "Saving Docker image as tar archive (platform: ${APPTAINER_TARGET_PLATFORM:-linux/amd64}): ${tar_filename}"
            docker image save --platform "${APPTAINER_TARGET_PLATFORM:-linux/amd64}" --output "${apptainer_save_dir}/${tar_filename}" "${image_name}" || {
                n2st::print_msg_error "Failed to save Docker image tar archive"
                return 1
            }

            dna::generate_apptainer_build_sif_script \
                "${tar_filename}" \
                "${sif_name}" \
                "${apptainer_save_dir}" \
                "${apptainer_profile}" || {
                n2st::print_msg_error "Failed to generate dna_tar_to_apptainer_sif_converter.sh"
                return 1
            }

            local generated_script="${apptainer_save_dir}/dna_tar_to_apptainer_sif_converter.sh"

            n2st::print_msg_done "Apptainer artifacts saved to: ${apptainer_save_dir}"

            n2st::print_msg "Next steps:
  1. Transfer to HPC: artifact/apptainer/ (use your preferred method, e.g., rsync, scp, sftp)
  2. Build SIF on HPC: bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh
  3. Generate run script: dna run slurm <sjob-id> --generate-apptainer ${apptainer_profile} <python-args>"

        elif [[ "${apptainer_pipeline}" == "push" ]]; then
            # ....Push pipeline: push to Docker registry + generate dna_registry_to_apptainer_sif_converter.sh.
            n2st::print_msg "Pushing slurm image to Docker registry: ${image_name}"
            docker push "${image_name}" || {
                n2st::print_msg_error "Failed to push Docker image to registry: ${image_name}"
                return 1
            }
            n2st::print_msg_done "Slurm image pushed to registry: ${image_name}"

            dna::generate_registry_to_apptainer_sif_script \
                "${image_name}" \
                "${sif_name}" \
                "${apptainer_save_dir}" \
                "${apptainer_profile}" || {
                n2st::print_msg_error "Failed to generate dna_registry_to_apptainer_sif_converter.sh"
                return 1
            }

            local generated_script="${apptainer_save_dir}/dna_registry_to_apptainer_sif_converter.sh"

            n2st::print_msg_done "Apptainer registry converter script saved to: ${apptainer_save_dir}"

            n2st::print_msg "Next steps:
  1. Transfer script to HPC: artifact/apptainer/dna_registry_to_apptainer_sif_converter.sh
  2. Build SIF on HPC: bash artifact/apptainer/dna_registry_to_apptainer_sif_converter.sh
  3. Generate run script: dna run slurm <sjob-id> --generate-apptainer ${apptainer_profile} <python-args>"
        fi
    fi

    # ....Teardown.................................................................................
    n2st::print_formated_script_footer "${header_footer_name}" "${line_format}" "${line_style}"
    return $fct_exit_code
}
