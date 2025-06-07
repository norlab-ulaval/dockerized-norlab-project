#!/bin/bash
# lib/commands/build.bash

DOCUMENTATION_BUFFER_BUILD=$( cat <<'EOF'
# =================================================================================================
# Build Docker images for the project
# Notes: build all services for host native architecture by default
#
# Usage:
#   $ dnp build [OPTIONS] [--] [--help|[<specialized-option>] [--] [<any-docker-argument>]]
#
# Options:
#   --multiarch            Build all service for multiple architectures
#                           (require a configured docker buildx multiarch builder)
#   --ci-tests             Build CI tests images only
#   --deploy               Build deploy images only
#   --develop              Build develop images only
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

test -d "${DNP_ROOT:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }

function dnp::build() {
    local multiarch=false
    local ci_tests=false
    local deploy=false
    local develop=false
    local slurm=false
    local help=false
    local remaining_args=()

    # Parse options
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --multiarch)
                multiarch=true
                shift
                ;;
            --ci-tests)
                ci_tests=true
                shift
                ;;
            --deploy)
                deploy=true
                shift
                ;;
            --develop)
                develop=true
                shift
                ;;
            --slurm)
                slurm=true
                shift
                ;;
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_BUILD}"
                ;;
            --) # no more option
                shift
                remaining_args=("$@")
                break
                ;;
            *)
                remaining_args=("$@")
                break
                ;;
        esac
    done

    # Load super project configuration
    source "${DNP_LIB_PATH}/core/utils/load_super_project_config.bash" || exit 1

    # Determine which build script to execute
    if [[ "${ci_tests}" == true ]]; then
        if [[ "${multiarch}" == true ]]; then
            echo "Building CI tests images (multiarch)..."
            source "${DNP_LIB_EXEC_PATH}/build.ci_tests.multiarch.bash" "${remaining_args[@]}"
        else
            echo "Building CI tests images..."
            source "${DNP_LIB_EXEC_PATH}/build.ci_tests.bash" "${remaining_args[@]}"
        fi
    elif [[ "${slurm}" == true ]]; then
        build_slurm_flag+=("--service-names" "project-core,project-slurm,project-slurm-no-gpu")
        if [[ "${multiarch}" == true ]]; then
            echo "Building slurm images (multiarch)..."
            source "${DNP_LIB_EXEC_PATH}/build.all.multiarch.bash"
            dnp::build_dn_project_multiarch_services "${build_slurm_flag[@]}" -- "${remaining_args[@]}"
            fct_exit_code=$?
        else
            echo "Building slurm images..."
            source "${DNP_LIB_EXEC_PATH}/build.all.bash"
            dnp::build_dn_project_services  "${build_slurm_flag[@]}" -- "${remaining_args[@]}"
            fct_exit_code=$?
        fi
    elif [[ "${deploy}" == true ]]; then
        echo "Building deploy images..."
        source "${DNP_LIB_EXEC_PATH}/build.deploy.bash" "${remaining_args[@]}"
    elif [[ "${develop}" == true ]]; then
        echo "Building develop images..."
#        source "${DNP_LIB_EXEC_PATH}/build.develop.bash" "${remaining_args[@]}"
        add_docker_flag=("--service-names" "project-core,project-develop")
        source "${DNP_LIB_EXEC_PATH}/build.all.bash"
        dnp::build_dn_project_services "${add_docker_flag[@]}" "${remaining_args[@]}"
        fct_exit_code=$?
    else
        if [[ "${multiarch}" == true ]]; then
            echo "Building all images (multiarch)..."
            source "${DNP_LIB_EXEC_PATH}/build.all.multiarch.bash"
            dnp::build_dn_project_multiarch_services "${remaining_args[@]}"
            fct_exit_code=$?
        else
            echo "Building all images..."
            source "${DNP_LIB_EXEC_PATH}/build.all.bash"
            dnp::build_dn_project_services "${remaining_args[@]}"
            fct_exit_code=$?
        fi
    fi

    return 0
}

