#!/bin/bash
# =================================================================================================
# Console related functions
#
# Note: Requires N2ST library be loaded
#
# Globals:
#   read N2ST
# =================================================================================================

# ....Pre-condition................................................................................
# Check if N2ST is loaded
dna_error_prefix="\033[1;31m[DNA error]\033[0m"
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

# ....Load DNA utils.............................................................................
set -o allexport
source "${DNA_LIB_PATH:?err}/core/utils/.env.cli_format_and_style" || exit 1
set +o allexport


function dna:help_header() {
#    echo -n -e "${MSG_DIMMED_FORMAT}"
#    echo -e "▶︎"
    n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_HELP:?err}" "${MSG_LINE_STYLE_LVL2:?err}"
#    n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_HELP:?err}" | sed 's/───/\/──/'
#    n2st::echo_centering_str "❄${MSG_LINE_CENTER_CHAR_HELP:?err}" "${MSG_LINE_STYLE_LVL2:?err}" "${MSG_LINE_CHAR_HELP:?err}"
#    echo -n -e "${MSG_END_FORMAT}"
}

function dna::help_footer() {
#    echo -n -e "${MSG_LINE_STYLE_LVL2:?err}"
#    n2st::draw_horizontal_line_across_the_terminal_window "${MSG_LINE_CHAR_HELP:?err}"
#    n2st::draw_horizontal_line_across_the_terminal_window "─" | sed 's/──────$/─❄︎───/'
#    n2st::draw_horizontal_line_across_the_terminal_window " " | sed 's/      $/   ◀︎/'
#    echo -n -e "${MSG_END_FORMAT}"
    n2st::echo_centering_str "${MSG_LINE_CENTER_CHAR_HELP:?err}" "${MSG_LINE_STYLE_LVL2:?err}" "${MSG_LINE_CHAR_HELP:?err}"
}

function dna::documentation_buffer_to_help_parser() {
    local documentation_buffer=$*
    echo -e "${documentation_buffer}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//'
}

function dna::command_help_menu() {
    local documentation_buffer=$*
    dna:help_header
    dna::documentation_buffer_to_help_parser "${documentation_buffer}"
    dna::help_footer
    return 0
}

# ====Warning/error msg============================================================================
function dna::unknown_option_msg() {
    local the_name=$1
    local the_option=$2
    n2st::print_msg "Unknown option ${MSG_DIMMED_FORMAT}${the_option}${MSG_END_FORMAT} ‼️\n\nRun ${MSG_DIMMED_FORMAT}\$ ${the_name} --help${MSG_END_FORMAT} for usage information"
    return 1
}

function dna::unknown_command_msg() {
    local the_command=$1
    n2st::print_msg "Unknown command ${MSG_DIMMED_FORMAT}dna ${the_command}${MSG_END_FORMAT} ‼️\n\nRun ${MSG_DIMMED_FORMAT}dna --help${MSG_END_FORMAT} for usage information"
    return 1
}

function dna::unknown_subcommand_msg() {
    local the_command=$1
    local sub_the_command=${2:-""}
    n2st::print_msg "Unknown command ${MSG_DIMMED_FORMAT}dna ${the_command} ${sub_the_command}${MSG_END_FORMAT} ‼️\n\nRun ${MSG_DIMMED_FORMAT}dna ${the_command} --help${MSG_END_FORMAT} for usage information"
    return 1
}

function dna::illegal_command_msg() {
    # Usage:
    #   local original_command="$*"
    #   dna::illegal_command_msg "build" "${original_command}" "Blabla blabla bla.\n"
    local the_command=$1
    local the_original_command=${2:-""}
    local message=${3:-""}
    n2st::print_msg "Illegal command ${MSG_DIMMED_FORMAT}${the_command} ${the_original_command}${MSG_END_FORMAT} ‼️\n${message}\nRun ${MSG_DIMMED_FORMAT}dna ${the_command} --help${MSG_END_FORMAT} for usage information"
    return 1
}

function n2st::print_msg_error() {
  # This is an override version of the original one but with no newline before and after
  local error_msg=$1
  echo -e "${MSG_ERROR}: ${error_msg}" 1>&2
}

# ====Entrypoint splash============================================================================
function dna::show_entrypoint_help() {
    local documentation_buffer_dna="${1:?err}"
    # Splash type: small, negative or big
    n2st::norlab_splash "${DNA_SPLASH_NAME_FULL:?err}" "${DNA_GIT_REMOTE_URL}" 'negative'
    n2st::echo_centering_str 'A tool for managing Docker-based robotic projects' "\033[1;37m" " "

    # Note:
    #   - Strip shell comment bloc comment character `#` of both empty line and line with text,
    #   - Delete both horizontal lines
    #   - Delet both header ligne
    echo -e "${documentation_buffer_dna}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//' | sed '/Dockerized-NorLab project application (DNA)/d' | sed '/A tool for managing Docker-based robotic projects/d' | sed "/Run 'dna COMMAND --help' for more information on a command./d"
    echo -e "Run ${MSG_DIMMED_FORMAT}dna COMMAND --help${MSG_END_FORMAT} for more information on a command."
}

function dna::show_entrypoint_help_no_splash() {
    local documentation_buffer_dna="${1:?err}"
    # Note:
    #   - Strip shell comment bloc comment character `#` of both empty line and line with text,
    #   - Delete both horizontal lines
    #   - Reformat the header left align
    echo -e "${documentation_buffer_dna}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//' | sed 's/^[[:space:]]*Dockerized-NorLab project application (DNA)/Dockerized-NorLab project application (DNA)/' | sed 's/^[[:space:]]*A tool for managing Docker-based robotic projects/A tool for managing Docker-based robotic projects/'
}
