#!/bin/bash
# =================================================================================================
# Console related functions
#
# Note: Requires N2ST library be loaded
#
# Globals:
#   read N2ST
# =================================================================================================
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"

# ....Pre-condition................................................................................
# Check if N2ST is loaded
n2st::print_msg "test" 2>/dev/null >/dev/null || { echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} The N2ST lib is not loaded!" ; exit 1 ; }

# ====Help/doc=====================================================================================
function dnp:help_header() {
    echo -n -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "." #| sed 's/.../\/../'
#    n2st::echo_centering_str "❄︎" "${MSG_DIMMED_FORMAT}" "."
#    n2st::echo_centering_str "▼" "${MSG_DIMMED_FORMAT}" "."
    echo -n -e "${MSG_END_FORMAT}"
}

function dnp::help_footer() {
    echo -n -e "${MSG_DIMMED_FORMAT}"
#    n2st::draw_horizontal_line_across_the_terminal_window "." #| sed 's/..........$/.....❄︎.../'
    n2st::echo_centering_str "❄︎" "${MSG_DIMMED_FORMAT}" "."
#    n2st::echo_centering_str "▲" "${MSG_DIMMED_FORMAT}" "."
    echo -n -e "${MSG_END_FORMAT}"
}

function dnp::documentation_buffer_to_help_parser() {
    local documentation_buffer=$*
    echo -e "${documentation_buffer}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//'
}

function dnp::command_help_menu() {
    local documentation_buffer=$*
    dnp:help_header
    dnp::documentation_buffer_to_help_parser "${documentation_buffer}"
    dnp::help_footer
    return 0
}

# ====Warning/error msg============================================================================
function dnp::unknown_option_msg() {
    local the_name=$1
    local the_option=$2
    n2st::print_msg "Unknown option ${MSG_DIMMED_FORMAT}${the_option}${MSG_END_FORMAT} ‼️\n\nRun ${MSG_DIMMED_FORMAT}\$ ${the_name} --help${MSG_END_FORMAT} for usage information"
    return 1
}

function dnp::unknown_command_msg() {
    local the_command=$1
    n2st::print_msg "Unknown command ${MSG_DIMMED_FORMAT}dnp ${the_command}${MSG_END_FORMAT} ‼️\n\nRun ${MSG_DIMMED_FORMAT}dnp --help${MSG_END_FORMAT} for usage information"
    return 1
}

function dnp::unknown_subcommand_msg() {
    local the_command=$1
    local sub_the_command=${2:-""}
    n2st::print_msg "Unknown command ${MSG_DIMMED_FORMAT}dnp ${the_command} ${sub_the_command}${MSG_END_FORMAT} ‼️\n\nRun ${MSG_DIMMED_FORMAT}dnp ${the_command} --help${MSG_END_FORMAT} for usage information"
    return 1
}

function dnp::illegal_command_msg() {
    # Usage:
    #   local original_command="$*"
    #   dnp::illegal_command_msg "build" "${original_command}" "Blabla blabla bla.\n"
    local the_command=$1
    local the_original_command=${2:-""}
    local message=${3:-""}
    n2st::print_msg "Illegal command ${MSG_DIMMED_FORMAT}${the_command} ${the_original_command}${MSG_END_FORMAT} ‼️\n${message}\nRun ${MSG_DIMMED_FORMAT}dnp ${the_command} --help${MSG_END_FORMAT} for usage information"
    return 1
}

function n2st::print_msg_error() {
  # This is an override version of the original one but with no newline before and after
  local error_msg=$1
  echo -e "${MSG_ERROR}: ${error_msg}" 1>&2
}

# ====Entrypoint splash============================================================================
function dnp::show_entrypoint_help() {
    # Splash type: small, negative or big
    n2st::norlab_splash 'Dockerized-NorLab-Project' 'https://github.com/norlab-ulaval/dockerized-norlab-project.git' 'negative'
    n2st::echo_centering_str 'A tool for managing Docker-based robotic projects' "\033[1;37m" " "

    # Note:
    #   - Strip shell comment bloc comment character `#` of both empty line and line with text,
    #   - Delete both horizontal lines
    #   - Delet both header ligne
    echo -e "${DOCUMENTATION_BUFFER_DNP}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//' | sed '/Dockerized-NorLab-Project (DNP)/d' | sed '/A tool for managing Docker-based robotic projects/d' | sed "/Run 'dnp COMMAND --help' for more information on a command./d"
    echo -e "Run ${MSG_DIMMED_FORMAT}dnp COMMAND --help${MSG_END_FORMAT} for more information on a command."
}

function dnp::show_entrypoint_help_no_splash() {
    # Note:
    #   - Strip shell comment bloc comment character `#` of both empty line and line with text,
    #   - Delete both horizontal lines
    #   - Reformat the header left align
    echo -e "${DOCUMENTATION_BUFFER_DNP}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//' | sed 's/^[[:space:]]*Dockerized-NorLab-Project (DNP)/Dockerized-NorLab-Project (DNP)/' | sed 's/^[[:space:]]*A tool for managing Docker-based robotic projects/A tool for managing Docker-based robotic projects/'
}
