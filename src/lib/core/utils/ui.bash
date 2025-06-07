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

# ====UI utilities=================================================================================
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
    exit 0
}

function dnp::unknown_option_msg() {
    local the_script_name=$1
    local the_option=$2
    n2st::print_msg "Unknown command ${MSG_DIMMED_FORMAT}${the_option}${MSG_END_FORMAT} ‼️\n\nRun ${MSG_DIMMED_FORMAT}\$ ${the_script_name} --help${MSG_END_FORMAT} for usage information"
    exit 1
}

function dnp::unknown_command_msg() {
    local the_command=$1
    n2st::print_msg "Unknown command ${MSG_DIMMED_FORMAT}dnp ${the_command}${MSG_END_FORMAT} ‼️\n\nRun ${MSG_DIMMED_FORMAT}dnp --help${MSG_END_FORMAT} for usage information"
    exit 1
}

function dnp::unknown_subcommand_msg() {
    local the_command=$1
    local sub_the_command=${2:-""}
    n2st::print_msg "Unknown command ${MSG_DIMMED_FORMAT}dnp ${the_command} ${sub_the_command}${MSG_END_FORMAT} ‼️\n\nRun ${MSG_DIMMED_FORMAT}dnp ${the_command} --help${MSG_END_FORMAT} for usage information"
    exit 1
}

function n2st::print_msg_error() {
  # This is an override version of the original one but with no newline before and after
  local error_msg=$1
  echo -e "${MSG_ERROR}: ${error_msg}" 1>&2
}
