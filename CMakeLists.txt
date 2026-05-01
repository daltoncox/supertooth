cmake_minimum_required(VERSION 3.10)

project(supertooth C)

# Main source directory
add_subdirectory(src)

# Examples directory
add_subdirectory(examples EXCLUDE_FROM_ALL)