cmake_minimum_required(VERSION 3.20)

project(student_character_plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(student_character_plugin SHARED
    src/StudentController.cpp
)

target_include_directories(student_character_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(student_character_plugin PROPERTIES
    OUTPUT_NAME "character_plugin_200201852"
    PREFIX ""
)
