cmake_minimum_required(VERSION 2.8)
project(facom)

# Find packages

# Project source code
set(SOURCES
    src/facom.c
    src/private.c
)

# Configuration
include_directories(include/)

# Compiling
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Linking
target_link_libraries(${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/facom)

