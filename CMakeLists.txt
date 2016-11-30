project(facom)

cmake_minimum_required(VERSION 2.8)

# Find packages

# Project source code
set(SOURCES
)

# Configuration
include_directories(include/)

# Compiling
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Install
install(TARGETS ${PROJJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/)

