cmake_minimum_required(VERSION 3.18)
project(COVERAGE CXX C)

# set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

# sources
set(SOURCES
    src/mesh.cpp
    src/rply.c
    src/camera.cpp
    src/main.cpp
)

# headers
set(HEADERS
    include/rply.h
    include/rplyfile.h
    include/mesh.h
    include/camera.h
)

# create executable file
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME})
