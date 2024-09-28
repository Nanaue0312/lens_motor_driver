cmake_minimum_required(VERSION 3.5)
project(utools_lib)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ring_buffer
    ${CMAKE_CURRENT_SOURCE_DIR}/src/umempool
    ${CMAKE_CURRENT_SOURCE_DIR}/src/coded
    ${CMAKE_CURRENT_SOURCE_DIR}/src/logger
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utime
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ufmt
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collab
    ${CMAKE_CURRENT_SOURCE_DIR}/src/umath
    ${CMAKE_CURRENT_SOURCE_DIR}/src/fliter
    ${CMAKE_CURRENT_SOURCE_DIR}/src/uthread_pool
    ${CMAKE_CURRENT_SOURCE_DIR}/src/about_mang
    ${CMAKE_CURRENT_SOURCE_DIR}/src/any_store
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pattern
)

# 查找所有源文件
file(GLOB_RECURSE SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(utools_lib STATIC ${SOURCE_FILES})
