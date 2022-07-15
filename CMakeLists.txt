cmake_minimum_required(VERSION 3.0.0)
project(rkdpl VERSION 0.1.0)

include(CTest)
enable_testing()

# set cross-platform complier interface
option(CROSS_COMPILE "build project using cross-platform complier" ON)
set(CROSS_COMPILE_PATH "/home/slg/AM437X/Toolchain/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf")
if(CROSS_COMPILE)
    message(STATUS "Building with cross-compiler")
    set(CMAKE_SYSTEM_NAME Linux)
    # specify the cross compiler
    set(CMAKE_C_COMPILER ${CROSS_COMPILE_PATH}/bin/arm-linux-gnueabihf-gcc)
    set(CMAKE_CXX_COMPILER ${CROSS_COMPILE_PATH}/bin/arm-linux-gnueabihf-g++)
    set(CMAKE_FIND_ROOT_PATH ${CROSS_COMPILE_PATH}/arm-linux-gnueabihf)
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
include(CPack)

# generate libraries and executable file
include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories("/usr/include/eigen3")

set(LIB_NAME "rtpl")
file(GLOB_RECURSE SRC_LIST ${PROJECT_SOURCE_DIR}/rtpl/*.cpp)
add_library(${LIB_NAME} ${SRC_LIST})

set(LIB_NAME "rdcl")
file(GLOB_RECURSE SRC_LIST ${PROJECT_SOURCE_DIR}/rdcl/*.cpp)
add_library(${LIB_NAME} ${SRC_LIST})

if(${CROSS_COMPILE} STREQUAL "OFF")
    set(EXE_NAME "test_rkdpl")
    file(GLOB_RECURSE TEST_LIST ${PROJECT_SOURCE_DIR}/test/*.cpp)
    link_directories(${LIBRARY_OUTPUT_PATH})
    add_executable(${EXE_NAME} ${TEST_LIST})
    add_dependencies(${EXE_NAME} rtpl rdcl)
    target_link_libraries(${EXE_NAME} rtpl rdcl)
endif()

# install libraries and header files
install(TARGETS rtpl rdcl
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib)
file(GLOB_RECURSE INCLUDE_FILE_LIST ${PROJECT_SOURCE_DIR}/inc/*.h 
                                    ${PROJECT_SOURCE_DIR}/inc/*.hpp)
install(FILES ${INCLUDE_FILE_LIST} DESTINATION include/${PROJECT_NAME})

# uninstall libraries and header files
if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
