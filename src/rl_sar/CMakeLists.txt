cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(USE_CMAKE ON CACHE BOOL "Use Cmake build system")
if(USE_CMAKE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    add_compile_definitions(USE_CMAKE)
endif()

option(BUILD_DIAN_ONLY "Retained for scripts/build_dian.sh; this tree only builds DIAN" ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()

get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/policy")

set(INFERENCE_RUNTIME_DIR "${PROJECT_ROOT_DIR}/library/inference_runtime")
set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")
set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")

set(IS_JETSON FALSE)
if(EXISTS "/etc/nv_tegra_release")
    set(IS_JETSON TRUE)
    message(STATUS "Jetson platform detected - ONNX Runtime will be disabled")
endif()

if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib" AND NOT IS_JETSON)
    set(USE_ONNX "ON")
    add_compile_definitions(USE_ONNX)
    message(STATUS "Found ONNX Runtime at: ${ONNX_RUNTIME_DIR}")
    if(EXISTS "${ONNX_RUNTIME_DIR}/VERSION_NUMBER")
        file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
        string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
    endif()
    include_directories("${ONNX_RUNTIME_DIR}/include")
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(LIB_EXT "so")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(LIB_EXT "dylib")
    else()
        set(LIB_EXT "lib")
    endif()
    find_library(ONNX_RUNTIME_LIB_PATH
        NAMES libonnxruntime.${LIB_EXT} onnxruntime.${LIB_EXT}
        PATHS "${ONNX_RUNTIME_DIR}/lib"
        NO_DEFAULT_PATH
    )
    if(ONNX_RUNTIME_LIB_PATH)
        add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
        set_target_properties(onnxruntime_prebuilt PROPERTIES
            IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
        )
        set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
        message(STATUS "ONNX Runtime library: ${ONNX_RUNTIME_LIB_PATH}")
    else()
        message(WARNING "ONNX Runtime library not found")
        set(USE_ONNX "OFF")
    endif()
else()
    message(WARNING "ONNX Runtime not found. Please run: bash scripts/download_inference_runtime.sh onnx")
endif()

if(USE_ONNX)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${ONNX_RUNTIME_DIR}/lib")
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
    endif()
endif()

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
if(Python3_LIBRARIES)
    get_filename_component(PYTHON_LIB_DIR "${Python3_LIBRARIES}" DIRECTORY)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PYTHON_LIB_DIR}")
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(YAML_CPP_LIBRARIES yaml-cpp)
    link_directories(/usr/local/lib)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
    set(YAML_CPP_LIBRARIES ${YAML_CPP_LIB})
endif()

include_directories(
    include
    library/core/observation_buffer
    library/core/rl_sdk
    library/core/fsm
    library/core/vector_math
    library/core/motion_loader
    library/core/inference_runtime
    library/core/logger
    fsm_robot
)

add_library(inference_runtime library/core/inference_runtime/inference_runtime.cpp)
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(inference_runtime PRIVATE USE_ONNX)
endif()

add_library(motion_loader library/core/motion_loader/motion_loader.cpp)

add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk PUBLIC motion_loader inference_runtime Python3::Python Python3::Module TBB::tbb)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()

add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)

include(cmake/dian.cmake)

message(STATUS "RL_SAR ${PROJECT_VERSION} ${CMAKE_SYSTEM_PROCESSOR} ${CMAKE_BUILD_TYPE} ONNX=${USE_ONNX}")
