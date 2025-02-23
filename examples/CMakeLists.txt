cmake_minimum_required(VERSION 3.16 )
set(CMAKE_BUILD_TYPE Release)
project(Qontrol_examples
        VERSION 0.0.1
        LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include(FetchContent)

FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/deepmind/mujoco.git
    GIT_TAG 3.0.0
)

set(MUJOCO_BUILD_EXAMPLES OFF)
set(MUJOCO_BUILD_TESTS OFF)
set(MUJOCO_BUILD_SIMULATE OFF)
set(MUJOCO_TEST_PYTHON_UTIL OFF)
FetchContent_MakeAvailable(mujoco)

set(MUJOCO_DEP_VERSION_glfw3
    7482de6071d21db77a7236155da44c172a7f6c9e # 3.3.8
    CACHE STRING "Version of `glfw` to be fetched."
)

find_package(glfw QUIET)
if (NOT glfw)
  set(GLFW_BUILD_EXAMPLES OFF)
  set(GLFW_BUILD_TESTS OFF)
  FetchContent_Declare(
    glfw3
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG ${MUJOCO_DEP_VERSION_glfw3}
  )
  FetchContent_MakeAvailable(glfw3)
endif()
  

set(OpenGL_GL_PREFERENCE LEGACY)
link_libraries(mujoco::mujoco)

# Building mujoco_sim library
add_library(platform_ui_adapter_custom mujoco/platform_ui_adapter.cc)
target_compile_features(platform_ui_adapter_custom PRIVATE cxx_std_17)
target_link_libraries(platform_ui_adapter_custom mujoco::mujoco glfw)    

add_library(glfw_dispatch_custom mujoco/glfw_dispatch.cc)
target_compile_features(glfw_dispatch_custom PRIVATE cxx_std_17)
target_link_libraries(glfw_dispatch_custom mujoco::mujoco glfw)    

add_library(glfw_adapter_custom mujoco/glfw_adapter.cc)
target_compile_features(glfw_adapter_custom PRIVATE cxx_std_17)
target_link_libraries(glfw_adapter_custom platform_ui_adapter_custom glfw_dispatch_custom mujoco::mujoco glfw)    

add_library(simulate_custom mujoco/simulate.cc)
target_compile_features(simulate_custom PRIVATE cxx_std_17)
target_link_libraries(simulate_custom glfw_adapter_custom platform_ui_adapter_custom glfw_dispatch_custom mujoco::mujoco glfw)    

add_library(mujoco_sim mujoco/mujoco_sim.cpp)
target_compile_features(mujoco_sim PRIVATE cxx_std_17)
target_link_libraries(mujoco_sim Eigen3::Eigen glfw_adapter_custom platform_ui_adapter_custom glfw_dispatch_custom mujoco::mujoco glfw)    

# Build examples

set(EXAMPLE_NAMES
        velocityQontrol
        torqueQontrol
        # Qontrol_qpOASES
        Qontrol_qpmad
        QontrolCustomConstraint
        QontrolCustomTask
    )

foreach(example ${EXAMPLE_NAMES})
  add_executable(${example} ${example}.cpp)
  target_compile_features(${example} PRIVATE cxx_std_17) 
  target_link_libraries(${example} mujoco_sim simulate_custom Qontrol)
endforeach(example ${EXAMPLE_NAMES})

# Installing resources

file(COPY resources DESTINATION .)

