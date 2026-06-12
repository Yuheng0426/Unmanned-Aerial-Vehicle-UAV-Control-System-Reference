# Top-level CMake project file.
cmake_minimum_required(VERSION 3.16)

# Enable MSVC hot reload debug information when the policy is available.
if(POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project(UAVcontrol_system
  VERSION 0.1.0
  DESCRIPTION "Educational UAV flight-control reference simulator"
  LANGUAGES CXX
)

# Enable tests at the root so ctest can discover them from the build directory.
include(CTest)

# Add the flight-control demo target.
add_subdirectory("UAVcontrol_system")
