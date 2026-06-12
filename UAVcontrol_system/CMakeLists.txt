# Build the flight-control demo executable.
add_executable(UAVcontrol_system
  "UAVcontrol_system.cpp"
  "UAVcontrol_system.h"
)

# Use modern C++ without third-party runtime dependencies.
target_compile_features(UAVcontrol_system PRIVATE cxx_std_20)

# Enable common compiler warnings so contributors catch issues early.
if(MSVC)
  target_compile_options(UAVcontrol_system PRIVATE /W4 /permissive- /utf-8)
else()
  target_compile_options(UAVcontrol_system PRIVATE -Wall -Wextra -Wpedantic)
endif()

# The self-test checks arming safety, failsafe disarming, and mixer limits.
if(BUILD_TESTING)
  add_test(NAME self_test COMMAND UAVcontrol_system --self-test)
endif()
