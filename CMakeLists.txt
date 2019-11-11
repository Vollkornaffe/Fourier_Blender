cmake_minimum_required (VERSION 3.0)
project (FourierBlender)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_library (FourierBlender SHARED core.cpp)
target_link_libraries (FourierBlender Eigen3::Eigen)

# this is a really ugly solution, but it should work
# the problem is that the binary code isn't reloaded after a change 
# the workaround is to name the liraries different each time
if (NOT DEFINED CACHE{RELOAD_COUNTER})
  set(RELOAD_COUNTER "0" CACHE INTERNAL "")
else()
  math(EXPR RELOAD_COUNTER "${RELOAD_COUNTER} + 1")
  set(RELOAD_COUNTER ${RELOAD_COUNTER} CACHE INTERNAL "")
endif()
message("Reload counter at ${RELOAD_COUNTER}")
set_target_properties(FourierBlender PROPERTIES SUFFIX ".so.${RELOAD_COUNTER}")

configure_file(FourierBlender.py.in FourierBlender.py @ONLY)
configure_file(FourierBlender_path.py.in FourierBlender_path.py @ONLY)

install(FILES
  ${CMAKE_SOURCE_DIR}/FourierBlender_blender.py
  ${CMAKE_SOURCE_DIR}/FourierBlender_interface.py
  ${CMAKE_BINARY_DIR}/FourierBlender_path.py
  ${CMAKE_BINARY_DIR}/FourierBlender.py
  DESTINATION ${CMAKE_INSTALL_PREFIX}/FourierBlender
)
install(TARGETS FourierBlender
  DESTINATION ${CMAKE_INSTALL_PREFIX}/FourierBlender)
