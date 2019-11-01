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

set(pydest "NOT_SET" CACHE PATH "Target directory for the python modules, e.g. ~/blender-2.80/2.80/python/lib/python3.7")

if (pydest STREQUAL "NOT_SET")
  message( SEND_ERROR "Need to specify the pydest" )
endif()

configure_file(FourierBlender_interface.py.in FourierBlender_interface.py @ONLY)

install(FILES
  ${CMAKE_BINARY_DIR}/FourierBlender_interface.py
  ${CMAKE_SOURCE_DIR}/FourierBlender_helper.py
  DESTINATION ${pydest}
)
