cmake_minimum_required (VERSION 3.0)
project (FourierBlender)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_library (FourierBlender SHARED core.cpp)
target_link_libraries (FourierBlender Eigen3::Eigen)

set(pydest "NOT_SET" CACHE PATH "Target directory for the python modules, e.g. ~/blender-2.80/2.80/python/lib/python3.7")

if (pydest STREQUAL "NOT_SET")
  message( SEND_ERROR "Need to specify the pydest" )
endif()

configure_file(FourierBlender_interface.py.in FourierBlender_interface.py @ONLY)

install(FILES
  ${CMAKE_BINARY_DIR}/FourierBlender_interface.py
  ${CMAKE_SOURCE_DIR}/FourierBlender_addArrow.py
  DESTINATION ${pydest}
)
