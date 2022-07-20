cmake_minimum_required(VERSION 3.12.1)
project(LCCPSeg)

#-----------------------------------------------------------------------------
# Enable C++11
#-----------------------------------------------------------------------------
set(_msg "Setting C++ standard")
message(STATUS "${_msg}")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
message(STATUS "${_msg} - C++${CMAKE_CXX_STANDARD}")
if(NOT CMAKE_CXX_STANDARD MATCHES "^(11|14|17)$")
  message(FATAL_ERROR "CMAKE_CXX_STANDARD must be set to 11, 14 or 17")
endif()

#-----------------------------------------------------------------------------
# Qt - REQUIRED_QT_MODULES
#-----------------------------------------------------------------------------
# Module name should be specified as they appear in FindQt5.cmake.
# For example, the module name associated with the variable QT_USE_QTXML is QTXML.
# Note that the modules will be installed when packaging.
  set(${PROJECT_NAME}_REQUIRED_QT_MODULES
    Core Widgets 
    )
  find_package(Qt5 COMPONENTS Core Widgets Gui Core REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#find Qt5 LIB
  set(QT_LIBRARIES)
  foreach(lib IN LISTS ${PROJECT_NAME}_REQUIRED_QT_MODULES)
    if(lib MATCHES "^(LinguistTools)$")
      continue()
    endif()
    list(APPEND QT_LIBRARIES Qt5::${lib})
  endforeach()

#----------------------------------------------------------------------------
# PCL
#----------------------------------------------------------------------------
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Source files
set(KIT_SRCS
    src/main.cpp
    src/lccp_segmentation.cpp
    src/mainwindow.cpp
  )

# Headers that should run through moc
set(KIT_MOC_SRCS
    src/lccp_segmentation.h
    src/mainwindow.h
    src/lccp_segmentation.hpp 
  )

add_executable(${PROJECT_NAME} ${KIT_SRCS} ${KIT_MOC_SRCS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${PCL_LIBRARIES})
