##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(parkingUI)
#add
set(CMAKE_INCLUDE_CURRENT_DIR ON)
##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    parking_msgs
    )
find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

#add_service_files(
#  FILES
#  order.srv
#)

#add
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

#add_library(parkingUI
#  src/ParkingInfo.cpp
#  src/ParkingLot.cpp
#)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#remove
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/parkingUI/*.hpp *.h)



#change
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(parkingUI ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})


add_dependencies(parkingUI parkingUI)


FIND_LIBRARY(mylib1_LIBRARIES parkingUI src/ParkingInfo.cpp)
FIND_LIBRARY(mylib2_LIBRARIES parkingUI src/ParkingLot.cpp)

target_link_libraries(parkingUI ${QT_LIBRARIES} ${catkin_LIBRARIES} )#${mylib_LIBRARIES})
install(TARGETS parkingUI RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

