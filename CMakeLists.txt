cmake_minimum_required(VERSION 3.0.2)
project(alphasense_description)

find_package(catkin REQUIRED COMPONENTS xacro)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(yaml-cpp)
include_directories(${YAML_CPP})

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(CATKIN_DEPENDS xacro
               DEPENDS EIGEN3 YAML_CPP Boost)

set(xacro_files
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/alphasense_standalone.urdf.xacro
)
xacro_add_files(${xacro_files} INSTALL DESTINATION urdf)

add_executable(kalibr_urdf_calibration_converter src/kalibr_urdf_calibration_converter.cpp)
target_link_libraries(kalibr_urdf_calibration_converter ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS kalibr_urdf_calibration_converter
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

