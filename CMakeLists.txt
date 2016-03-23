cmake_minimum_required(VERSION 2.8.3)
project(nostop_analysis)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  base_math
)

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  DEPENDS base_math
)

include_directories(include/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(analysis_SOURCES
        src/Analysis.cpp
        src/AnalysisElement.cpp
        src/main.cpp
        src/LatticeCache.cpp
)

add_executable(${PROJECT_NAME}_node ${analysis_SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} base_math)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp)

install(TARGETS ${PROJECT_NAME}_node
 EXPORT nostop_analysis
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
