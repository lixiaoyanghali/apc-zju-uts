project(shelf_processing)
cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find package OpenMP
find_package( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message(" == OPENMP FOUND ${OpenMP_C_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

pkg_check_modules(EIGEN REQUIRED eigen3)
message( "== EIGEN INCLUDE DIRS ${EIGEN_INCLUDE_DIRS}")
include_directories( ${EIGEN_INCLUDE_DIRS} )

find_package(Boost 1.40 COMPONENTS program_options system filesystem thread regex REQUIRED)

message( "== BOOST LIBRARIES ${Boost_LIBRARIES} ")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

include_directories( ${PROJECT_BINARY_DIR} )
add_executable(main main.cpp)
target_link_libraries( main ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
