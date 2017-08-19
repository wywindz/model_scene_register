cmake_minimum_required(VERSION 2.8)

project(model_scene_register_test)

find_package(PCL 1.7 REQUIRED PATHS /usr NO_DEFAULT_PATH)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(${PCL_INCLUDE_DIRS} ${PROJ_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
