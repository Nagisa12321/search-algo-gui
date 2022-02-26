cmake_minimum_required(VERSION 3.5)

project(dijkstra-gui)

# debug mode
add_compile_options(-g)

# compile json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# SDL2 library
find_package(SDL2 REQUIRED)

# the source files
file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

foreach(SOURCE ${SOURCES})
    get_filename_component(EXEC_NAME ${SOURCE} NAME)
    add_executable(${EXEC_NAME} ${SOURCE})
   
    # include dir
    include_directories(${EXEC_NAME} ${SDL2_INCLUDE_DIRS})
    
    # link SDL2 library 
    target_link_libraries(${EXEC_NAME} SDL2)
endforeach()



