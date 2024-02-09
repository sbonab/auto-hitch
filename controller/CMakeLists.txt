
# Set the project name
project(Controller)

cmake_minimum_required(VERSION 3.10)

#find_package(Boost REQUIRED)
#find_package(Threads REQUIRED)

#SET( CMAKE_CXX_COMPILER "C:/MinGW/bin/g++" )

# Set the C++ standard to use
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add an executable target
add_executable(controller src/main.cpp src/InputController.cpp src/Point.cpp)

# Set the CXX flags to include debugging symbols
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Set the include directory
target_include_directories(controller PUBLIC inc)

# Set the source files
#target_sources(main PRIVATE src/main.cpp)

target_link_libraries(controller
    PUBLIC  
        #Boost::boost # Target for header-only dependencies. (Boost include directory)
        #Threads::Threads
)

# Set the output directory
set_target_properties(controller PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)