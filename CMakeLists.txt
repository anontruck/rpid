cmake_minimum_required(VERSION 3.0.0)
project(rpid VERSION 0.0.0)

include_directories(
    includes
)
file (GLOB SOURCES "src/*.c")

add_executable(rpid src/main.c)

#add_library(rpid STATIC src/rpid.c)
#install(TARGETS rpid DESTINATION lib)
#install(FILES rpid.h DESTINATION includes)