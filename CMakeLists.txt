cmake_minimum_required(VERSION 2.6)

project (hdr)

add_definitions(-Wall -O2 -std=c++11 -g -Wno-unused-local-typedefs )

find_package(JPEG REQUIRED)

include_directories(~/eigen include)

#Déclaration du fichier dans lequel on met les executables
set(EXECUTABLE_OUTPUT_PATH bin)

add_subdirectory(src)

#Déclaration de l'exécutable
add_executable(

        hdr
        src/main.cpp
)

target_link_libraries(hdr image jpeg)


