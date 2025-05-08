cmake_minimum_required(VERSION 3.19)
project(Collision3d)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_COLOR_DIAGNOSTICS ON)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
else()
endif()

add_subdirectory(SpatialPartitioning)

include_directories(include)

aux_source_directory(./include/collision3d/ header_files)
aux_source_directory(./src/ source_files)

add_library(collision3d STATIC
	${header_files}
	${source_files}
)
target_link_libraries(collision3d PUBLIC spatial_partitioning)
