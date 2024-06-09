#common headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB COMMON_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/common/*.h")

add_subdirectory ("reconstruction")