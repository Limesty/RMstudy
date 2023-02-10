cmake_minimum_required (VERSION 3.8)

project ("aiming")

find_package(OpenCV REQUIRED)

set(SOURCES main.cpp)

add_executable("aiming" "main.cpp")

target_include_directories("aiming" PRIVATE ${OpenCV_INCLUDE_DIRS}
                                        ${PROJECT_SOURCE_DIR}/include)


target_link_libraries("aiming"  ${OpenCV_LIBS})
