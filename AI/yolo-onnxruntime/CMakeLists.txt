cmake_minimum_required(VERSION 3.10)

project(yoloOrt)
add_executable(yoloOrt main.cpp yolo.cpp)

# connect opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARIES})



# connect onnxruntime
set(ONNXRUNTIME_INCLUDE_DIRS /usr/local/include/onnxruntime/core/session)
set(ONNXRUNTIME_LIB /usr/local/lib/libonnxruntime.so)
include_directories(${ONNXRUNTIME_INCLUDE_DIRS})

target_link_libraries(yoloOrt PRIVATE ${OpenCV_LIBRARIES} ${ONNXRUNTIME_LIB})