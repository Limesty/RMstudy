cmake_minimum_required (VERSION 3.8)

project ("sas-vision-2023")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} "./include")
link_directories("./lib")

#add_executable (test "test.cpp" "AI/yolo-opencv/yolo.cpp")
add_executable (main "main.cpp" "AI/yolo-onnxruntime/yolo.cpp" "Aim/Camera/Camera.cpp" "Aim/Solver/solver.cpp" "Aim/Serial/serial.cpp") #"Auto/aiming/Aiming_CTRV.cpp"

#target_include_directories(test PRIVATE ${OpenCV_INCLUDE_DIRS})
#target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS} "include")

#target_link_libraries(test PRIVATE ${OpenCV_LIBS})
target_link_libraries(main PRIVATE ${OpenCV_LIBS} "libonnxruntime.so" "libMVSDK.so")

#target_link_libraries(test PRIVATE "/usr/lib/libMVSDK.so")
