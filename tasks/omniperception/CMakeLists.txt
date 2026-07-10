cmake_minimum_required(VERSION 3.16)

if(NOT OpenVINO_DIR)
  if(EXISTS "/opt/intel/openvino_2024.6.0/runtime/cmake")
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
  elseif(EXISTS "/opt/intel/openvino/runtime/cmake")
    set(OpenVINO_DIR "/opt/intel/openvino/runtime/cmake")
  endif()
endif()

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT 
    decider.cpp
    perceptron.cpp
)
target_link_libraries(omniperception openvino::runtime)
