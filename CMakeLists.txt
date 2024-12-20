#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(MyColorViewer)

add_executable(${PROJECT_NAME}
  src/ColorViewer.cpp
)

find_package(OpenCV REQUIRED)

set(OrbbecSDK_LIBS  /home/enes/Desktop/ORBBEC/OrbbecSDK/lib/linux_x64)
set(OrbbecSDK_INCLUDE /home/enes/Desktop/ORBBEC/OrbbecSDK/include) # Adjust this path as necessary
set(OrbbecSDK_INCLUDE_HPP /home/enes/Desktop/ORBBEC/OrbbecSDK/include/libobsensor/hpp
                          /home/enes/Desktop/ORBBEC/OrbbecSDK/examples/cpp) # Adjust this path as necessary

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  )

# Link the required libraries from Orbbec SDK
target_link_libraries(${PROJECT_NAME}
  ${OrbbecSDK_LIBS}/libOrbbecSDK.so
  ${OrbbecSDK_LIBS}/libdepthengine.so
  ${OrbbecSDK_LIBS}/libob_usb.so
  ${OrbbecSDK_LIBS}/liblive555.so
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${OrbbecSDK_INCLUDE}
  ${OrbbecSDK_INCLUDE_HPP}
  )

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )

  # Print project name
message(STATUS "Project Name: ${PROJECT_NAME}")

# Print OpenCV libraries
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

# Print Orbbec SDK libraries
message(STATUS "OrbbecSDK_LIBS: ${OrbbecSDK_LIBS}")

# Print current binary directory
message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")

# Print current source directory
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

# Print OpenCV include directories
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# Print Orbbec SDK include directories
message(STATUS "OrbbecSDK_INCLUDE_DIRS: ${OrbbecSDK_INCLUDE_DIRS}")
