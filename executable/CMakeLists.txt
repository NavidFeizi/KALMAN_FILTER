add_executable(${PROJECT_NAME} Main.cpp) # ${CMAKE_SOURCE_DIR}/ndi_api/src/UdpSocket.cpp)

# Link the Kalman Filter static library
target_link_libraries(${PROJECT_NAME} PUBLIC EMT)