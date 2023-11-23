# List the EMT library source files into a variable
set(EMT_LIB_SOURCES
    src/KalmanFilter.cpp
)

# creating a CTR static library
add_library(EMT STATIC ${EMT_LIB_SOURCES})
target_include_directories(EMT PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)