# List the EMT library source files into a variable
set(EMT_LIB_SOURCES
    src/KalmanFilter.cpp
)

# creating a Kalman Filter static library
add_library(EMT STATIC ${EMT_LIB_SOURCES})
target_include_directories(EMT PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


# ================================= BLAS library =================================
set(BLA_VENDOR OpenBLAS)
set(BLA_STATIC                   ON)  # use static linkage
find_package(BLAS REQUIRED)

if (BLAS_FOUND)
    message(STATUS "BLAS has been found!")
    include_directories(${BLAS_INCLUDE_DIRS})
    target_link_libraries(EMT PUBLIC ${BLAS_LIBRARIES})
elseif(NOT BLAS_FOUND)
    error(STATUS "BLAS library has not been found")
endif()


# ================================= LAPACK library =================================
find_package(LAPACK REQUIRED)

# Check for the LAPACK libray, if found print message, include dirs and link libraries.
if(LAPACK_FOUND)
    message("LAPACK has been found!")
    include_directories(${LAPACK_INCLUDE_DIRS})
    target_link_libraries(EMT PUBLIC ${LAPACK_LIBRARIES})
elseif(NOT LAPACK_FOUND)
    error("LAPACK API has not been found")
endif(LAPACK_FOUND)


# ================================= Blaze library =================================
find_package(blaze CONFIG REQUIRED)

# Check for the Blaze libray, if found print message, include dirs and link libraries.
if(blaze_FOUND)
    message(STATUS "Blaze has been found!")
    # target_link_libraries(${PROJECT_NAME} INTERFACE blaze::blaze)
    target_link_libraries(EMT INTERFACE blaze::blaze)
elseif(NOT Blaze_FOUND)
    error(STATUS "Blaze has not been found!")
endif(blaze_FOUND)