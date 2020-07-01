# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(gdaltest)


find_package(GDAL REQUIRED)


message(STATUS "GDAL library status:")
message(STATUS "    version: ${GDAL_VERSION}")
message(STATUS "    libraries: ${GDAL_LIBRARY}")
message(STATUS "    include path: ${GDAL_INCLUDE_DIRS}")


include_directories(${GDAL_INCLUDE_DIRS})


add_executable(main gdal.cc)


target_link_libraries(main ${GDAL_LIBRARY})
