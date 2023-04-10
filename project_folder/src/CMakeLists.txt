cmake_minimum_required(VERSION 3.8)

#add the library
add_library(CustomLibrary STATIC subfolder/source.cpp)

#add target include directories of header files
target_include_directories(CustomLibrary PUBLIC ${CMAKE_HOME_DIRECTORY}/include/)

# All users of this library will need at least C++11
target_compile_features(CustomLibrary PUBLIC cxx_std_17)