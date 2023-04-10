# #add an executable that call the contructor of the class 
add_executable(app subfolder/app.cpp)
# compiler version
target_compile_features(app PUBLIC cxx_std_17)
#link the library created above since in the file is included the calculator.hpp
target_link_libraries(app PUBLIC CustomLibrary)
