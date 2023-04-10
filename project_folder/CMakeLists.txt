cmake_minimum_required(VERSION 3.8)

# set the project name
project(TemplateProject VERSION 1.0 DESCRIPTION "template_project_start" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_CLANG_TIDY clang-tidy)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#add subdirectories
add_subdirectory(src)
add_subdirectory(app)
enable_testing()
add_subdirectory(test)



