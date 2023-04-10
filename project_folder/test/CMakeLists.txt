# # Testing library
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)

FetchContent_MakeAvailable(googletest)

add_executable(test-source subfolder/test-source.cpp)

set(TEST_LIBS_CALC pthread)
target_link_libraries(test-source PUBLIC ${TEST_LIBS_CALC} CustomLibrary gtest_main)

# Register your GTest test files with CTest
include(GoogleTest)
gtest_discover_tests(test-source)


enable_testing()

