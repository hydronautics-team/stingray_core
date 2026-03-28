cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_INCLUDE_DIRECTORIES_BEFORE ON)

add_library(HydrolibPID INTERFACE)

target_include_directories(HydrolibPID INTERFACE include)

include(CTest)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)
    add_executable(TestPID ${TESTS})
    set_target_properties(TestPID
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestPID PUBLIC cxx_std_20)
    target_compile_options(TestPID PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override)
    # target_compile_options(TestSerialProtocol PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
    #     -Woverloaded-virtual -Wsuggest-override -fsanitize=address -fsanitize=undefined)
    # target_link_options(TestSerialProtocol PRIVATE -fsanitize=address -fsanitize=undefined)
    target_link_libraries(TestPID HydrolibPID GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestPID)
endif()
