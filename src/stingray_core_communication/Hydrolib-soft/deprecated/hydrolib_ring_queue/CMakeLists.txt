cmake_minimum_required(VERSION 3.10)

project(HydrolibRingQueue VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)

add_library(HydrolibRingQueue src/hydrolib_ring_queue.c)

target_include_directories(HydrolibRingQueue PUBLIC include)

target_link_libraries(HydrolibRingQueue HydrolibCommon)

include(CTest)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)
    add_executable(TestRingBuffer ${TESTS})
    set_target_properties(TestRingBuffer
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestRingBuffer PUBLIC cxx_std_20)
    target_compile_options(TestRingBuffer PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override)
    target_link_libraries(TestRingBuffer HydrolibRingQueue GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestRingBuffer)
endif()
