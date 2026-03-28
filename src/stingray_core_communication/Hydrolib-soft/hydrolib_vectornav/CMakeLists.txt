cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(HydrolibVectorNAV INTERFACE)

target_include_directories(HydrolibVectorNAV INTERFACE include)

target_link_libraries(HydrolibVectorNAV INTERFACE HydrolibLogger INTERFACE HydrolibIMU INTERFACE HydrolibReturnCodes)

include(CTest)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)
    add_executable(TestVectorNAV ${TESTS})
    set_target_properties(TestVectorNAV
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestVectorNAV PUBLIC cxx_std_20)
    target_compile_options(TestVectorNAV PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override)
    target_link_libraries(TestVectorNAV HydrolibVectorNAV HydrolibLogger GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestVectorNAV)
endif()
