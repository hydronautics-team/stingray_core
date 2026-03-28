add_library(HydrolibLogger INTERFACE)

target_include_directories(HydrolibLogger INTERFACE include)

target_link_libraries(HydrolibLogger INTERFACE HydrolibReturnCodes INTERFACE HydrolibStrings)

if(BUILD_TESTS)
    include(CTest)

    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)
    add_executable(TestLogger ${TESTS})
    set_target_properties(TestLogger
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestLogger PUBLIC cxx_std_20)
    target_compile_options(TestLogger PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override)
    target_link_libraries(TestLogger HydrolibLogger GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestLogger)
endif()
