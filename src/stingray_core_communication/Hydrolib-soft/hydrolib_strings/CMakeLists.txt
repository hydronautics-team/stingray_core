set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(HydrolibStrings INTERFACE)

target_include_directories(HydrolibStrings INTERFACE include)

target_link_libraries(HydrolibStrings INTERFACE HydrolibReturnCodes INTERFACE HydrolibConcepts
    INTERFACE HydrolibMath)

if(BUILD_TESTS)
    include(CTest)

    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)
    add_executable(TestStrings ${TESTS})
    set_target_properties(TestStrings
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestStrings PUBLIC cxx_std_20)
    target_compile_options(TestStrings PRIVATE -save-temps=obj)
    target_compile_options(TestStrings PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override -fno-exceptions -fno-rtti)
    target_link_libraries(TestStrings HydrolibStrings GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestStrings)
endif()
