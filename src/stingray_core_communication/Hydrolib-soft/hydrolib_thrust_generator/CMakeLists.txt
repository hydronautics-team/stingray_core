add_library(HydrolibThrustGenerator INTERFACE)

target_include_directories(HydrolibThrustGenerator INTERFACE include)

include(CTest)

if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)

    add_executable(TestThrustGenerator ${TESTS})
    set_target_properties(TestThrustGenerator
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestThrustGenerator PUBLIC cxx_std_20)

    target_compile_options(TestThrustGenerator PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override -fsanitize=address -fsanitize=undefined)
    target_link_options(TestThrustGenerator PRIVATE -fsanitize=address -fsanitize=undefined)
    target_link_libraries(TestThrustGenerator HydrolibThrustGenerator GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestThrustGenerator)
endif()
