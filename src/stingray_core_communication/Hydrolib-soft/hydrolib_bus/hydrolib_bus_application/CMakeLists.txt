add_library(HydrolibBusApplication INTERFACE)

target_include_directories(HydrolibBusApplication INTERFACE include)

target_link_libraries(HydrolibBusApplication INTERFACE HydrolibConcepts
    INTERFACE HydrolibLogger INTERFACE HydrolibReturnCodes)

include(CTest)

if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)

    add_executable(TestBusApplication ${TESTS})
    set_target_properties(TestBusApplication
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestBusApplication PUBLIC cxx_std_20)

    target_compile_options(TestBusApplication PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override -fsanitize=address -fsanitize=undefined)
    target_link_options(TestBusApplication PRIVATE -fsanitize=address -fsanitize=undefined)
    target_link_libraries(TestBusApplication HydrolibBusApplication GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestBusApplication)
endif()
