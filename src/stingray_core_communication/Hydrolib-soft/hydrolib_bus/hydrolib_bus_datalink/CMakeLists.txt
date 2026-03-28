add_library(HydrolibBusDatalink INTERFACE)

target_include_directories(HydrolibBusDatalink INTERFACE include)

target_link_libraries(HydrolibBusDatalink INTERFACE HydrolibConcepts
    INTERFACE HydrolibLogger INTERFACE HydrolibReturnCodes INTERFACE HydrolibCRC)

include(CTest)

if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)

    add_executable(TestBusDatalink ${TESTS})
    set_target_properties(TestBusDatalink
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestBusDatalink PUBLIC cxx_std_20)

    target_compile_options(TestBusDatalink PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override -fsanitize=address -fsanitize=undefined)
    target_link_options(TestBusDatalink PRIVATE -fsanitize=address -fsanitize=undefined)
    target_link_libraries(TestBusDatalink HydrolibBusDatalink GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestBusDatalink)
endif()
