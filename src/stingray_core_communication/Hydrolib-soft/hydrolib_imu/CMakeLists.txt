set(LIBRARY_NAME HydrolibIMU)

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE include)

target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibMath)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)
    add_executable(TestIMU ${TESTS})
    set_target_properties(TestIMU
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestIMU PUBLIC cxx_std_20)
    target_compile_options(TestIMU PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override)
    # target_compile_options(TestSerialProtocol PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
    #     -Woverloaded-virtual -Wsuggest-override -fsanitize=address -fsanitize=undefined)
    # target_link_options(TestSerialProtocol PRIVATE -fsanitize=address -fsanitize=undefined)
    target_link_libraries(TestIMU HydrolibIMU GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestIMU)
endif()
