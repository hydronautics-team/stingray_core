set(LIBRARY_NAME HydrolibACS)
set(TEST_TARGET_NAME Test${LIBRARY_NAME})

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE include)

target_link_libraries(${LIBRARY_NAME} INTERFACE HydrolibPID INTERFACE HydrolibIMU INTERFACE HydrolibPRESSURE)

if(BUILD_TESTS)
    include(CTest)

    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)
    add_executable(${TEST_TARGET_NAME} ${TESTS})
    set_target_properties(${TEST_TARGET_NAME}
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(${TEST_TARGET_NAME} PUBLIC cxx_std_20)
    target_compile_options(${TEST_TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override)
    target_link_libraries(${TEST_TARGET_NAME} ${LIBRARY_NAME} GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(${TEST_TARGET_NAME})
endif()