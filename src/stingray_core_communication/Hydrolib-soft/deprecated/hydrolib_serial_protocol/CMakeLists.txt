add_library(HydrolibSerialProtocol INTERFACE)

target_include_directories(HydrolibSerialProtocol INTERFACE include)

target_link_libraries(HydrolibSerialProtocol INTERFACE HydrolibCommon INTERFACE HydrolibConcepts
    INTERFACE HydrolibLogger)

include(CTest)

if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)

    # add_executable(TestSerialProtocol ${TESTS})
    add_executable(TestSerialProtocol
        tests/test_hydrolib_sp_serialize_env.cpp
        tests/test_hydrolib_sp_serialize_write.cpp
        tests/test_hydrolib_sp_serialize_read.cpp
        tests/test_hydrolib_sp_serialize_responce.cpp
        tests/test_hydrolib_sp_serialize_error.cpp
        tests/test_hydrolib_sp_interpreter_env.cpp
        tests/test_hydrolib_sp_interpreter.cpp
        tests/test_hydrolib_sp_communicate_env.cpp
        tests/test_hydrolib_sp_communicate_write.cpp
        tests/test_hydrolib_sp_communicate_read.cpp)
    set_target_properties(TestSerialProtocol
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(TestSerialProtocol PUBLIC cxx_std_20)

    target_compile_options(TestSerialProtocol PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override -fsanitize=address -fsanitize=undefined)
    target_link_options(TestSerialProtocol PRIVATE -fsanitize=address -fsanitize=undefined)
    target_link_libraries(TestSerialProtocol HydrolibSerialProtocol GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(TestSerialProtocol)
endif()
