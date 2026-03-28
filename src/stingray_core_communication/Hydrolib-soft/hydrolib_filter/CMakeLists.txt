set(LIBRARY_NAME HydrolibFilter)

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE include)

if(BUILD_TESTS)
    include(CTest)

    find_package(GTest REQUIRED)
    include(GoogleTest)

    file(GLOB TESTS tests/*.cpp)
    add_executable(Test${LIBRARY_NAME} ${TESTS})
    set_target_properties(Test${LIBRARY_NAME}
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing"
    )
    target_compile_features(Test${LIBRARY_NAME} PUBLIC cxx_std_20)
    target_compile_options(Test${LIBRARY_NAME} PRIVATE -save-temps=obj)
    target_compile_options(Test${LIBRARY_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wc++20-compat -Wno-format-security
        -Woverloaded-virtual -Wsuggest-override -fno-exceptions -fno-rtti)
    target_link_libraries(Test${LIBRARY_NAME} ${LIBRARY_NAME} HydrolibMath GTest::gtest GTest::gtest_main -pthread)
    gtest_discover_tests(Test${LIBRARY_NAME})
endif()
