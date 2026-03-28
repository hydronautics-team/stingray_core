cmake_minimum_required(VERSION 3.10)

project(HydrolibCommon VERSION 1.0)

include_directories(include)

add_library(HydrolibCommon INTERFACE)

target_include_directories(HydrolibCommon INTERFACE include)
