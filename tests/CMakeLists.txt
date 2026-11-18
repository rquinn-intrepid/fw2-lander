cmake_minimum_required(VERSION 3.13)
project(fw2_lander_tests C)
set(CMAKE_C_STANDARD 11)
enable_testing()

# Host-only: the guidance/descent model has no Pico SDK or hardware dependency.
add_executable(test_lander_sim
    test_lander_sim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/lander_sim.c)
target_include_directories(test_lander_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
if(NOT MSVC)
    target_link_libraries(test_lander_sim m)
endif()
add_test(NAME lander_sim COMMAND test_lander_sim)
