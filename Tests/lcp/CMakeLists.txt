FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_executable(gtest_moby_lcp_tests ${headers} ${sources})
target_link_libraries(gtest_moby_lcp_tests gtest_main SJMobyLCP)