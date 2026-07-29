# Plain-assert unit tests; no framework dependency so they run anywhere.
#
# Every test here asserts. A Release build defines NDEBUG, which turns
# assert() into a no-op -- the tests would then run, check nothing, and
# report success. Undefine it for test targets so the suite is always
# armed regardless of CMAKE_BUILD_TYPE.
function(spite_d_add_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE ${ARGN})
  target_compile_options(${name} PRIVATE -UNDEBUG)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

spite_d_add_test(test_predictor spite_d_trajectory)

spite_d_add_test(test_replanner spite_d_planner)

if(TARGET spite_d_validity)
  spite_d_add_test(test_validity_server spite_d_validity spite_d_trajectory spite_d_planner)

  spite_d_add_test(test_roadmap_roundtrip spite_d_validity spite_d_trajectory spite_d_planner)
endif()

spite_d_add_test(test_span spite_d_dynamic_map)

if(TARGET spite_d_validity)
  spite_d_add_test(test_span_pipeline spite_d_dynamic_map spite_d_validity spite_d_trajectory)
endif()

if(TARGET spite_d_perception)
  spite_d_add_test(test_perception spite_d_perception)
endif()
