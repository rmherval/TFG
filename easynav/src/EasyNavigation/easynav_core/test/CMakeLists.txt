ament_add_gtest(core_method_test core_method_test.cpp)
target_link_libraries(core_method_test
  ${PROJECT_NAME}
)
