# 手工调试程序：需要人眼看输出，不属于自动化测试，所以不注册到 CTest。
if(COVERAGE_WITH_FOXGLOVE)
  add_executable(stc_debug stc_debug.cc)
  target_link_libraries(stc_debug
      PRIVATE
      foxglove_cpp
  )
endif()

if(NOT COVERAGE_BUILD_TESTS)
  return()
endif()

if(NOT TARGET coverage_path_planning::gtest)
  message(STATUS
      "未找到 GTest，跳过单元测试目标（安装 libgtest-dev 或 -DCOVERAGE_BUILD_TESTS=OFF）")
  return()
endif()

include(GoogleTest)

# ASan 的运行期选项。detect_container_overflow 必须关闭：我们的代码被插桩了，
# 而系统的 libstdc++ 与 GTest 没有，混用时该检查会报假阳性。
# 权限受限的容器里 LeakSanitizer 无法 ptrace，需要额外追加 :detect_leaks=0。
set(COVERAGE_ASAN_TEST_OPTIONS "detect_container_overflow=0"
    CACHE STRING "测试运行时的 ASAN_OPTIONS")

# 每个被测组件一个可执行文件；gtest_discover_tests 把其中每个 TEST 单独注册
# 成一个 CTest 用例，这样 ctest -R 可以按名字精准重跑单个用例。
# DISCOVERY_MODE PRE_TEST：在 ctest 阶段而非构建阶段执行测试二进制文件，
# 否则打开 sanitizer 或交叉编译时，构建会因为跑不了该文件而失败。
add_executable(path_smoother_test path_smoother_test.cc)
target_link_libraries(path_smoother_test
    PRIVATE
    coverage_path_planning::planning
    coverage_path_planning::gtest
    coverage_path_planning::warnings
)
if(COVERAGE_ENABLE_SANITIZERS)
  # sanitizer 构建下，执行测试二进制文件需要带上 ASAN_OPTIONS，而 gtest_discover_tests
  # 无法为“测试发现”这一步设置环境变量（只能设置给发现出来的用例）。
  # 所以这里退回“一个二进制文件一个 CTest 用例”，细粒度筛选用 --gtest_filter。
  add_test(NAME path_smoother_test COMMAND path_smoother_test)
  set_tests_properties(path_smoother_test PROPERTIES
      ENVIRONMENT "ASAN_OPTIONS=${COVERAGE_ASAN_TEST_OPTIONS}")
else()
  gtest_discover_tests(path_smoother_test)
endif()


add_executable(fox_backcall fox_backcall.cc)
target_link_libraries(fox_backcall
    PRIVATE
    foxglove_cpp
)