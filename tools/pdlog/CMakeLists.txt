add_library(pdlog_core STATIC
    application.cc
    config.cc
    log_reader.cc
    ppmd_decoder.cc
)
target_include_directories(pdlog_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(pdlog_core PRIVATE
    ppmd coverage_path_planning::yaml_cpp
)
add_executable(pdlog pdlog.cc)
target_link_libraries(pdlog PRIVATE pdlog_core)
target_compile_definitions(pdlog PRIVATE
    COVERAGE_PATH_PLANNING_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

add_executable(pdlog_test EXCLUDE_FROM_ALL ../../tests/pdlog_test.cc)
target_link_libraries(pdlog_test PRIVATE pdlog_core)

find_package(ZLIB REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_library(local_map_log STATIC local_map_log.cc)
target_link_libraries(local_map_log PUBLIC ZLIB::ZLIB
    PRIVATE nlohmann_json::nlohmann_json)
target_include_directories(local_map_log PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(local_map local_map_test.cc)
target_link_libraries(local_map PRIVATE
    pdlog_core
    local_map_log
    coverage_path_planning::rerun
)
# Blueprint 已随源码提供；构建及运行 C++ 示例不需要 Python。
configure_file(local_map_blueprint.rbl local_map_blueprint.rbl COPYONLY)
target_compile_definitions(local_map PRIVATE
    LOCAL_MAP_BLUEPRINT_PATH="${CMAKE_CURRENT_BINARY_DIR}/local_map_blueprint.rbl"
)

if(COVERAGE_BUILD_TESTS)
    add_executable(local_map_log_test ../../tests/local_map_log_test.cc)
    target_link_libraries(local_map_log_test PRIVATE local_map_log nlohmann_json::nlohmann_json)
    add_test(NAME local_map_log_test COMMAND local_map_log_test)
endif()
