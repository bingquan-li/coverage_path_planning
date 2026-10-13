add_executable(rtrajectory_show trajectory_show.cc)
target_link_libraries(rtrajectory_show
    PRIVATE
    coverage_path_planning::rerun
)
add_executable(simu_show simu_show.cc)
target_link_libraries(simu_show
    PRIVATE
    coverage_path_planning::rerun
)
add_executable(reference_line reference_line.cc)
target_link_libraries(reference_line
    PRIVATE
    coverage_path_planning::rerun
)
add_executable(smooth_line smooth_line.cc)
target_link_libraries(smooth_line
    PRIVATE
    coverage_path_planning::rerun
    coverage_path_planning::planning
    coverage_path_planning::warnings
)
add_executable(min_square min_square.cc)
target_link_libraries(min_square
    PRIVATE
    coverage_path_planning::rerun
)
add_executable(later_error later_error.cc)
target_link_libraries(later_error
    PRIVATE
    coverage_path_planning::rerun
)