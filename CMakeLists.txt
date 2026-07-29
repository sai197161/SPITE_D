cmake_minimum_required(VERSION 3.20)
project(spite_d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dual-mode build.
#
# ROS mode (Linux): built by colcon; ament_cmake is present. Builds message
#   interfaces, core libraries, and node executables.
# Core mode (macOS dev): plain CMake; ament is absent. Builds only the
#   ROS-free core libraries, offline tools, and unit tests so the pipeline
#   logic can be developed and tested without a ROS installation.
#     cmake -B build -DSPITE_D_OPEN_SPITE_DIR=/path/to/open-spite \
#           -DCMAKE_TOOLCHAIN_FILE=/path/to/open-spite/build/conan_toolchain.cmake
# ---------------------------------------------------------------------------
find_package(ament_cmake QUIET)

# open-spite is compiled from source as part of this build, so CMake needs
# its source directory -- unlike CGAL/OMPL/OpenCV, which are installed
# system-wide and found automatically. It is vendored as a git submodule
# at external/open-spite so no path has to be supplied; point
# SPITE_D_OPEN_SPITE_DIR elsewhere to build against a different checkout.
set(SPITE_D_OPEN_SPITE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/open-spite"
    CACHE PATH "Path to the open-spite source tree")

if(EXISTS "${SPITE_D_OPEN_SPITE_DIR}/CMakeLists.txt")
  # EXCLUDE_FROM_ALL keeps open-spite's examples and demos (which fetch OMPL
  # and VAMP) out of this build. Its library still builds because we link
  # against it, but its test binary would not -- while add_test() still
  # registers the test, so ctest would report it as "Not Run". Opt that one
  # target back in so the upstream suite actually runs.
  add_subdirectory(${SPITE_D_OPEN_SPITE_DIR} open_spite_build EXCLUDE_FROM_ALL)
  set(SPITE_D_HAVE_OPEN_SPITE TRUE)
  if(TARGET test_open_spite)
    set_property(TARGET test_open_spite PROPERTY EXCLUDE_FROM_ALL FALSE)
  endif()
else()
  # An empty external/open-spite means the repo was cloned without its
  # submodules; say so rather than failing deep inside add_subdirectory.
  message(WARNING
    "spite_d: open-spite not found at ${SPITE_D_OPEN_SPITE_DIR}\n"
    "  Run: git submodule update --init --recursive\n"
    "  Certification, span, and roadmap targets will be skipped.")
  set(SPITE_D_HAVE_OPEN_SPITE FALSE)
endif()

# ---------------------------------------------------------------------------
# Core libraries (ROS-free)
# ---------------------------------------------------------------------------
add_library(spite_d_core INTERFACE)
target_include_directories(spite_d_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(spite_d_trajectory
  src/trajectory/constant_velocity_predictor.cpp
)
target_link_libraries(spite_d_trajectory PUBLIC spite_d_core)

add_library(spite_d_planner
  src/planner/replanner.cpp
  src/planner/roadmap_io.cpp
)
target_link_libraries(spite_d_planner PUBLIC spite_d_core)

if(SPITE_D_HAVE_OPEN_SPITE)
  add_library(spite_d_validity
    src/spite/validity_server.cpp
  )
  target_link_libraries(spite_d_validity PUBLIC spite_d_core open_spite)
endif()

# Spans (the temporal contribution): pure span math always builds;
# the amortization pipeline needs the validity server.
add_library(spite_d_dynamic_map
  src/trajectory/span.cpp
)
target_link_libraries(spite_d_dynamic_map PUBLIC spite_d_core)
if(SPITE_D_HAVE_OPEN_SPITE)
  target_sources(spite_d_dynamic_map PRIVATE src/trajectory/span_pipeline.cpp)
  target_link_libraries(spite_d_dynamic_map
    PUBLIC spite_d_validity spite_d_trajectory)
endif()

# Perception core: UV-map detector + box tracker (map_manager port).
# OpenCV: brew on macOS, ROS-provided on Linux.
find_package(OpenCV QUIET COMPONENTS core imgproc)
if(OpenCV_FOUND)
  add_library(spite_d_perception
    src/perception/uv_detector.cpp
    src/perception/box_tracker.cpp
  )
  target_link_libraries(spite_d_perception
    PUBLIC spite_d_core opencv_core opencv_imgproc)
else()
  message(STATUS "spite_d: perception skipped (OpenCV not found)")
endif()

# ---------------------------------------------------------------------------
# Offline tools
# ---------------------------------------------------------------------------
# build_roadmap needs OMPL (brew on macOS, ROS-provided on Linux). The
# roadmap graph is extracted via ompl::base::PlannerData, so any OMPL
# build works regardless of its Boost version.
find_package(ompl CONFIG QUIET)
if(SPITE_D_HAVE_OPEN_SPITE AND ompl_FOUND)
  add_executable(build_roadmap tools/build_roadmap.cpp)
  target_link_libraries(build_roadmap
    PRIVATE spite_d_planner spite_d_validity ompl::ompl)

  # UR5 variant: joint-space PRM with per-link approximations. FK header
  # comes from open-spite's examples; needs Eigen.
  find_package(Eigen3 CONFIG QUIET)
  if(Eigen3_FOUND)
    add_executable(build_roadmap_ur5 tools/build_roadmap_ur5.cpp)
    target_include_directories(build_roadmap_ur5
      PRIVATE ${SPITE_D_OPEN_SPITE_DIR}/examples)
    target_link_libraries(build_roadmap_ur5
      PRIVATE spite_d_planner spite_d_validity ompl::ompl Eigen3::Eigen)
  else()
    message(STATUS "spite_d: build_roadmap_ur5 skipped (needs Eigen3)")
  endif()
else()
  message(STATUS "spite_d: build_roadmap tool skipped (needs open-spite + OMPL)")
endif()

# Terminal demo of the dynamic-validity loop; needs only open-spite.
if(SPITE_D_HAVE_OPEN_SPITE)
  add_executable(demo_dynamic tools/demo_dynamic.cpp)
  target_link_libraries(demo_dynamic
    PRIVATE spite_d_dynamic_map spite_d_planner spite_d_trajectory
            spite_d_validity)
endif()

# ---------------------------------------------------------------------------
# Tests (core mode and ROS mode)
# ---------------------------------------------------------------------------
enable_testing()
add_subdirectory(tests)

# ---------------------------------------------------------------------------
# ROS mode only: interfaces, nodes, install rules
# ---------------------------------------------------------------------------
if(ament_cmake_FOUND)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ObstacleState.msg"
    "msg/ObstacleArray.msg"
    "msg/PredictedTrajectory.msg"
    "msg/PredictedTrajectoryArray.msg"
    DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
  )

  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                                rosidl_typesupport_cpp)

  if(TARGET spite_d_perception)
    add_executable(perception_node src/nodes/perception_node.cpp)
    target_link_libraries(perception_node PRIVATE
      spite_d_perception
      "${cpp_typesupport_target}"
      rclcpp::rclcpp
      ${sensor_msgs_TARGETS})
    install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
  endif()

  add_executable(prediction_node src/nodes/prediction_node.cpp)
  target_link_libraries(prediction_node PRIVATE
    spite_d_trajectory
    "${cpp_typesupport_target}"
    rclcpp::rclcpp)
  install(TARGETS prediction_node DESTINATION lib/${PROJECT_NAME})

  if(SPITE_D_HAVE_OPEN_SPITE)
    add_executable(validity_node src/nodes/validity_node.cpp)
    target_link_libraries(validity_node PRIVATE
      spite_d_validity
      spite_d_planner
      "${cpp_typesupport_target}"
      rclcpp::rclcpp
      ${nav_msgs_TARGETS})
    install(TARGETS validity_node DESTINATION lib/${PROJECT_NAME})

    if(TARGET build_roadmap)
      install(TARGETS build_roadmap DESTINATION lib/${PROJECT_NAME})
    endif()
  endif()

  # ament sets BUILD_SHARED_LIBS=ON, so every library here builds as a .so
  # and must be installed where setup.bash's LD_LIBRARY_PATH points --
  # including open-spite's own targets, whose install rules are skipped
  # because we add it with EXCLUDE_FROM_ALL. Without this the executables
  # link fine but fail at startup with "cannot open shared object file".
  set(_spite_d_libs spite_d_trajectory spite_d_planner spite_d_dynamic_map)
  if(TARGET spite_d_perception)
    list(APPEND _spite_d_libs spite_d_perception)
  endif()
  if(SPITE_D_HAVE_OPEN_SPITE)
    list(APPEND _spite_d_libs spite_d_validity open_spite open_spite_gdiam)
  endif()
  install(TARGETS ${_spite_d_libs}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  # Only install asset directories that exist: empty ones (e.g. rviz/)
  # do not survive a git clone, and a missing directory here fails the
  # whole build for anyone cloning the repo.
  foreach(_asset_dir launch gazebo rviz config)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_asset_dir}")
      install(DIRECTORY ${_asset_dir} DESTINATION share/${PROJECT_NAME})
    endif()
  endforeach()
  install(DIRECTORY include/ DESTINATION include)

  ament_export_include_directories(include)
  ament_package()
endif()
