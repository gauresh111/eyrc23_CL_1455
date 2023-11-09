cmake_minimum_required(VERSION 3.5)
project(mani_stack)

install(
  DIRECTORY config launch meshes rviz urdf test assets
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
endif()

install(PROGRAMS
  scripts/task1b.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY assets testingfolder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
