cmake_minimum_required(VERSION 3.16)
project(iiwa7_ik)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(AMENT_CMAKE "Build using ament_cmake for ros2" ON)

  # find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pinocchio
  Eigen3
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

if(AMENT_CMAKE)

  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)

  add_library(${PROJECT_NAME} INTERFACE)

  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  ament_target_dependencies(${PROJECT_NAME} INTERFACE ${THIS_PACKAGE_INCLUDE_DEPENDS})

  target_compile_definitions(${PROJECT_NAME} INTERFACE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  # INSTALL
  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
  )

  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME})
  ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_package()
else()

  add_library(${PROJECT_NAME} INTERFACE)

  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  add_subdirectory(tests)
  
endif()


