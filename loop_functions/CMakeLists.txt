include_directories(${CMAKE_SOURCE_DIR})


add_subdirectory(altruistic_behavior_loop_functions)

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
  add_subdirectory(altruistic_behavior_qt_user_functions)
endif(ARGOS_COMPILE_QTOPENGL)
