file(GLOB_RECURSE
  ALLTESTS_SOURCES
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/test_*.cpp"
)

if (ALLTESTS_SOURCES)
  enable_testing()
  include(GoogleTest)
  find_package(GTest MODULE REQUIRED)

  add_executable(AllTests)

  target_sources(AllTests
    PRIVATE
      ${ALLTESTS_SOURCES}
  )
  target_include_directories(AllTests
    PRIVATE
      "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR};${PROJECT_SOURCE_DIR}/src>"
  )
  target_link_libraries(AllTests
    PRIVATE
      ${PROJECT_TARGET}
      geommodel::geommodel
      stepparse::stepparse
      fmt::fmt
      GTest::GTest
      GTest::Main
  )
  set_target_properties(AllTests
    PROPERTIES
      DEBUG_POSTFIX d
      CXX_EXTENSIONS NO
      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_RUNTIME_OUTPUT_DIR}
      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIBRARY_OUTPUT_DIR}
      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_ARCHIVE_OUTPUT_DIR}
  )

  gtest_discover_tests(AllTests
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  )
endif()
