cmake_minimum_required(VERSION 3.10)

set(PROJECT_TARGET quadmesh)
set(PROJECT_EXECUTABLE quadmeshGen)
set(PROJECT_SHORT_NAME qmsh)
string(TOUPPER ${PROJECT_TARGET} PROJECT_TARGET_UPPER)
string(TOUPPER ${PROJECT_SHORT_NAME} PROJECT_SHORT_NAME_UPPER)

project(quadmesh VERSION 0.1.0 LANGUAGES CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)
option(QMSH_ENABLE_TESTS "Enable unit tests (requires GTest to be installed)" YES)


# Include additional cmake packages
set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP TRUE)

include(InstallRequiredSystemLibraries)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)
#

# Declare dependencies
find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(cxxopts CONFIG REQUIRED)
find_package(RapidJSON CONFIG REQUIRED)

find_package(commons CONFIG REQUIRED)
find_package(geommodel CONFIG REQUIRED)
find_package(stepparse CONFIG REQUIRED)
#

# Find source files
set(PROJECT_RUNTIME_OUTPUT_DIR "${PROJECT_BINARY_DIR}/bin")
set(PROJECT_LIBRARY_OUTPUT_DIR "${PROJECT_BINARY_DIR}/lib")
set(PROJECT_ARCHIVE_OUTPUT_DIR "${PROJECT_LIBRARY_OUTPUT_DIR}")

file(GLOB_RECURSE
  PROJECT_SOURCES
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
)
file(GLOB
  PROJECT_EXECUTABLE_SOURCES
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
file(GLOB_RECURSE
  PROJECT_PUBLIC_HEADERS
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
  "${CMAKE_CURRENT_BINARY_DIR}/exports.hpp"
)
#


# Configure project library target
add_library(${PROJECT_TARGET})

target_sources(${PROJECT_TARGET}
  PRIVATE
    ${PROJECT_SOURCES}
    ${PROJECT_PUBLIC_HEADERS}
)
target_include_directories(${PROJECT_TARGET}
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR};${PROJECT_SOURCE_DIR}/src;${PROJECT_SOURCE_DIR}/include>"
  PRIVATE
    ${RAPIDJSON_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_TARGET}
  PUBLIC
    cxx_std_17
)
target_compile_definitions(${PROJECT_TARGET}
  PUBLIC
    $<BUILD_INTERFACE:_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS>
)
target_link_libraries(${PROJECT_TARGET}
  PUBLIC
    commons::commons
    geommodel::geommodel
)
set_target_properties(${PROJECT_TARGET}
  PROPERTIES
    DEBUG_POSTFIX d
    CXX_EXTENSIONS NO
    WINDOWS_EXPORT_ALL_SYMBOLS ${QMSH_ENABLE_TESTS}
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_RUNTIME_OUTPUT_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIBRARY_OUTPUT_DIR}
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_ARCHIVE_OUTPUT_DIR}
)

generate_export_header(${PROJECT_TARGET}
  BASE_NAME "${PROJECT_SHORT_NAME_UPPER}"
  EXPORT_FILE_NAME "exports.hpp"
  NO_DEPRECATED_MACRO_NAME "${PROJECT_SHORT_NAME_UPPER}_OMIT_DEPRECATED"
  DEFINE_NO_DEPRECATED
)
if (QMSH_ENABLE_TESTS)
  target_compile_definitions(${PROJECT_TARGET}
    PUBLIC
      "${PROJECT_SHORT_NAME_UPPER}_STATIC_DEFINE"
  )
endif()

add_library(${PROJECT_TARGET}::${PROJECT_TARGET} ALIAS ${PROJECT_TARGET})
#

# Configure project executable target
add_executable(${PROJECT_EXECUTABLE})
target_sources(${PROJECT_EXECUTABLE}
  PRIVATE
    ${PROJECT_EXECUTABLE_SOURCES}
)
target_include_directories(${PROJECT_EXECUTABLE}
  PRIVATE
    "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>"
    ${RAPIDJSON_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_EXECUTABLE}
  PRIVATE
    ${PROJECT_TARGET}
    stepparse::stepparse
    cxxopts::cxxopts
)
set_target_properties(${PROJECT_EXECUTABLE}
  PROPERTIES
    DEBUG_POSTFIX d
    CXX_EXTENSIONS NO
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_RUNTIME_OUTPUT_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIBRARY_OUTPUT_DIR}
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_ARCHIVE_OUTPUT_DIR}
)

add_executable(${PROJECT_TARGET}::${PROJECT_EXECUTABLE} ALIAS ${PROJECT_EXECUTABLE})
#

# Add tests
if (QMSH_ENABLE_TESTS)
  add_subdirectory(tests)
endif()
#


# Install targets
install(TARGETS ${PROJECT_TARGET} ${PROJECT_EXECUTABLE}
  EXPORT ${PROJECT_TARGET}_targets
  RUNTIME
    DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  INCLUDES
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)
install(
  DIRECTORY "${PROJECT_SOURCE_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)
install(
  FILES "${PROJECT_BINARY_DIR}/exports.hpp"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_SHORT_NAME}"
)

set(PROJECT_INSTALL_CONFIGDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_TARGET}")
install(EXPORT ${PROJECT_TARGET}_targets
  DESTINATION "${PROJECT_INSTALL_CONFIGDIR}"
  NAMESPACE "${PROJECT_TARGET}::"
  FILE "${PROJECT_TARGET}Targets.cmake"
)

get_target_property(PROJECT_TARGET_TYPE ${PROJECT_TARGET} TYPE)
if (WIN32 AND
    PROJECT_TARGET_TYPE STREQUAL "SHARED_LIBRARY" AND
    NOT (CMAKE_CXX_COMPILER_ID STREQUAL "GNU"))
  install(
    FILES "$<TARGET_PDB_FILE:${PROJECT_TARGET}>"
    DESTINATION "${CMAKE_INSTALL_BINDIR}"
    OPTIONAL
  )
endif()

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_TARGET}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_TARGET}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_TARGET}Config.cmake"
  INSTALL_DESTINATION "${PROJECT_INSTALL_CONFIGDIR}"
)

# Install the config, configversion and custom find modules
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_TARGET}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_TARGET}ConfigVersion.cmake"
  DESTINATION "${PROJECT_INSTALL_CONFIGDIR}"
)
#
