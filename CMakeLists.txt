cmake_minimum_required(VERSION 3.4)

set(CMAKE_CXX_STANDARD 17)

project(mc-rtc-magnum LANGUAGES CXX VERSION 1.0.0)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/ext/magnum/modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/ext/magnum-integration/modules/" ${CMAKE_MODULE_PATH})

OPTION(MC_RTC_MAGNUM_BUILD_INTERFACE "Build the Magnum interface" ON)
option(MC_RTC_MAGNUM_BUILD_TICKER "Build the ticker" ON)

if(MC_RTC_MAGNUM_BUILD_INTERFACE)
  # Magnum options
  set(BUILD_STATIC ON CACHE BOOL "" FORCE)
  set(BUILD_PLUGINS_STATIC ON CACHE BOOL "" FORCE)
  set(WITH_GLFWAPPLICATION ON CACHE BOOL "" FORCE)
  set(WITH_ANYIMAGEIMPORTER ON CACHE BOOL "" FORCE)
  set(WITH_ANYSCENEIMPORTER ON CACHE BOOL "" FORCE)
  set(WITH_OBJIMPORTER ON CACHE BOOL "" FORCE)

  add_subdirectory(ext/glfw EXCLUDE_FROM_ALL)
  add_subdirectory(ext/corrade EXCLUDE_FROM_ALL)

  add_subdirectory(ext/magnum EXCLUDE_FROM_ALL)

  set(WITH_ASSIMPIMPORTER ON CACHE BOOL "" FORCE)
  set(WITH_STBIMAGEIMPORTER ON CACHE BOOL "" FORCE)
  add_subdirectory(ext/magnum-plugins EXCLUDE_FROM_ALL)

  set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ext/imgui)
  set(WITH_IMGUI ON CACHE BOOL "" FORCE)
  add_subdirectory(ext/magnum-integration EXCLUDE_FROM_ALL)

  find_package(Magnum REQUIRED GL)
  find_package(MagnumIntegration REQUIRED ImGui)

  if(CORRADE_TARGET_ANDROID)
      find_package(Magnum REQUIRED AndroidApplication)
  elseif(CORRADE_TARGET_EMSCRIPTEN)
      find_package(Magnum REQUIRED EmscriptenApplication)
  else()
      find_package(Magnum REQUIRED GlfwApplication)
  endif()
endif()

# mc_rtc
find_package(mc_rtc REQUIRED)

add_subdirectory(src)
