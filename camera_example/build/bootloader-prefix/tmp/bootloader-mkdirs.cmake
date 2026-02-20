# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/riftka221/esp/v5.5.2/esp-idf/components/bootloader/subproject")
  file(MAKE_DIRECTORY "/Users/riftka221/esp/v5.5.2/esp-idf/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "/Users/riftka221/code/upv/V3D/proyecto_posicionamiento/camera_example/build/bootloader"
  "/Users/riftka221/code/upv/V3D/proyecto_posicionamiento/camera_example/build/bootloader-prefix"
  "/Users/riftka221/code/upv/V3D/proyecto_posicionamiento/camera_example/build/bootloader-prefix/tmp"
  "/Users/riftka221/code/upv/V3D/proyecto_posicionamiento/camera_example/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/riftka221/code/upv/V3D/proyecto_posicionamiento/camera_example/build/bootloader-prefix/src"
  "/Users/riftka221/code/upv/V3D/proyecto_posicionamiento/camera_example/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/riftka221/code/upv/V3D/proyecto_posicionamiento/camera_example/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/riftka221/code/upv/V3D/proyecto_posicionamiento/camera_example/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
