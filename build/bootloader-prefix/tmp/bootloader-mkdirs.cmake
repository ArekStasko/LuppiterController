# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/arekstasko/sources/Espressif-IDE/esp-idf-v5.3.1/components/bootloader/subproject"
  "/home/arekstasko/sources/repos/LuppiterController/build/bootloader"
  "/home/arekstasko/sources/repos/LuppiterController/build/bootloader-prefix"
  "/home/arekstasko/sources/repos/LuppiterController/build/bootloader-prefix/tmp"
  "/home/arekstasko/sources/repos/LuppiterController/build/bootloader-prefix/src/bootloader-stamp"
  "/home/arekstasko/sources/repos/LuppiterController/build/bootloader-prefix/src"
  "/home/arekstasko/sources/repos/LuppiterController/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/arekstasko/sources/repos/LuppiterController/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/arekstasko/sources/repos/LuppiterController/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
