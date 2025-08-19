# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/Espressif/frameworks/components/bootloader/subproject")
  file(MAKE_DIRECTORY "D:/Espressif/frameworks/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "D:/File_dai_hoc_UIT/Thuc_Tap_FSOFT/INTERN_PROJECT/STM32_BOOTLOADER/ESP32_HOST/ESP32_HOST/build/bootloader"
  "D:/File_dai_hoc_UIT/Thuc_Tap_FSOFT/INTERN_PROJECT/STM32_BOOTLOADER/ESP32_HOST/ESP32_HOST/build/bootloader-prefix"
  "D:/File_dai_hoc_UIT/Thuc_Tap_FSOFT/INTERN_PROJECT/STM32_BOOTLOADER/ESP32_HOST/ESP32_HOST/build/bootloader-prefix/tmp"
  "D:/File_dai_hoc_UIT/Thuc_Tap_FSOFT/INTERN_PROJECT/STM32_BOOTLOADER/ESP32_HOST/ESP32_HOST/build/bootloader-prefix/src/bootloader-stamp"
  "D:/File_dai_hoc_UIT/Thuc_Tap_FSOFT/INTERN_PROJECT/STM32_BOOTLOADER/ESP32_HOST/ESP32_HOST/build/bootloader-prefix/src"
  "D:/File_dai_hoc_UIT/Thuc_Tap_FSOFT/INTERN_PROJECT/STM32_BOOTLOADER/ESP32_HOST/ESP32_HOST/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/File_dai_hoc_UIT/Thuc_Tap_FSOFT/INTERN_PROJECT/STM32_BOOTLOADER/ESP32_HOST/ESP32_HOST/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/File_dai_hoc_UIT/Thuc_Tap_FSOFT/INTERN_PROJECT/STM32_BOOTLOADER/ESP32_HOST/ESP32_HOST/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
