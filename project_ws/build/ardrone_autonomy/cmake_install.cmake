# Install script for directory: /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/vinayaka/project_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vinayaka/project_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/vinayaka/project_ws/install" TYPE PROGRAM FILES "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vinayaka/project_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/vinayaka/project_ws/install" TYPE PROGRAM FILES "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vinayaka/project_ws/install/setup.bash;/home/vinayaka/project_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/vinayaka/project_ws/install" TYPE FILE FILES
    "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/setup.bash"
    "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vinayaka/project_ws/install/setup.sh;/home/vinayaka/project_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/vinayaka/project_ws/install" TYPE FILE FILES
    "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/setup.sh"
    "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vinayaka/project_ws/install/setup.zsh;/home/vinayaka/project_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/vinayaka/project_ws/install" TYPE FILE FILES
    "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/setup.zsh"
    "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/vinayaka/project_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/vinayaka/project_ws/install" TYPE FILE FILES "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/srv" TYPE FILE FILES
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/srv/CamSelect.srv"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/srv/FlightAnim.srv"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/srv/LedAnim.srv"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/srv/RecordEnable.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/msg" TYPE FILE FILES
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/matrix33.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_adc_data_frame.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_altitude.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_demo.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_euler_angles.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_games.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_gyros_offsets.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_hdvideo_stream.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_kalman_pressure.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_magneto.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/Navdata.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_phys_measures.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_pressure_raw.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_pwm.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_raw_measures.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_rc_references.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_references.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_time.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_trackers_send.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_trims.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_video_stream.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_vision_detect.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_vision.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_vision_of.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_vision_perf.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_vision_raw.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_watchdog.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_wifi.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_wind_speed.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/navdata_zimmu_3000.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/vector21.msg"
    "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/msg/vector31.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/cmake" TYPE FILE FILES "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomy-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/include/ardrone_autonomy")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/share/roseus/ros/ardrone_autonomy")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/share/common-lisp/ros/ardrone_autonomy")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/share/gennodejs/ros/ardrone_autonomy")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/python2.7/dist-packages/ardrone_autonomy")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/python2.7/dist-packages/ardrone_autonomy")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomy.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/cmake" TYPE FILE FILES "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomy-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/cmake" TYPE FILE FILES
    "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomyConfig.cmake"
    "/home/vinayaka/project_ws/build/ardrone_autonomy/catkin_generated/installspace/ardrone_autonomyConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy" TYPE FILE FILES "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy" TYPE EXECUTABLE FILES "/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver"
         OLD_RPATH "/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone:/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ardrone_autonomy/ardrone_driver")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_autonomy/launch" TYPE DIRECTORY FILES "/home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/launch/" REGEX "/\\.git$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/vinayaka/project_ws/build/ardrone_autonomy/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/vinayaka/project_ws/build/ardrone_autonomy/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
