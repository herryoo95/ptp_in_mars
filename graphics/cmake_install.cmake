# Install script for directory: /home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmars_graphics.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmars_graphics.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmars_graphics.so"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/libmars_graphics.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmars_graphics.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmars_graphics.so")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmars_graphics.so")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmars_graphics.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mars/graphics" TYPE FILE FILES
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/GraphicsCamera.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/GraphicsManager.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/GraphicsViewer.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/GraphicsWidget.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/gui_helper_functions.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/HUD.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/PostDrawCallback.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/QtOsgMixGraphicsWidget.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/AxisPrimitive.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/CapsuleDrawObject.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/Clouds.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/CoordsPrimitive.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/CubeDrawObject.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/CylinderDrawObject.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/DrawObject.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/GridPrimitive.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/LoadDrawObject.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/OceanDrawObject.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/PlaneDrawObject.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/SphereDrawObject.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/TerrainDrawObject.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/VertexBufferTerrain.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/MarsVBOGeom.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/3d_objects/MultiResHeightMapRenderer.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/shader/bumpmapping.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/shader/pixellight.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/shader/shader-generator.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/shader/shader-function.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/shader/shader-types.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/shader/texgen.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/shadow/ShadowMap.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mars/graphics/2d_objects" TYPE FILE FILES
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/2d_objects/HUDElement.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/2d_objects/HUDNode.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/2d_objects/HUDLabel.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/2d_objects/HUDLines.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/2d_objects/HUDTerminal.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/2d_objects/HUDTexture.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/2d_objects/HUDOSGNode.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mars/graphics/wrapper" TYPE FILE FILES
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/wrapper/OSGDrawItem.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/wrapper/OSGHudElementStruct.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/wrapper/OSGLightStruct.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/wrapper/OSGMaterialStruct.h"
    "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/src/wrapper/OSGNodeStruct.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/mars_graphics.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mars/graphics" TYPE DIRECTORY FILES "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/resources")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/dfki.uni-bremen.de/yoo/development/learning/simulation/mars/graphics/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
