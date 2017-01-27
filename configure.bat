:: This file is a helper for gazebo configuration (cmake) on Windows
::
:: It is designed to work with a workspace detailed in the INSTALL_WIN32.md
:: file. Please follow the instructions there. The workspace layout should be:
::
:: gz-ws/
::   sdformat/
::   ...     # all dependencies detailed in the INSTALL file
::   #dep1/
::   #dep2/
::   ...
::   gazebo/
::
:: Usage: cd gz-ws/gazebo/build && ../configure <Release|Debug>
@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

@set BOOST_PATH=%cd%\..\..\boost_1_56_0
@set BOOST_LIBRARY_DIR=%BOOST_PATH%\lib64-msvc-12.0

@set PROTOBUF_PATH=%cd%\..\..\protobuf-2.6.0-win64-vc12

@set CURL_PATH=%cd%\..\..\libcurl-vc12-x64-release-debug-static-ipv6-sspi-winssl\%build_type%
@set CURL_INCLUDE_DIR=%CURL_PATH%\include
@set CURL_LIBRARY_DIR=%CURL_PATH%\lib
set CURL_LIBRARY_NAME=libcurl_a
@if "%build_type%"=="Debug" set CURL_LIBRARY_NAME=libcurl_a_debug

@set FREEIMAGE_PATH=%cd%\..\..\FreeImage\Dist
@set FREEIMAGE_LIBRARY_DIR=%FREEIMAGE_PATH%\x64
@set FREEIMAGE_INCLUDE_DIR=%FREEIMAGE_PATH%\x64

@set SDFORMAT_PATH=%cd%\..\..\sdformat\build\install\%build_type%
@set IGNITION-MATH_PATH=%cd%\..\..\ign-math\build\install\%build_type%
@set IGNITION-MSGS_PATH=%cd%\..\..\ign-msgs\build\install\%build_type%
@set IGNITION-TRANSPORT_PATH=%cd%\..\..\ign-transport\build\install\%build_type%
@set IGNITION-TRANSPORT_CMAKE_PREFIX_PATH=%IGNITION-TRANSPORT_PATH%\lib\cmake\ignition-transport3

@set TBB_PATH=%cd%\..\..\tbb43_20141023oss
@set TBB_LIBRARY_DIR=%TBB_PATH%\lib\intel64\vc12
@set TBB_INCLUDEDIR=%TBB_PATH%\include

@set QWT_PATH=%cd%\..\..\qwt_6.1.0~github_zalf_lsa
@set QWT_LIBRARY_DIR=%QWT_PATH%\%build_type%\qwt-6.1.0-vc12-x64
@set QWT_INCLUDEDIR=%QWT_PATH%\include

@set OGRE_VERSION=1.9.0
@set OGRE_PATH=%cd%\..\..\OGRE-SDK-1.9.0-vc120-x64-12.03.2016
@set OGRE_INCLUDE_DIR=%OGRE_PATH%\include;%OGRE_PATH%\include\OGRE;%OGRE_PATH%\include\OGRE\RTShaderSystem;%OGRE_PATH%\include\OGRE\Terrain;%OGRE_PATH%\include\OGRE\Paging
@set OGRE_LIBRARY_DIR=%OGRE_PATH%\lib\%build_type%
@set OGRE_PLUGIN_DIR=%OGRE_LIBRARY_DIR%\opt
set OGRE_LIB_SUFFIX=.lib
@if "%build_type%"=="Debug" set OGRE_LIB_SUFFIX=_d.lib
@set OGRE_LIBS=%OGRE_LIBRARY_DIR%\OgreMain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreOverlay%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreRTShaderSystem%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreTerrain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgrePaging%OGRE_LIB_SUFFIX%
@set OGRE_LIBS=%OGRE_LIBRARY_DIR%\OgreMain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreRTShaderSystem%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreTerrain%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgrePaging%OGRE_LIB_SUFFIX%;%OGRE_LIBRARY_DIR%\OgreOverlay%OGRE_LIB_SUFFIX%

@set DLFCN_WIN32_PATH=%cd%\..\..\dlfcn-win32-vc12-x64-release-debug\build\install\%build_type%
@set DLFCN_WIN32_LIBRARY_DIR=%DLFCN_WIN32_PATH%\lib
@set DLFCN_WIN32_INCLUDE_DIR=%DLFCN_WIN32_PATH%\include

@set QT5_PATH=C:\Qt\5.7\\msvc2013_64
@set QT5_BIN_DIR=%QT5_PATH%\bin

@set INCLUDE=%FREEIMAGE_INCLUDE_DIR%;%TBB_INCLUDEDIR%;%DLFCN_WIN32_INCLUDE_DIR%;%INCLUDE%
@set LIB=%FREEIMAGE_LIBRARY_DIR%;%BOOST_LIBRARY_DIR%;%TBB_LIBRARY_DIR%;%DLFCN_WIN32_LIBRARY_DIR%;%LIB%
@set PATH=%QT5_BIN_DIR%;%PATH%

cmake -Wno-dev -G "NMake Makefiles"^
    -DCMAKE_PREFIX_PATH="%SDFORMAT_PATH%;%IGNITION-MATH_PATH%;%IGNITION-MSGS_PATH%;%IGNITION-TRANSPORT_CMAKE_PREFIX_PATH%"^
    -DUSE_EXTERNAL_TINYXML:BOOL=False^
    -DUSE_EXTERNAL_TINYXML2:BOOL=False^
    -DFREEIMAGE_RUNS=1^
    -DPROTOBUF_SRC_ROOT_FOLDER="%PROTOBUF_PATH%"^
    -DBOOST_ROOT:STRING="%BOOST_PATH%"^
    -DBOOST_LIBRARYDIR:STRING="%BOOST_LIBRARY_DIR%"^
    -DOGRE_FOUND=1^
    -DOGRE-RTShaderSystem_FOUND=1^
    -DOGRE-Terrain_FOUND=1^
    -DOGRE-Overlay_FOUND=1^
    -DOGRE_VERSION=%OGRE_VERSION%^
    -DOGRE_PLUGINDIR="%OGRE_PLUGIN_DIR%"^
    -DOGRE_INCLUDE_DIRS="%OGRE_INCLUDE_DIR%"^
    -DOGRE_LIBRARIES="%OGRE_LIBS%"^
    -DQWT_WIN_INCLUDE_DIR="%QWT_INCLUDEDIR%"^
    -DQWT_WIN_LIBRARY_DIR="%QWT_LIBRARY_DIR%"^
    -DCURL_FOUND=1^
    -DCURL_INCLUDEDIR="%CURL_INCLUDE_DIR%"^
    -DCURL_LIBDIR="%CURL_LIBRARY_DIR%"^
    -DCURL_LIBRARIES="%CURL_LIBRARY_NAME%"^
    -DTBB_FOUND=1^
    -DTBB_INCLUDEDIR="%TBB_INCLUDEDIR%"^
    -DTBB_LIBRARY_DIR="%TBB_LIBRARY_DIR%"^
    -DCMAKE_INSTALL_PREFIX="install\%build_type%"^
    -DCMAKE_BUILD_TYPE="%build_type%"^
    ..
