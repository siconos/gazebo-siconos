@REM Add paths to Windows. Run this script prior to running gzserver.exe
@REM and gzclient.ext on a Windows machine.
@REM Meant to be run from inside build/
@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

@REM Need absolute paths in order to run anything inside build/
@set gz_root_path=%~dp0
@set gz_build_path=%gz_root_path%\build
@pushd %~dp0\..
@set deps_path=%CD%
@popd
@echo - script path is %gz_root_path%
@echo - dependencies path is %deps_path%

@set HOME=%HOMEDRIVE%%HOMEPATH%

@set GAZEBO_MODEL_PATH=%gz_root_path%\models
@set GAZEBO_PLUGINS_PATH=%gz_build_path%\plugins
@set GAZEBO_RESOURCE_PATH=%gz_root_path%
@set OGRE_RESOURCE_PATH=%deps_path%\OGRE-SDK-1.9.0-vc120-x64-12.03.2016\bin\%build_type%

@set PATH=%deps_path%\boost_1_56_0\lib64-msvc-12.0;^
%deps_path%\FreeImage-vc12-x64-release-debug\x64\%build_type%\DLL;^
%gz_build_path%\deps\opende;^
%deps_path%\OGRE-SDK-1.9.0-vc120-x64-12.03.2016\bin\%build_type%;^
%deps_path%\sdformat\build\install\%build_type%\lib;^
%deps_path%\ign-math\build\install\%build_type%\lib;^
%deps_path%\ign-transport\build\install\%build_type%\lib;^
%deps_path%\tbb43_20141023oss\bin\intel64\vc12;^
%gz_build_path%\deps\libccd;^
%gz_build_path%\deps\opende\OPCODE;^
%gz_build_path%\deps\opende\ou;^
%deps_path%\zlib-1.2.8-vc12-x64-release-debug\contrib\vstudio\vc11\x64\ZlibDll%build_type%;^
%deps_path%\bzip2-1.0.6-vc12-x64-release-debug\%build_type%;^
%deps_path%\ZeroMQ 4.0.4\bin;^
C:\Qt\4.8.6\x64\msvc2013\bin;^
%PATH%
