## Gazebo 9

## Gazebo 9.x.x (2018-xx-xx)

1. Remove Gazebo 8 deprecations
    * [Pull request #2605](https://bitbucket.org/osrf/gazebo/pull-request/2605)
    * [Pull request #2607](https://bitbucket.org/osrf/gazebo/pull-request/2607)
    * [Pull request #2603](https://bitbucket.org/osrf/gazebo/pull-request/2603)
    * [Pull request #2604](https://bitbucket.org/osrf/gazebo/pull-request/2604)
    * [Pull request #2627](https://bitbucket.org/osrf/gazebo/pull-request/2627)

1. Bullet: sending feedback on contact points on depth 0 as well
    * [Pull request #2630](https://bitbucket.org/osrf/gazebo/pull-requests/2630/)

1. Deprecate functions to set linear/angular acceleration 
    * [Pull request #2622](https://bitbucket.org/osrf/gazebo/pull-request/2622)

## Gazebo 8

## Gazebo 8.x.x (2017-xx-xx)

1. Fix loading gui plugins and OSX framerate issue
    * [Pull request #2631](https://bitbucket.org/osrf/gazebo/pull-request/2631)
    * [Issue #1311](https://bitbucket.org/osrf/gazebo/issues/1311)
    * [Issue #2133](https://bitbucket.org/osrf/gazebo/issues/2133)

## Gazebo 8.0.0 (2017-01-25)

1. Depend on ignition math3
    * [Pull request #2588](https://bitbucket.org/osrf/gazebo/pull-request/2588)

1. Use ignition math with ServerFixture
    * [Pull request #2552](https://bitbucket.org/osrf/gazebo/pull-request/2552)

1. Changed the type of `FrictionPyramid::direction1` from `gazebo::math::Vector3` to `ignition::math::Vector3d`.
    * [Pull request #2548](https://bitbucket.org/osrf/gazebo/pull-request/2548)

1. Added igntition::transport interfaces to header files
    * [Pull request #2559](https://bitbucket.org/osrf/gazebo/pull-request/2559)

1. Added ignition transport dependency, and output camera sensor images on
   an ignition transport topic.
    * [Pull request #2544](https://bitbucket.org/osrf/gazebo/pull-request/2544)

1. Fix restoring submesh material transparency
    * [Pull request #2536](https://bitbucket.org/osrf/gazebo/pull-request/2536)

1. Updated `gz_log` tool to use `ignition::math`.
    * [Pull request #2532](https://bitbucket.org/osrf/gazebo/pull-request/2532)

1. Updated the following rendering classes to use `ignition::math`:
   `FPSViewController`, `JointVisual`, `OculusCamera`, `OrbitViewController`,
   `OrthoViewController`, `Projector`, `UserCamera`, `ViewController`.
    * [Pull request #2551](https://bitbucket.org/osrf/gazebo/pull-request/2551)

1. Update examples to use ign-math.
    * [Pull request #2539](https://bitbucket.org/osrf/gazebo/pull-request/2539)

1. Update plugins to use ign-math.
    * [Pull request #2531](https://bitbucket.org/osrf/gazebo/pull-request/2531)
    * [Pull request #2534](https://bitbucket.org/osrf/gazebo/pull-request/2534)
    * [Pull request #2538](https://bitbucket.org/osrf/gazebo/pull-request/2538)

1. Use ignition math with `rendering/Distortion` and update function names.
    * [Pull request #2529](https://bitbucket.org/osrf/gazebo/pull-request/2529)

1. Updated COMVisual class to use `ignition::math`.
    * [Pull request #2528](https://bitbucket.org/osrf/gazebo/pull-request/2528)

1. Deprecate angle API from physics::Joint, in favor of using doubles
    * [Pull request #2568](https://bitbucket.org/osrf/gazebo/pull-request/2568)
    * [Issue #553](https://bitbucket.org/osrf/gazebo/issues/553)
    * [Issue #1108](https://bitbucket.org/osrf/gazebo/issues/1108)

1. PIMPL-ize `gazebo/physics/Gripper` and use ignition-math.
    * [Pull request #2523](https://bitbucket.org/osrf/gazebo/pull-request/2523)

1. Added VisualMarkers to the rendering engine. Visual markers support
   programmatic rendering of various shapes in a scene.
    * [Pull request 2541](https://bitbucket.org/osrf/gazebo/pull-request/2541)

1. Support version 5 of the DART Physics Engine.
    * [Pull request #2459](https://bitbucket.org/osrf/gazebo/pull-request/2459)

1. UserCamera overrides `Camera::Render` to reduce CPU usage.
    * [Pull request 2480](https://bitbucket.org/osrf/gazebo/pull-request/2480)

1. Static links no longer subscribe to wrench topics.
    * [Pull request #2452]((https://bitbucket.org/osrf/gazebo/pull-request/2452)

1. Add Gazebo math helper functions to convert to and from Ignition Math
   objects.
    * [Pull request #2461](https://bitbucket.org/osrf/gazebo/pull-request/2461)

1. Add video recording of user camera. This change added an optional
   dependency on libavdevice>=56.4.100 for linux systems. When installed,
   libavdevice will allow a user to stream a simulated camera to a video4linux2
   loopback device.
    * [Pull request #2443](https://bitbucket.org/osrf/gazebo/pull-request/2443)

1. Removed deprecations
    * [Pull request #2427]((https://bitbucket.org/osrf/gazebo/pull-request/2427)

1. Include basic support for GNU Precompiled Headers to reduce compile time
    * [Pull request #2268](https://bitbucket.org/osrf/gazebo/pull-request/2268)

1. Plotting utility
    * [Pull request #2348](https://bitbucket.org/osrf/gazebo/pull-request/2348)
    * [Pull request #2325](https://bitbucket.org/osrf/gazebo/pull-request/2325)
    * [Pull request #2382](https://bitbucket.org/osrf/gazebo/pull-request/2382)
    * [Pull request #2448](https://bitbucket.org/osrf/gazebo/pull-request/2448)

1. Renamed `gazebo/gui/SaveDialog` to `gazebo/gui/SaveEntityDialog`. A new
   `SaveDialog` class will be added in a future pull request. The migration
   guide will be updated with that pull request.
    * [Pull request #2384](https://bitbucket.org/osrf/gazebo/pull-request/2384)

1. Add FiducialCameraPlugin for Camera Sensors
    * [Pull request #2350](https://bitbucket.org/osrf/gazebo/pull-request/2350)

1. Fix Road2d vertices and shadows
    * [Pull request #2362](https://bitbucket.org/osrf/gazebo/pull-request/2362)

1. Rearrange GLWidget::OnMouseMove so that the more common use cases it
   fewer if statements. Use std::thread in place of boost in OculusWindow.
   Pragma statements to prevent warnings. Prevent variable hiding in
   WallSegmentItem.
    * [Pull request #2376](https://bitbucket.org/osrf/gazebo/pull-request/2376)

1. Use single pixel selection buffer for mouse picking
    * [Pull request #2335](https://bitbucket.org/osrf/gazebo/pull-request/2335)

1. Refactor Visual classes
  * [Pull request #2331](https://bitbucket.org/osrf/gazebo/pull-requests/2331)

1. Windows plugins (with .dll extension) now accepted
    * [Pull request #2311](https://bitbucket.org/osrf/gazebo/pull-requests/2311)
    * Writing libMyPlugin.so in the sdf file will look for MyPlugin.dll on windows.

1. Add Introspection Manager and Client util
    * [Pull request #2304](https://bitbucket.org/osrf/gazebo/pull-request/2304)

1. Refactor Event classes and improve memory management.
    * [Pull request #2277](https://bitbucket.org/osrf/gazebo/pull-request/2277)
    * [Pull request #2317](https://bitbucket.org/osrf/gazebo/pull-request/2317)
    * [Pull request #2329](https://bitbucket.org/osrf/gazebo/pull-request/2329)
    * [gazebo_design Pull request #33](https://bitbucket.org/osrf/gazebo_design/pull-requests/33)

1. Remove EntityMakerPrivate and move its members to derived classes
    * [Pull request #2310](https://bitbucket.org/osrf/gazebo/pull-request/2310)

1. Conversion between ign-msgs and sdf, for plugin
    * [Pull request #2403](https://bitbucket.org/osrf/gazebo/pull-request/2403)

1. Change NULL to nullptr.
    * [Pull request #2294](https://bitbucket.org/osrf/gazebo/pull-request/2294)
    * [Pull request #2297](https://bitbucket.org/osrf/gazebo/pull-request/2297)
    * [Pull request #2298](https://bitbucket.org/osrf/gazebo/pull-request/2298)
    * [Pull request #2302](https://bitbucket.org/osrf/gazebo/pull-request/2302)
    * [Pull request #2295](https://bitbucket.org/osrf/gazebo/pull-request/2295)
    * [Pull request #2300](https://bitbucket.org/osrf/gazebo/pull-request/2300)

1. Fix memory and other issues found from running Coverity.
    * A contribution from Olivier Crave
    * [Pull request #2241](https://bitbucket.org/osrf/gazebo/pull-request/2241)
    * [Pull request #2242](https://bitbucket.org/osrf/gazebo/pull-request/2242)
    * [Pull request #2243](https://bitbucket.org/osrf/gazebo/pull-request/2243)
    * [Pull request #2244](https://bitbucket.org/osrf/gazebo/pull-request/2244)
    * [Pull request #2245](https://bitbucket.org/osrf/gazebo/pull-request/2245)

1. Deprecate gazebo::math
    * [Pull request #2594](https://bitbucket.org/osrf/gazebo/pull-request/2594)
    * [Pull request #2513](https://bitbucket.org/osrf/gazebo/pull-request/2513)
    * [Pull request #2586](https://bitbucket.org/osrf/gazebo/pull-request/2586)
    * [Pull request #2326](https://bitbucket.org/osrf/gazebo/pull-request/2326)
    * [Pull request #2579](https://bitbucket.org/osrf/gazebo/pull-request/2579)
    * [Pull request #2574](https://bitbucket.org/osrf/gazebo/pull-request/2574)
    * [Pull request #2426](https://bitbucket.org/osrf/gazebo/pull-request/2426)
    * [Pull request #2567](https://bitbucket.org/osrf/gazebo/pull-request/2567)
    * [Pull request #2355](https://bitbucket.org/osrf/gazebo/pull-request/2355)
    * [Pull request #2407](https://bitbucket.org/osrf/gazebo/pull-request/2407)
    * [Pull request #2564](https://bitbucket.org/osrf/gazebo/pull-request/2564)
    * [Pull request #2591](https://bitbucket.org/osrf/gazebo/pull-request/2591)
    * [Pull request #2425](https://bitbucket.org/osrf/gazebo/pull-request/2425)
    * [Pull request #2570](https://bitbucket.org/osrf/gazebo/pull-request/2570)
    * [Pull request #2436](https://bitbucket.org/osrf/gazebo/pull-request/2436)
    * [Pull request #2556](https://bitbucket.org/osrf/gazebo/pull-request/2556)
    * [Pull request #2472](https://bitbucket.org/osrf/gazebo/pull-request/2472)
    * [Pull request #2505](https://bitbucket.org/osrf/gazebo/pull-request/2505)
    * [Pull request #2583](https://bitbucket.org/osrf/gazebo/pull-request/2583)
    * [Pull request #2514](https://bitbucket.org/osrf/gazebo/pull-request/2514)
    * [Pull request #2522](https://bitbucket.org/osrf/gazebo/pull-request/2522)
    * [Pull request #2565](https://bitbucket.org/osrf/gazebo/pull-request/2565)
    * [Pull request #2525](https://bitbucket.org/osrf/gazebo/pull-request/2525)
    * [Pull request #2533](https://bitbucket.org/osrf/gazebo/pull-request/2533)
    * [Pull request #2543](https://bitbucket.org/osrf/gazebo/pull-request/2543)
    * [Pull request #2549](https://bitbucket.org/osrf/gazebo/pull-request/2549)
    * [Pull request #2554](https://bitbucket.org/osrf/gazebo/pull-request/2554)
    * [Pull request #2560](https://bitbucket.org/osrf/gazebo/pull-request/2560)
    * [Pull request #2585](https://bitbucket.org/osrf/gazebo/pull-request/2585)
    * [Pull request #2575](https://bitbucket.org/osrf/gazebo/pull-request/2575)
    * [Pull request #2563](https://bitbucket.org/osrf/gazebo/pull-request/2563)
    * [Pull request #2573](https://bitbucket.org/osrf/gazebo/pull-request/2573)
    * [Pull request #2577](https://bitbucket.org/osrf/gazebo/pull-request/2577)
    * [Pull request #2581](https://bitbucket.org/osrf/gazebo/pull-request/2581)
    * [Pull request #2566](https://bitbucket.org/osrf/gazebo/pull-request/2566)
    * [Pull request #2578](https://bitbucket.org/osrf/gazebo/pull-request/2578)

1. Add Wind support
    * [Pull request #1985](https://bitbucket.org/osrf/gazebo/pull-request/1985)
    * A contribution from Olivier Crave

1. Add const accessors to uri path and query
    * [Pull request #2400](https://bitbucket.org/osrf/gazebo/pull-request/2400)

1. Server generates unique model names in case of overlap, and added allow_renaming field to factory message.
    * [Pull request 2301](https://bitbucket.org/osrf/gazebo/pull-request/2301)
    * [Issue 510](https://bitbucket.org/osrf/gazebo/issues/510)

1. Adds an output option to gz log that allows the tool to filter a log file and write to a new log file.
    * [Pull request #2149](https://bitbucket.org/osrf/gazebo/pull-request/2149)

1. Add common::URI class
    * [Pull request #2275](https://bitbucket.org/osrf/gazebo/pull-request/2275)

1. Update Actor animations by faciliting skeleton visualization, control via a plugin. Also resolves issue #1785.
    * [Pull request #2219](https://bitbucket.org/osrf/gazebo/pull-request/2219)

1. Generalize actors to work even if not all elements are specified
    * [Pull request #2360](https://bitbucket.org/osrf/gazebo/pull-request/2360)

1. PIMPLize rendering/Grid
    * [Pull request 2330](https://bitbucket.org/osrf/gazebo/pull-request/2330)

1. Use only Gazebo's internal version of tinyxml2. The version of tinyxml2 distributed with Ubuntu fails when parsing large log files.
    * [Pull request #2146](https://bitbucket.org/osrf/gazebo/pull-request/2146)

1. Moved gazebo ODE includes to have correct include path
    * [Pull request #2186](https://bitbucket.org/osrf/gazebo/pull-request/2186)

1. Atmosphere model
    * [Pull request #1989](https://bitbucket.org/osrf/gazebo/pull-request/1989)

1. Added static camera when following a model.
    * [Pull request #1980](https://bitbucket.org/osrf/gazebo/pull-request/1980)
    * A contribution from Oliver Crave

1. Get plugin info with Ignition transport service
    * [Pull request #2420](https://bitbucket.org/osrf/gazebo/pull-request/2420)

1. Support conversions between SDF and protobuf for more sensors.
    * [Pull request #2118](https://bitbucket.org/osrf/gazebo/pull-request/2118)

1. Fix ODE Ray-Cylinder collision, and added ability to instantiate stand alone MultiRayShapes.
    * [Pull request #2122](https://bitbucket.org/osrf/gazebo/pull-request/2122)

1. Update depth camera sensor to publish depth data over a topic.
    * [Pull request #2112](https://bitbucket.org/osrf/gazebo/pull-request/2112)

1. Add color picker to config widget and fix visual and collision duplication.
    * [Pull request #2381](https://bitbucket.org/osrf/gazebo/pull-request/2381)

1. Model editor updates

    1. Undo / redo inserting and deleting links
        * [Pull request #2151](https://bitbucket.org/osrf/gazebo/pull-request/2151)

    1. Undo / redo inserting and deleting nested models
        * [Pull request #2229](https://bitbucket.org/osrf/gazebo/pull-request/2229)

    1. Undo insert / delete joints
        * [Pull request #2266](https://bitbucket.org/osrf/gazebo/pull-request/2266)

    1. Undo insert / delete model plugins
        * [Pull request #2334](https://bitbucket.org/osrf/gazebo/pull-request/2334)

    1. Undo translate, rotate, snap and align links and nested models
        * [Pull request #2314](https://bitbucket.org/osrf/gazebo/pull-request/2314)

    1. Undo scale links
        * [Pull request #2368](https://bitbucket.org/osrf/gazebo/pull-request/2368)

1. Google Summer of Code Graphical interface for inserting plugins during simulation.

    1. Display attached model plugins in the world tab / Add subheaders for model links, joints and plugins
        * [Pull request #2323](https://bitbucket.org/osrf/gazebo/pull-request/2323)
        * [Issue #1698](https://bitbucket.org/osrf/gazebo/issues/1698)

## Gazebo 7

## Gazebo 7.X.X (201X-XX-XX)

1. Support Heightmap LOD
    * [Pull request 2636](https://bitbucket.org/osrf/gazebo/pull-request/2636)

## Gazebo 7.5.0 (2017-01-11)

1. Remove qt4 webkit in gazebo7 (used for HotkeyDialog).
    * [Pull request 2584](https://bitbucket.org/osrf/gazebo/pull-request/2584)

1. Support configuring heightmap sampling level
    * [Pull request 2519](https://bitbucket.org/osrf/gazebo/pull-request/2519)

1. Fix `model.config` dependency support, and add ability to reference
   textures using a URI.
    * [Pull request 2517](https://bitbucket.org/osrf/gazebo/pull-request/2517)

1. Fix DEM heightmap size, collision, scale
    * [Pull request 2477](https://bitbucket.org/osrf/gazebo/pull-request/2477)

1. Create ode_quiet parameter to silence solver messages
    * [Pull request 2512](https://bitbucket.org/osrf/gazebo/pull-request/2512)

1. Update QT render loop to throttle based on UserCamera::RenderRate.
    * [Pull request 2476](https://bitbucket.org/osrf/gazebo/pull-request/2476)
    * [Issue 1560](https://bitbucket.org/osrf/gazebo/issues/1560)

1. Generate visualization on demand, instead of on load. This helps to
   reduce load time.
    * [Pull request 2457](https://bitbucket.org/osrf/gazebo/pull-request/2457)

1. Added a plugin to teleoperate joints in a model with the keyboard.
    * [Pull request 2490](https://bitbucket.org/osrf/gazebo/pull-request/2490)

1. Add GUI items to change the user camera clip distance
    * [Pull request 2470](https://bitbucket.org/osrf/gazebo/pull-request/2470)
    * [Issue 2064](https://bitbucket.org/osrf/gazebo/issues/2064)

1. Support custom material scripts for heightmaps
    * [Pull request 2473](https://bitbucket.org/osrf/gazebo/pull-request/2473)

1. Sim events plugin accepts custom topics
    * [Pull request 2535](https://bitbucket.org/osrf/gazebo/pull-request/2535)

1. Model Editor: Show / hide collisions
    * [Pull request 2503](https://bitbucket.org/osrf/gazebo/pull-request/2503)

1. Model Editor: Show / hide visuals
    * [Pull request 2516](https://bitbucket.org/osrf/gazebo/pull-request/2516)

1. Model Editor: Show / hide link frames
    * [Pull request 2521](https://bitbucket.org/osrf/gazebo/pull-request/2521)

## Gazebo 7.4.0 (2016-10-11)

1. Add test for HarnessPlugin, reduce likelihood of race condition
    * [Pull request 2431](https://bitbucket.org/osrf/gazebo/pull-request/2431)
    * [Issue 2034](https://bitbucket.org/osrf/gazebo/issues/2034)

1. Add `syntax = proto2` in proto files to fix some protobuf3 warnings
    * [Pull request 2456](https://bitbucket.org/osrf/gazebo/pull-request/2456)

1. Add support for loading wavefront obj mesh files
    * [Pull request 2454](https://bitbucket.org/osrf/gazebo/pull-request/2454)

1. Added filesystem operations to the common library. Additions include
   `cwd`, `exists`, `isDirectory`, `isFile`, `copyFile`, and `moveFile`.
    * [Pull request 2417](https://bitbucket.org/osrf/gazebo/pull-request/2417)

1. Fix loading collada files with multiple texture coordinates.
    * [Pull request 2413](https://bitbucket.org/osrf/gazebo/pull-request/2413)

1. Added visualization of minimum range to laservisual.
    * [Pull request 2412](https://bitbucket.org/osrf/gazebo/pull-request/2412)
    * [Issue 2018](https://bitbucket.org/osrf/gazebo/issues/2018)

1. Use precision 2 for FPS display in TimePanel
    * [Pull request 2405](https://bitbucket.org/osrf/gazebo/pull-request/2405)

1. Switch ImuSensor::worldToReference transform from Pose to Quaternion
    * [Pull request 2410](https://bitbucket.org/osrf/gazebo/pull-request/2410)
    * [Issue 1959](https://bitbucket.org/osrf/gazebo/issues/1959)

1. Include Boost_LIBRARIES  in the linking of gazebo_physics
    * [Pull request 2402](https://bitbucket.org/osrf/gazebo/pull-request/2402)

1. Backported KeyboardGUIPlugin and msgs::Any
    * [Pull request 2416](https://bitbucket.org/osrf/gazebo/pull-request/2416)

1. Use XML_SUCCESS enum instead of XML_NO_ERROR, which has been deleted in tinyxml2 4.0
    * [Pull request 2397](https://bitbucket.org/osrf/gazebo/pull-request/2397)

1. Ignore ffmpeg deprecation warnings to clean up CI since they are noted in #2002
    * [Pull request 2388](https://bitbucket.org/osrf/gazebo/pull-request/2388)

1. Added a visual blinking plugin
    * [Pull request 2394](https://bitbucket.org/osrf/gazebo/pull-request/2394)

1. Fix InertiaVisual for non-diagonal inertia matrices
    * [Pull request 2354](https://bitbucket.org/osrf/gazebo/pull-request/2354)

## Gazebo 7.3.1 (2016-07-13)

1. Fix homebrew test failure of UNIT_ApplyWrenchDialog_TEST
    * [Pull request 2393](https://bitbucket.org/osrf/gazebo/pull-request/2393)

1. Fix MainWindow crash when window is minimized and maximized
    * [Pull request 2392](https://bitbucket.org/osrf/gazebo/pull-request/2392)
    * [Issue 2003](https://bitbucket.org/osrf/gazebo/issues/2003)

## Gazebo 7.3.0 (2016-07-12)

1. Fix selecting ApplyWrenchVisual's force torque visuals
    * [Pull request 2377](https://bitbucket.org/osrf/gazebo/pull-request/2377)
    * [Issue 1999](https://bitbucket.org/osrf/gazebo/issues/1999)

1. Use ignition math in gazebo::msgs
    * [Pull request 2389](https://bitbucket.org/osrf/gazebo/pull-request/2389)

1. Parse command-line options for GUI plugins in Server to fix parsing of
   positional argument for world file.
   This fixes command-line parsing for `gazebo -g gui_plugin.so`.
    * [Pull request 2387](https://bitbucket.org/osrf/gazebo/pull-request/2387)

1. Added a harness plugin that supports lowering a model at a controlled rate
    * [Pull request 2346](https://bitbucket.org/osrf/gazebo/pull-request/2346)

1. Fix ogre log test on xenial+nvidia
    * [Pull request 2374](https://bitbucket.org/osrf/gazebo/pull-request/2374)

1. Redirect QT messages to Gazebo's console message handling system.
    * [Pull request 2375](https://bitbucket.org/osrf/gazebo/pull-request/2375)

1. Fix buoyancy plugin when multiple link tags are used within the plugin
    * [Pull request 2369](https://bitbucket.org/osrf/gazebo/pull-request/2369)

1. Remove contact filters with names that contain `::`
    * [Pull request 2363](https://bitbucket.org/osrf/gazebo/pull-request/2363)
    * [Issue 1805](https://bitbucket.org/osrf/gazebo/issues/1805)

1. Fix Model Manipulator switching between local and global frames
    * [Pull request 2361](https://bitbucket.org/osrf/gazebo/pull-request/2361)

1. Remove duplicate code from cmake config file caused by bad merge
    * [Pull request 2347](https://bitbucket.org/osrf/gazebo/pull-request/2347)

1. Properly cleanup pointers when destroying a world with joints.
    * [Pull request 2309](https://bitbucket.org/osrf/gazebo/pull-request/2309)

1. Fix right click view options after deleting and respawning a model.
    * [Pull request 2349](https://bitbucket.org/osrf/gazebo/pull-request/2349)
    * [Issue 1985](https://bitbucket.org/osrf/gazebo/issues/1985)

1. Implement missing function: LogicalCamera::Topic()
    * [Pull request 2343](https://bitbucket.org/osrf/gazebo/pull-request/2343)
    * [Issue 1980](https://bitbucket.org/osrf/gazebo/issues/1980)

## Gazebo 7.2.0 (2016-06-13)

1. Backport single pixel selection buffer for mouse picking
    * [Pull request 2338](https://bitbucket.org/osrf/gazebo/pull-request/2338)

1. Prevent mouse pan and orbit from deselecting entities in model editor
    * [Pull request 2333](https://bitbucket.org/osrf/gazebo/pull-request/2333)

1. Handle model manipulation tool RTS shortcuts in keyPress
    * [Pull request 2312](https://bitbucket.org/osrf/gazebo/pull-request/2312)

1. Reset ODE joint force feedback after world reset
    * [Pull request 2255](https://bitbucket.org/osrf/gazebo/pull-request/2255)

1. Update model editor snap to grid modifier key
    * [Pull request 2259](https://bitbucket.org/osrf/gazebo/pull-request/2259)
    * [Issue #1583](https://bitbucket.org/osrf/gazebo/issues/1583)

1. PIMPLize gui/model/ModelEditorPalette
    * [Pull request 2279](https://bitbucket.org/osrf/gazebo/pull-request/2279)

1. Properly cleanup pointers when destroying a blank world.
    * [Pull request 2220](https://bitbucket.org/osrf/gazebo/pull-request/2220)

1. Properly cleanup pointers when destroying a world with models and lights.
    * [Pull request 2263](https://bitbucket.org/osrf/gazebo/pull-request/2263)

1. Fix view control mouse focus in model editor
    * [Pull request 2315](https://bitbucket.org/osrf/gazebo/pull-request/2315)
    * [Issue #1791](https://bitbucket.org/osrf/gazebo/issues/1791)

1. Server generates unique model names in case of overlap
    * [Pull request 2296](https://bitbucket.org/osrf/gazebo/pull-request/2296)
    * [Issue 510](https://bitbucket.org/osrf/gazebo/issues/510)

1. Model Editor: Select and align nested models
    * [Pull request 2282](https://bitbucket.org/osrf/gazebo/pull-request/2282)

## Gazebo 7.1.0 (2016-04-07)

1. fix: remove back projection
    * [Pull request 2201](https://bitbucket.org/osrf/gazebo/pull-request/2201)
    * A contribution from Yuki Furuta

1. Fix oculus 2 camera field of view
    * [Pull request 2157](https://bitbucket.org/osrf/gazebo/pull-request/2157)

1. Added BeforePhysicsUpdate world event
    * [Pull request 2128](https://bitbucket.org/osrf/gazebo/pull-request/2128)
    * A contribution from Martin Pecka

1. Update `gz sdf -c` command line tool to use the new `sdf::convertFile` API.
    * [Pull request #2227](https://bitbucket.org/osrf/gazebo/pull-requests/2227)

1. Backport depth camera OSX fix
    * [Pull request 2233](https://bitbucket.org/osrf/gazebo/pull-request/2233)

1. Feat load collision.sdf only once
    * [Pull request 2236](https://bitbucket.org/osrf/gazebo/pull-request/2236)

1. Update gui/building/Item API
    * [Pull request 2228](https://bitbucket.org/osrf/gazebo/pull-request/2228)

1. Semantic version class to compare model versions in the model database.
    * [Pull request 2207](https://bitbucket.org/osrf/gazebo/pull-request/2207)

1. Backport issue 1834 fix to gazebo7
    * [Pull request 2222](https://bitbucket.org/osrf/gazebo/pull-request/2222)

1. Backport ImagesView_TEST changes
    * [Pull request 2217](https://bitbucket.org/osrf/gazebo/pull-request/2217)

1. Backport pull request #2189 (mutex in Transport::Conection)
    * [Pull request 2208](https://bitbucket.org/osrf/gazebo/pull-request/2208)

1. Process insertions on World::SetState
    * [Pull request #2200](https://bitbucket.org/osrf/gazebo/pull-requests/2200)

1. Process deletions on World::SetState
    * [Pull request #2204](https://bitbucket.org/osrf/gazebo/pull-requests/2204)

1. Fix ray-cylinder collision
    * [Pull request 2124](https://bitbucket.org/osrf/gazebo/pull-request/2124)

1. Fix editing physics parameters in gzclient, update test
    * [Pull request 2192](https://bitbucket.org/osrf/gazebo/pull-request/2192)

1. Fix Audio Decoder test failure
    * [Pull request 2193](https://bitbucket.org/osrf/gazebo/pull-request/2193)

1. Add layers to building levels
    * [Pull request 2180](https://bitbucket.org/osrf/gazebo/pull-request/2180)

1. Allow dynamically adding links to a model.
    * [Pull request #2185](https://bitbucket.org/osrf/gazebo/pull-requests/2185)

1. Fix editing physics parameters in gzclient, update test
    * [Pull request #2192](https://bitbucket.org/osrf/gazebo/pull-requests/2192)
    * [Issue #1876](https://bitbucket.org/osrf/gazebo/issues/1876)

1. Model database selects the latest model version.
    * [Pull request #2207](https://bitbucket.org/osrf/gazebo/pull-requests/2207)

1. Only link relevant libraries to tests
    * [Pull request 2130](https://bitbucket.org/osrf/gazebo/pull-request/2130)

1. PIMPLize gui/model/ModelCreator
    * [Pull request 2171](https://bitbucket.org/osrf/gazebo/pull-request/2171)

1. backport warning and test fixes from pull request #2177
    * [Pull request 2179](https://bitbucket.org/osrf/gazebo/pull-request/2179)

1. Prevent xml parser error from crashing LogPlay on osx -> gazebo7
    * [Pull request 2174](https://bitbucket.org/osrf/gazebo/pull-request/2174)

1. PIMPLize gui/building/ScaleWidget
    * [Pull request 2164](https://bitbucket.org/osrf/gazebo/pull-request/2164)

1. Fix using Shift key while scaling inside the model editor
    * [Pull request 2165](https://bitbucket.org/osrf/gazebo/pull-request/2165)

1. Backport fix for ign-math explicit constructors -> gazebo7
    * [Pull request 2163](https://bitbucket.org/osrf/gazebo/pull-request/2163)

1. Display physics engine type in the GUI
    * [Pull request #2155](https://bitbucket.org/osrf/gazebo/pull-requests/2155)
    * [Issue #1121](https://bitbucket.org/osrf/gazebo/issues/1121)
    * A contribution from Mohamd Ayman

1. Fix compilation against ffmpeg3 (libavcodec)
    * [Pull request #2154](https://bitbucket.org/osrf/gazebo/pull-request/2154)

1. Append a missing </gazebo_log> tag to log files when played.
    * [Pull request #2143](https://bitbucket.org/osrf/gazebo/pull-request/2143)

1. Add helper function QTestFixture::ProcessEventsAndDraw
    * [Pull request #2147](https://bitbucket.org/osrf/gazebo/pull-request/2147)

1. Add qt resources to gazebo gui library
    * [Pull request 2134](https://bitbucket.org/osrf/gazebo/pull-request/2134)

1. Undo scaling during simulation
    * [Pull request #2108](https://bitbucket.org/osrf/gazebo/pull-request/2108)

1. Fix SensorManager::SensorContainer::RunLoop sensor update time assertion
    * [Pull request #2115](https://bitbucket.org/osrf/gazebo/pull-request/2115)

1. Fix use of not initialized static attribute in Light class
    * [Pull request 2075](https://bitbucket.org/osrf/gazebo/pull-request/2075)
    * A contribution from Silvio Traversaro

1. Install GuiTypes header
    * [Pull request 2106](https://bitbucket.org/osrf/gazebo/pull-request/2106)

1. Removes one function call and replaces a manual swap with std::swap in ODE heightfield.
    * [Pull request #2114](https://bitbucket.org/osrf/gazebo/pull-request/2114)

1. New world event: BeforePhysicsUpdate
    * [Pull request #2128](https://bitbucket.org/osrf/gazebo/pull-request/2128)
    * [Issue #1851](https://bitbucket.org/osrf/gazebo/issues/1851)

1. Model editor: Fix setting relative pose after alignment during joint creation.
    * [Issue #1844](https://bitbucket.org/osrf/gazebo/issues/1844)
    * [Pull request #2150](https://bitbucket.org/osrf/gazebo/pull-request/2150)

1. Model editor: Fix saving and spawning model with its original name
    * [Pull request #2183](https://bitbucket.org/osrf/gazebo/pull-request/2183)

1. Model editor: Fix inserting custom links
    * [Pull request #2222](https://bitbucket.org/osrf/gazebo/pull-request/2222)
    * [Issue #1834](https://bitbucket.org/osrf/gazebo/issues/1834)

1. Model editor: Reset visual / collision insertion / deletion
        * [Pull request #2254](https://bitbucket.org/osrf/gazebo/pull-request/2254)
        * [Issue #1777](https://bitbucket.org/osrf/gazebo/issues/1777)
        * [Issue #1852](https://bitbucket.org/osrf/gazebo/issues/1852)

1. Building editor: Add layers to building levels
    * [Pull request #2180](https://bitbucket.org/osrf/gazebo/pull-request/2180)
    * [Issue #1806](https://bitbucket.org/osrf/gazebo/issues/1806)

1. Building editor: Update gui/building/Item API
    * [Pull request #2228](https://bitbucket.org/osrf/gazebo/pull-request/2228)

## Gazebo 7.0.0 (2016-01-25)

1. Add FollowerPlugin
    * [Pull request #2085](https://bitbucket.org/osrf/gazebo/pull-request/2085)

1. Fix circular dependency so that physics does not call the sensors API.
    * [Pull request #2089](https://bitbucket.org/osrf/gazebo/pull-request/2089)
    * [Issue #1516](https://bitbucket.org/osrf/gazebo/issues/1516)

1. Add Gravity and MagneticField API to World class to match sdformat change.
    * [SDFormat pull request 247](https://bitbucket.org/osrf/sdformat/pull-requests/247)
    * [Issue #1823](https://bitbucket.org/osrf/gazebo/issues/1823)
    * [Pull request #2090](https://bitbucket.org/osrf/gazebo/pull-request/2090)

1. Use opaque pointers and deprecate functions in the rendering library
    * [Pull request #2069](https://bitbucket.org/osrf/gazebo/pull-request/2069)
    * [Pull request #2064](https://bitbucket.org/osrf/gazebo/pull-request/2064)
    * [Pull request #2066](https://bitbucket.org/osrf/gazebo/pull-request/2066)
    * [Pull request #2069](https://bitbucket.org/osrf/gazebo/pull-request/2069)
    * [Pull request #2074](https://bitbucket.org/osrf/gazebo/pull-request/2074)
    * [Pull request #2076](https://bitbucket.org/osrf/gazebo/pull-request/2076)
    * [Pull request #2070](https://bitbucket.org/osrf/gazebo/pull-request/2070)
    * [Pull request #2071](https://bitbucket.org/osrf/gazebo/pull-request/2071)
    * [Pull request #2084](https://bitbucket.org/osrf/gazebo/pull-request/2084)
    * [Pull request #2073](https://bitbucket.org/osrf/gazebo/pull-request/2073)

1. Use opaque pointers for the Master class.
    * [Pull request #2036](https://bitbucket.org/osrf/gazebo/pull-request/2036)

1. Use opaque pointers in the gui library
    * [Pull request #2057](https://bitbucket.org/osrf/gazebo/pull-request/2057)
    * [Pull request #2037](https://bitbucket.org/osrf/gazebo/pull-request/2037)
    * [Pull request #2052](https://bitbucket.org/osrf/gazebo/pull-request/2052)
    * [Pull request #2053](https://bitbucket.org/osrf/gazebo/pull-request/2053)
    * [Pull request #2028](https://bitbucket.org/osrf/gazebo/pull-request/2028)
    * [Pull request #2051](https://bitbucket.org/osrf/gazebo/pull-request/2051)
    * [Pull request #2027](https://bitbucket.org/osrf/gazebo/pull-request/2027)
    * [Pull request #2026](https://bitbucket.org/osrf/gazebo/pull-request/2026)
    * [Pull request #2029](https://bitbucket.org/osrf/gazebo/pull-request/2029)
    * [Pull request #2042](https://bitbucket.org/osrf/gazebo/pull-request/2042)

1. Use more opaque pointers.
    * [Pull request #2022](https://bitbucket.org/osrf/gazebo/pull-request/2022)
    * [Pull request #2025](https://bitbucket.org/osrf/gazebo/pull-request/2025)
    * [Pull request #2043](https://bitbucket.org/osrf/gazebo/pull-request/2043)
    * [Pull request #2044](https://bitbucket.org/osrf/gazebo/pull-request/2044)
    * [Pull request #2065](https://bitbucket.org/osrf/gazebo/pull-request/2065)
    * [Pull request #2067](https://bitbucket.org/osrf/gazebo/pull-request/2067)
    * [Pull request #2079](https://bitbucket.org/osrf/gazebo/pull-request/2079)

1. Fix visual transparency issues
    * [Pull request #2031](https://bitbucket.org/osrf/gazebo/pull-request/2031)
    * [Issue #1726](https://bitbucket.org/osrf/gazebo/issue/1726)
    * [Issue #1790](https://bitbucket.org/osrf/gazebo/issue/1790)

1. Implemented private data pointer for the RTShaderSystem class. Minimized shader updates to once per render update.
    * [Pull request #2003](https://bitbucket.org/osrf/gazebo/pull-request/2003)

1. Updating physics library to use ignition math.
    * [Pull request #2007](https://bitbucket.org/osrf/gazebo/pull-request/2007)

1. Switching to ignition math for the rendering library.
    * [Pull request #1993](https://bitbucket.org/osrf/gazebo/pull-request/1993)
    * [Pull request #1994](https://bitbucket.org/osrf/gazebo/pull-request/1994)
    * [Pull request #1995](https://bitbucket.org/osrf/gazebo/pull-request/1995)
    * [Pull request #1996](https://bitbucket.org/osrf/gazebo/pull-request/1996)

1. Removed deprecations
    * [Pull request #1992]((https://bitbucket.org/osrf/gazebo/pull-request/1992)

1. Add ability to set the pose of a visual from a link.
    * [Pull request #1963](https://bitbucket.org/osrf/gazebo/pull-request/1963)

1. Copy visual visibility flags on clone
    * [Pull request #2008](https://bitbucket.org/osrf/gazebo/pull-request/2008)

1. Publish camera sensor image size when rendering is not enabled
    * [Pull request #1969](https://bitbucket.org/osrf/gazebo/pull-request/1969)

1. Added Poissons Ratio and Elastic Modulus for ODE.
    * [Pull request #1974](https://bitbucket.org/osrf/gazebo/pull-request/1974)

1. Update rest web plugin to publish response messages and display login user name in toolbar.
    * [Pull request #1956](https://bitbucket.org/osrf/gazebo/pull-request/1956)

1. Improve overall speed of log playback. Added new functions to LogPlay.
   Use tinyxml2 for playback.
    * [Pull request #1931](https://bitbucket.org/osrf/gazebo/pull-request/1931)

1. Improve SVG import. Added support for transforms in paths.
    * [Pull request #1981](https://bitbucket.org/osrf/gazebo/pull-request/1981)

1. Enter time during log playback
    * [Pull request #2000](https://bitbucket.org/osrf/gazebo/pull-request/2000)

1. Added Ignition Transport dependency.
    * [Pull request #1930](https://bitbucket.org/osrf/gazebo/pull-request/1930)

1. Make latched subscribers receive the message only once
    * [Issue #1789](https://bitbucket.org/osrf/gazebo/issue/1789)
    * [Pull request #2019](https://bitbucket.org/osrf/gazebo/pull-request/2019)

1. Implemented transport clear buffers
    * [Pull request #2017](https://bitbucket.org/osrf/gazebo/pull-request/2017)

1. KeyEvent constructor should be in a source file. Removed a few visibility
flags from c functions. Windows did not like `CPPTYPE_*` in
`gazebo/gui/ConfigWidget.cc`, so I replaced it with `TYPE_*`.
    * [Pull request #1943](https://bitbucket.org/osrf/gazebo/pull-request/1943)

1. Added wide angle camera sensor.
    * [Pull request #1866](https://bitbucket.org/osrf/gazebo/pull-request/1866)

1. Change the `near` and `far` members of `gazebo/msgs/logical_camera_sensors.proto` to `near_clip` and `far_clip`
    + [Pull request #1942](https://bitbucket.org/osrf/gazebo/pull-request/1942)

1. Resolve issue #1702
    * [Issue #1702](https://bitbucket.org/osrf/gazebo/issue/1702)
    * [Pull request #1905](https://bitbucket.org/osrf/gazebo/pull-request/1905)
    * [Pull request #1913](https://bitbucket.org/osrf/gazebo/pull-request/1913)
    * [Pull request #1914](https://bitbucket.org/osrf/gazebo/pull-request/1914)

1. Update physics when the world is reset
    * [Pull request #1903](https://bitbucket.org/osrf/gazebo/pull-request/1903)

1. Light and light state for the server side
    * [Pull request #1920](https://bitbucket.org/osrf/gazebo/pull-request/1920)

1. Add scale to model state so scaling works on log/playback.
    * [Pull request #2020](https://bitbucket.org/osrf/gazebo/pull-request/2020)

1. Added tests for WorldState
    * [Pull request #1968](https://bitbucket.org/osrf/gazebo/pull-request/1968)

1. Rename Reset to Reset Time in time widget
    * [Pull request #1892](https://bitbucket.org/osrf/gazebo/pull-request/1892)
    * [Issue #1730](https://bitbucket.org/osrf/gazebo/issue/1730)

1. Set QTestfFxture to verbose
    * [Pull request #1944](https://bitbucket.org/osrf/gazebo/pull-request/1944)
    * [Issue #1756](https://bitbucket.org/osrf/gazebo/issue/1756)

1. Added torsional friction
    * [Pull request #1831](https://bitbucket.org/osrf/gazebo/pull-request/1831)

1. Support loading and spawning nested models
    * [Pull request #1868](https://bitbucket.org/osrf/gazebo/pull-request/1868)
    * [Pull request #1895](https://bitbucket.org/osrf/gazebo/pull-request/1895)

1. Undo user motion commands during simulation, added physics::UserCmdManager and gui::UserCmdHistory.
    * [Pull request #1934](https://bitbucket.org/osrf/gazebo/pull-request/1934)

1. Forward user command messages for undo.
    * [Pull request #2009](https://bitbucket.org/osrf/gazebo/pull-request/2009)

1. Undo reset commands during simulation, forwarding commands
    * [Pull request #1986](https://bitbucket.org/osrf/gazebo/pull-request/1986)

1. Undo apply force / torque during simulation
    * [Pull request #2030](https://bitbucket.org/osrf/gazebo/pull-request/2030)

1. Add function to get the derived scale of a Visual
    * [Pull request #1881](https://bitbucket.org/osrf/gazebo/pull-request/1881)

1. Added EnumIface, which supports iterators over enums.
    * [Pull request #1847](https://bitbucket.org/osrf/gazebo/pull-request/1847)

1. Added RegionEventBoxPlugin - fires events when models enter / exit the region
    * [Pull request #1856](https://bitbucket.org/osrf/gazebo/pull-request/1856)

1. Added tests for checking the playback control via messages.
    * [Pull request #1885](https://bitbucket.org/osrf/gazebo/pull-request/1885)

1. Added LoadArgs() function to ServerFixture for being able to load a server
using the same arguments used in the command line.
    * [Pull request #1874](https://bitbucket.org/osrf/gazebo/pull-request/1874)

1. Added battery class, plugins and test world.
    * [Pull request #1872](https://bitbucket.org/osrf/gazebo/pull-request/1872)

1. Display gearbox and screw joint properties in property tree
    * [Pull request #1838](https://bitbucket.org/osrf/gazebo/pull-request/1838)

1. Set window flags for dialogs and file dialogs
    * [Pull request #1816](https://bitbucket.org/osrf/gazebo/pull-request/1816)

1. Fix minimum window height
   * [Pull request #1977](https://bitbucket.org/osrf/gazebo/pull-request/1977)
   * [Issue #1706](https://bitbucket.org/osrf/gazebo/issue/1706)

1. Add option to reverse alignment direction
   * [Pull request #2040](https://bitbucket.org/osrf/gazebo/pull-request/2040)
   * [Issue #1242](https://bitbucket.org/osrf/gazebo/issue/1242)

1. Fix unadvertising a publisher - only unadvertise topic if it is the last publisher.
   * [Pull request #2005](https://bitbucket.org/osrf/gazebo/pull-request/2005)
   * [Issue #1782](https://bitbucket.org/osrf/gazebo/issue/1782)

1. Log playback GUI for multistep, rewind, forward and seek
    * [Pull request #1791](https://bitbucket.org/osrf/gazebo/pull-request/1791)

1. Added Apply Force/Torque movable text
    * [Pull request #1789](https://bitbucket.org/osrf/gazebo/pull-request/1789)

1. Added cascade parameter (apply to children) for Visual SetMaterial, SetAmbient, SetEmissive, SetSpecular, SetDiffuse, SetTransparency
    * [Pull request #1851](https://bitbucket.org/osrf/gazebo/pull-request/1851)

1. Tweaks to Data Logger, such as multiline text edit for path
    * [Pull request #1800](https://bitbucket.org/osrf/gazebo/pull-request/1800)

1. Added TopToolbar and hide / disable several widgets according to WindowMode
    * [Pull request #1869](https://bitbucket.org/osrf/gazebo/pull-request/1869)

1. Added Visual::IsAncestorOf and Visual::IsDescendantOf
    * [Pull request #1850](https://bitbucket.org/osrf/gazebo/pull-request/1850)

1. Added msgs::PluginFromSDF and tests
    * [Pull request #1858](https://bitbucket.org/osrf/gazebo/pull-request/1858)

1. Added msgs::CollisionFromSDF msgs::SurfaceFromSDF and msgs::FrictionFromSDF
    * [Pull request #1900](https://bitbucket.org/osrf/gazebo/pull-request/1900)

1. Added hotkeys chart dialog
    * [Pull request #1835](https://bitbucket.org/osrf/gazebo/pull-request/1835)

1. Space bar to play / pause
   * [Pull request #2023](https://bitbucket.org/osrf/gazebo/pull-request/2023)
   * [Issue #1798](https://bitbucket.org/osrf/gazebo/issue/1798)

1. Make it possible to create custom ConfigWidgets
    * [Pull request #1861](https://bitbucket.org/osrf/gazebo/pull-request/1861)

1. AddItem / RemoveItem / Clear enum config widgets
    * [Pull request #1878](https://bitbucket.org/osrf/gazebo/pull-request/1878)

1. Make all child ConfigWidgets emit signals.
    * [Pull request #1884](https://bitbucket.org/osrf/gazebo/pull-request/1884)

1. Refactored makers
    * [Pull request #1828](https://bitbucket.org/osrf/gazebo/pull-request/1828)

1. Added gui::Conversions to convert between Gazebo and Qt
    * [Pull request #2034](https://bitbucket.org/osrf/gazebo/pull-request/2034)

1. Model editor updates
    1. Support adding model plugins in model editor
        * [Pull request #2060](https://bitbucket.org/osrf/gazebo/pull-request/2060)

    1. Added support for copying and pasting top level nested models
        * [Pull request #2006](https://bitbucket.org/osrf/gazebo/pull-request/2006)

    1. Make non-editable background models white in model editor
        * [Pull request #1950](https://bitbucket.org/osrf/gazebo/pull-request/1950)

    1. Choose / swap parent and child links in joint inspector
        * [Pull request #1887](https://bitbucket.org/osrf/gazebo/pull-request/1887)
        * [Issue #1500](https://bitbucket.org/osrf/gazebo/issue/1500)

    1. Presets combo box for Vector3 config widget
        * [Pull request #1954](https://bitbucket.org/osrf/gazebo/pull-request/1954)

    1. Added support for more joint types (gearbox and fixed joints).
        * [Pull request #1794](https://bitbucket.org/osrf/gazebo/pull-request/1794)

    1. Added support for selecting links and joints, opening context menu and inspectors in Schematic View.
        * [Pull request #1787](https://bitbucket.org/osrf/gazebo/pull-request/1787)

    1. Color-coded edges in Schematic View to match joint color.
        * [Pull request #1781](https://bitbucket.org/osrf/gazebo/pull-request/1781)

    1. Scale link mass and inertia when a link is scaled
        * [Pull request #1836](https://bitbucket.org/osrf/gazebo/pull-request/1836)

    1. Add density widget to config widget and link inspector
        * [Pull request #1978](https://bitbucket.org/osrf/gazebo/pull-request/1978)

    1. Added icons for child and parent link in joint inspector
        * [Pull request #1953](https://bitbucket.org/osrf/gazebo/pull-request/1953)

    1. Load and save nested models
        * [Pull request #1894](https://bitbucket.org/osrf/gazebo/pull-request/1894)

    1. Display model plugins on the left panel and added model plugin inspector
        * [Pull request #1863](https://bitbucket.org/osrf/gazebo/pull-request/1863)

    1. Context menu and deletion for model plugins
        * [Pull request #1890](https://bitbucket.org/osrf/gazebo/pull-request/1890)

    1. Delete self from inspector
        * [Pull request #1904](https://bitbucket.org/osrf/gazebo/pull-request/1904)
        * [Issue #1543](https://bitbucket.org/osrf/gazebo/issue/1543)

    1. Apply inspector changes in real time and add reset button
        * [Pull request #1945](https://bitbucket.org/osrf/gazebo/pull-request/1945)
        * [Issue #1472](https://bitbucket.org/osrf/gazebo/issue/1472)

    1. Set physics to be paused when exiting model editor mode
        * [Pull request #1893](https://bitbucket.org/osrf/gazebo/pull-request/1893)
        * [Issue #1734](https://bitbucket.org/osrf/gazebo/issue/1734)

    1. Add Insert tab to model editor
        * [Pull request #1924](https://bitbucket.org/osrf/gazebo/pull-request/1924)

    1. Support inserting nested models from model maker
        * [Pull request #1982](https://bitbucket.org/osrf/gazebo/pull-request/1982)

    1. Added joint creation dialog
        * [Pull request #2021](https://bitbucket.org/osrf/gazebo/pull-request/2021)

    1. Added reverse checkboxes to joint creation dialog
        * [Pull request #2086](https://bitbucket.org/osrf/gazebo/pull-request/2086)

    1. Use opaque pointers in the model editor
        * [Pull request #2056](https://bitbucket.org/osrf/gazebo/pull-request/2056)
        * [Pull request #2059](https://bitbucket.org/osrf/gazebo/pull-request/2059)
        * [Pull request #2087](https://bitbucket.org/osrf/gazebo/pull-request/2087)

    1. Support joint creation between links in nested model.
        * [Pull request #2080](https://bitbucket.org/osrf/gazebo/pull-request/2080)

1. Building editor updates

    1. Use opaque pointers in the building editor
        * [Pull request #2041](https://bitbucket.org/osrf/gazebo/pull-request/2041)
        * [Pull request #2039](https://bitbucket.org/osrf/gazebo/pull-request/2039)
        * [Pull request #2055](https://bitbucket.org/osrf/gazebo/pull-request/2055)
        * [Pull request #2032](https://bitbucket.org/osrf/gazebo/pull-request/2032)
        * [Pull request #2082](https://bitbucket.org/osrf/gazebo/pull-request/2082)
        * [Pull request #2038](https://bitbucket.org/osrf/gazebo/pull-request/2038)
        * [Pull request #2033](https://bitbucket.org/osrf/gazebo/pull-request/2033)

    1. Use opaque pointers for GrabberHandle, add *LinkedGrabbers functions
        * [Pull request #2034](https://bitbucket.org/osrf/gazebo/pull-request/2034)

    1. Removed unused class: BuildingItem
        * [Pull request #2045](https://bitbucket.org/osrf/gazebo/pull-request/2045)

    1. Use opaque pointers for BuildingModelManip, move attachment logic to BuildingMaker
        * [Pull request #2046](https://bitbucket.org/osrf/gazebo/pull-request/2046)

    1. Use opaque pointers for all Dialog classes, add conversion from QPointF, move common logic to BaseInspectorDialog.
        * [Pull request #2083](https://bitbucket.org/osrf/gazebo/pull-request/2083)

## Gazebo 6.0

### Gazebo 6.7.0 (201X-01-12)

1. Add vector3 and quaternion rendering conversions
    * [Pull request 2276](https://bitbucket.org/osrf/gazebo/pull-request/2276)

1. Reverse view angle widget left and right view
    * [Pull request 2265](https://bitbucket.org/osrf/gazebo/pull-request/2265)
    * [Issue 1924](https://bitbucket.org/osrf/gazebo/issue/1924)

1. Fix race condition in ~TimePanelPrivate (#1919)
    * [Pull request 2250](https://bitbucket.org/osrf/gazebo/pull-request/2250)

1. Prevent orthographic camera from resetting zoom after animation
    * [Pull request 2267](https://bitbucket.org/osrf/gazebo/pull-request/2267)
    * [Issue #1927](https://bitbucket.org/osrf/gazebo/issues/1927)

1. Fix MeshToSDF missing scale issue
    * [Pull request 2258](https://bitbucket.org/osrf/gazebo/pull-request/2258)
    * [Issue #1925](https://bitbucket.org/osrf/gazebo/issues/1925)

1. Register Qt metatypes in gui tests
    * [Pull request 2273](https://bitbucket.org/osrf/gazebo/pull-request/2273)

1. Fix resetting model to initial pose
    * [Pull request 2307](https://bitbucket.org/osrf/gazebo/pull-request/2307)
    * [Issue #1960](https://bitbucket.org/osrf/gazebo/issues/1960)


### Gazebo 6.6.0 (2016-04-07)

1. fix: remove back projection
    * [Pull request 2201](https://bitbucket.org/osrf/gazebo/pull-request/2201)
    * A contribution from Yuki Furuta

1. Backport depth camera OSX fix and test
    * [Pull request 2230](https://bitbucket.org/osrf/gazebo/pull-request/2230)

1. Add missing tinyxml includes (gazebo6)
    * [Pull request 2218](https://bitbucket.org/osrf/gazebo/pull-request/2218)

1. Fix ray-cylinder collision in ode
    * [Pull request 2125](https://bitbucket.org/osrf/gazebo/pull-request/2125)

1. backport fixes for ffmpeg3 to gazebo6 (from pull request #2154)
    * [Pull request 2162](https://bitbucket.org/osrf/gazebo/pull-request/2162)

1. Install shapes_bitmask.world
    * [Pull request 2104](https://bitbucket.org/osrf/gazebo/pull-request/2104)

1. Add gazebo_client to gazebo.pc (gazebo6)
    * [Pull request 2102](https://bitbucket.org/osrf/gazebo/pull-request/2102)

1. Fix removing multiple camera sensors that have the same camera name
    * [Pull request 2081](https://bitbucket.org/osrf/gazebo/pull-request/2081)

1. Ensure that LINK_FRAME_VISUAL arrow components are deleted (#1812)
    * [Pull request 2078](https://bitbucket.org/osrf/gazebo/pull-request/2078)

1. add migration notes for gazebo::setupClient to gazebo::client::setup
    * [Pull request 2068](https://bitbucket.org/osrf/gazebo/pull-request/2068)

1. Update inertia properties during simulation: part 2
    * [Pull request 1984](https://bitbucket.org/osrf/gazebo/pull-request/1984)

1. Fix minimum window height
    * [Pull request 2002](https://bitbucket.org/osrf/gazebo/pull-request/2002)

1. Backport gpu laser test fix
    * [Pull request 1999](https://bitbucket.org/osrf/gazebo/pull-request/1999)

1. Relax physics tolerances for single-precision bullet (gazebo6)
    * [Pull request 1997](https://bitbucket.org/osrf/gazebo/pull-request/1997)

1. Fix minimum window height
    * [Pull request 1998](https://bitbucket.org/osrf/gazebo/pull-request/1998)

1. backport model editor fixed joint option to gazebo6
    * [Pull request 1957](https://bitbucket.org/osrf/gazebo/pull-request/1957)

1. Update shaders once per render update
    * [Pull request 1991](https://bitbucket.org/osrf/gazebo/pull-request/1991)

1. Relax physics tolerances for single-precision bullet
    * [Pull request 1976](https://bitbucket.org/osrf/gazebo/pull-request/1976)

1. Fix visual transparency issues
    * [Pull request 1967](https://bitbucket.org/osrf/gazebo/pull-request/1967)

1. fix memory corruption in transport/Publisher.cc
    * [Pull request 1951](https://bitbucket.org/osrf/gazebo/pull-request/1951)

1. Add test for SphericalCoordinates::LocalFromGlobal
    * [Pull request 1959](https://bitbucket.org/osrf/gazebo/pull-request/1959)

### Gazebo 6.5.1 (2015-10-29)

1. Fix removing multiple camera sensors that have the same camera name.
    * [Pull request #2081](https://bitbucket.org/osrf/gazebo/pull-request/2081)
    * [Issue #1811](https://bitbucket.org/osrf/gazebo/issues/1811)

1. Backport model editor toolbar fixed joint option from [pull request #1794](https://bitbucket.org/osrf/gazebo/pull-request/1794)
    * [Pull request #1957](https://bitbucket.org/osrf/gazebo/pull-request/1957)

1. Fix minimum window height
    * Backport of [pull request #1977](https://bitbucket.org/osrf/gazebo/pull-request/1977)
    * [Pull request #1998](https://bitbucket.org/osrf/gazebo/pull-request/1998)
    * [Issue #1706](https://bitbucket.org/osrf/gazebo/issue/1706)

1. Fix visual transparency issues
    * [Pull request #1967](https://bitbucket.org/osrf/gazebo/pull-request/1967)
    * [Issue #1726](https://bitbucket.org/osrf/gazebo/issue/1726)

### Gazebo 6.5.0 (2015-10-22)

1. Added ability to convert from spherical coordinates to local coordinates.
    * [Pull request #1955](https://bitbucket.org/osrf/gazebo/pull-request/1955)

### Gazebo 6.4.0 (2015-10-14)

1. Fix ABI problem. Make `Sensor::SetPose` function non virtual.
    * [Pull request #1947](https://bitbucket.org/osrf/gazebo/pull-request/1947)

1. Update inertia properties during simulation
    * [Pull request #1909](https://bitbucket.org/osrf/gazebo/pull-requests/1909)
    * [Design document](https://bitbucket.org/osrf/gazebo_design/src/default/inertia_resize/inertia_resize.md)

1. Fix transparency correction for opaque materials
    * [Pull request #1946](https://bitbucket.org/osrf/gazebo/pull-requests/1946/fix-transparency-correction-for-opaque/diff)

### Gazebo 6.3.0 (2015-10-06)

1. Added `Sensor::SetPose` function
    * [Pull request #1935](https://bitbucket.org/osrf/gazebo/pull-request/1935)

### Gazebo 6.2.0 (2015-10-02)

1. Update physics when the world is reset
    * Backport of [pull request #1903](https://bitbucket.org/osrf/gazebo/pull-request/1903)
    * [Pull request #1916](https://bitbucket.org/osrf/gazebo/pull-request/1916)
    * [Issue #101](https://bitbucket.org/osrf/gazebo/issue/101)

1. Added Copy constructor and assignment operator to MouseEvent
    * [Pull request #1855](https://bitbucket.org/osrf/gazebo/pull-request/1855)

### Gazebo 6.1.0 (2015-08-02)

1. Added logical_camera sensor.
    * [Pull request #1845](https://bitbucket.org/osrf/gazebo/pull-request/1845)

1. Added RandomVelocityPlugin, which applies a random velocity to a model's link.
    * [Pull request #1839](https://bitbucket.org/osrf/gazebo/pull-request/1839)

1. Sim events for joint position, velocity and applied force
    * [Pull request #1849](https://bitbucket.org/osrf/gazebo/pull-request/1849)

### Gazebo 6.0.0 (2015-07-27)

1. Added magnetometer sensor. A contribution from Andrew Symington.
    * [Pull request #1788](https://bitbucket.org/osrf/gazebo/pull-request/1788)

1. Added altimeter sensor. A contribution from Andrew Symington.
    * [Pull request #1792](https://bitbucket.org/osrf/gazebo/pull-request/1792)

1. Implement more control options for log playback:
  1. Rewind: The simulation starts from the beginning.
  1. Forward: The simulation jumps to the end of the log file.
  1. Seek: The simulation jumps to a specific point specified by its simulation
  time.
      * [Pull request #1737](https://bitbucket.org/osrf/gazebo/pull-request/1737)

1. Added Gazebo splash screen
    * [Pull request #1745](https://bitbucket.org/osrf/gazebo/pull-request/1745)

1. Added a transporter plugin which allows models to move from one location
   to another based on their location and the location of transporter pads.
    * [Pull request #1738](https://bitbucket.org/osrf/gazebo/pull-request/1738)

1. Implement forward/backwards multi-step for log playback. Now, the semantics
of a multi-step while playing back a log session are different from a multi-step
during a live simulation. While playback, a multi-step simulates all the
intermediate steps as before, but the client only perceives a single step.
E.g: You have a log file containing a 1 hour simulation session. You want to
jump to the minute 00H::30M::00S to check a specific aspect of the simulation.
You should not see continuous updates until minute 00H:30M:00S. Instead, you
should visualize a single jump to the specific instant of the simulation that
you are interested.
    * [Pull request #1623](https://bitbucket.org/osrf/gazebo/pull-request/1623)

1. Added browse button to log record dialog.
    * [Pull request #1719](https://bitbucket.org/osrf/gazebo/pull-request/1719)

1. Improved SVG support: arcs in paths, and contours made of multiple paths.
    * [Pull request #1608](https://bitbucket.org/osrf/gazebo/pull-request/1608)

1. Added simulation iterations to the world state.
    * [Pull request #1722](https://bitbucket.org/osrf/gazebo/pull-request/1722)

1. Added multiple LiftDrag plugins to the cessna_demo.world to allow the Cessna
C-172 model to fly.
    * [Pull request #1715](https://bitbucket.org/osrf/gazebo/pull-request/1715)

1. Added a plugin to control a Cessna C-172 via messages (CessnaPlugin), and a
GUI plugin to test this functionality with the keyboard (CessnaGUIPlugin). Added
world with the Cessna model and the two previous plugins loaded
(cessna_demo.world).
    * [Pull request #1712](https://bitbucket.org/osrf/gazebo/pull-request/1712)

1. Added world with OSRF building and an elevator
    * [Pull request #1697](https://bitbucket.org/osrf/gazebo/pull-request/1697)

1. Fixed collide bitmask by changing default value from 0x1 to 0xffff.
    * [Pull request #1696](https://bitbucket.org/osrf/gazebo/pull-request/1696)

1. Added a plugin to control an elevator (ElevatorPlugin), and an OccupiedEvent plugin that sends a message when a model is within a specified region.
    * [Pull request #1694](https://bitbucket.org/osrf/gazebo/pull-request/1694)
    * [Pull request #1775](https://bitbucket.org/osrf/gazebo/pull-request/1775)

1. Added Layers tab and meta information for visuals.
    * [Pull request #1674](https://bitbucket.org/osrf/gazebo/pull-request/1674)

1. Added countdown behavior for common::Timer and exposed the feature in TimerGUIPlugin.
    * [Pull request #1690](https://bitbucket.org/osrf/gazebo/pull-request/1690)

1. Added BuoyancyPlugin for simulating the buoyancy of an object in a column of fluid.
    * [Pull request #1622](https://bitbucket.org/osrf/gazebo/pull-request/1622)

1. Added ComputeVolume function for simple shape subclasses of Shape.hh.
    * [Pull request #1605](https://bitbucket.org/osrf/gazebo/pull-request/1605)

1. Add option to parallelize the ODE quickstep constraint solver,
which solves an LCP twice with different parameters in order
to corrected for position projection errors.
    * [Pull request #1561](https://bitbucket.org/osrf/gazebo/pull-request/1561)

1. Get/Set user camera pose in GUI.
    * [Pull request #1649](https://bitbucket.org/osrf/gazebo/pull-request/1649)
    * [Issue #1595](https://bitbucket.org/osrf/gazebo/issue/1595)

1. Added ViewAngleWidget, removed hard-coded reset view and removed MainWindow::Reset(). Also added GLWidget::GetSelectedVisuals().
    * [Pull request #1768](https://bitbucket.org/osrf/gazebo/pull-request/1768)
    * [Issue #1507](https://bitbucket.org/osrf/gazebo/issue/1507)

1. Windows support. This consists mostly of numerous small changes to support
compilation on Windows.
    * [Pull request #1616](https://bitbucket.org/osrf/gazebo/pull-request/1616)
    * [Pull request #1618](https://bitbucket.org/osrf/gazebo/pull-request/1618)
    * [Pull request #1620](https://bitbucket.org/osrf/gazebo/pull-request/1620)
    * [Pull request #1625](https://bitbucket.org/osrf/gazebo/pull-request/1625)
    * [Pull request #1626](https://bitbucket.org/osrf/gazebo/pull-request/1626)
    * [Pull request #1627](https://bitbucket.org/osrf/gazebo/pull-request/1627)
    * [Pull request #1628](https://bitbucket.org/osrf/gazebo/pull-request/1628)
    * [Pull request #1629](https://bitbucket.org/osrf/gazebo/pull-request/1629)
    * [Pull request #1630](https://bitbucket.org/osrf/gazebo/pull-request/1630)
    * [Pull request #1631](https://bitbucket.org/osrf/gazebo/pull-request/1631)
    * [Pull request #1632](https://bitbucket.org/osrf/gazebo/pull-request/1632)
    * [Pull request #1633](https://bitbucket.org/osrf/gazebo/pull-request/1633)
    * [Pull request #1635](https://bitbucket.org/osrf/gazebo/pull-request/1635)
    * [Pull request #1637](https://bitbucket.org/osrf/gazebo/pull-request/1637)
    * [Pull request #1639](https://bitbucket.org/osrf/gazebo/pull-request/1639)
    * [Pull request #1647](https://bitbucket.org/osrf/gazebo/pull-request/1647)
    * [Pull request #1650](https://bitbucket.org/osrf/gazebo/pull-request/1650)
    * [Pull request #1651](https://bitbucket.org/osrf/gazebo/pull-request/1651)
    * [Pull request #1653](https://bitbucket.org/osrf/gazebo/pull-request/1653)
    * [Pull request #1654](https://bitbucket.org/osrf/gazebo/pull-request/1654)
    * [Pull request #1657](https://bitbucket.org/osrf/gazebo/pull-request/1657)
    * [Pull request #1658](https://bitbucket.org/osrf/gazebo/pull-request/1658)
    * [Pull request #1659](https://bitbucket.org/osrf/gazebo/pull-request/1659)
    * [Pull request #1660](https://bitbucket.org/osrf/gazebo/pull-request/1660)
    * [Pull request #1661](https://bitbucket.org/osrf/gazebo/pull-request/1661)
    * [Pull request #1669](https://bitbucket.org/osrf/gazebo/pull-request/1669)
    * [Pull request #1670](https://bitbucket.org/osrf/gazebo/pull-request/1670)
    * [Pull request #1672](https://bitbucket.org/osrf/gazebo/pull-request/1672)
    * [Pull request #1682](https://bitbucket.org/osrf/gazebo/pull-request/1682)
    * [Pull request #1683](https://bitbucket.org/osrf/gazebo/pull-request/1683)

1. Install `libgazebo_server_fixture`. This will facilitate tests external to the main gazebo repository. See `examples/stand_alone/test_fixture`.
    * [Pull request #1606](https://bitbucket.org/osrf/gazebo/pull-request/1606)

1. Laser visualization renders light blue for rays that do not hit obstacles, and dark blue for other rays.
    * [Pull request #1607](https://bitbucket.org/osrf/gazebo/pull-request/1607)
    * [Issue #1576](https://bitbucket.org/osrf/gazebo/issue/1576)

1. Add VisualType enum to Visual and clean up visuals when entity is deleted.
    * [Pull request #1614](https://bitbucket.org/osrf/gazebo/pull-request/1614)

1. Alert user of connection problems when using the REST service plugin
    * [Pull request #1655](https://bitbucket.org/osrf/gazebo/pull-request/1655)
    * [Issue #1574](https://bitbucket.org/osrf/gazebo/issue/1574)

1. ignition-math is now a dependency.
    + [http://ignitionrobotics.org/libraries/math](http://ignitionrobotics.org/libraries/math)
    + [Gazebo::math migration](https://bitbucket.org/osrf/gazebo/src/583edbeb90759d43d994cc57c0797119dd6d2794/ign-math-migration.md)

1. Detect uuid library during compilation.
    * [Pull request #1655](https://bitbucket.org/osrf/gazebo/pull-request/1655)
    * [Issue #1572](https://bitbucket.org/osrf/gazebo/issue/1572)

1. New accessors in LogPlay class.
    * [Pull request #1577](https://bitbucket.org/osrf/gazebo/pull-request/1577)

1. Added a plugin to send messages to an existing website.
   Added gui::MainWindow::AddMenu and msgs/rest_error, msgs/rest_login, msgs rest/post
    * [Pull request #1524](https://bitbucket.org/osrf/gazebo/pull-request/1524)

1. Fix deprecation warnings when using SDFormat 3.0.2, 3.0.3 prereleases
    * [Pull request #1568](https://bitbucket.org/osrf/gazebo/pull-request/1568)

1. Use GAZEBO_CFLAGS or GAZEBO_CXX_FLAGS in CMakeLists.txt for example plugins
    * [Pull request #1573](https://bitbucket.org/osrf/gazebo/pull-request/1573)

1. Added Link::OnWrenchMsg subscriber with test
    * [Pull request #1582](https://bitbucket.org/osrf/gazebo/pull-request/1582)

1. Show/hide GUI overlays using the menu bar.
    * [Pull request #1555](https://bitbucket.org/osrf/gazebo/pull-request/1555)

1. Added world origin indicator rendering::OriginVisual.
    * [Pull request #1700](https://bitbucket.org/osrf/gazebo/pull-request/1700)

1. Show/hide toolbars using the menu bars and shortcut.
   Added MainWindow::CloneAction.
   Added Window menu to Model Editor.
    * [Pull request #1584](https://bitbucket.org/osrf/gazebo/pull-request/1584)

1. Added event to show/hide toolbars.
    * [Pull request #1707](https://bitbucket.org/osrf/gazebo/pull-request/1707)

1. Added optional start/stop/reset buttons to timer GUI plugin.
    * [Pull request #1576](https://bitbucket.org/osrf/gazebo/pull-request/1576)

1. Timer GUI Plugin: Treat negative positions as positions from the ends
    * [Pull request #1703](https://bitbucket.org/osrf/gazebo/pull-request/1703)

1. Added Visual::GetDepth() and Visual::GetNthAncestor()
    * [Pull request #1613](https://bitbucket.org/osrf/gazebo/pull-request/1613)

1. Added a context menu for links
    * [Pull request #1589](https://bitbucket.org/osrf/gazebo/pull-request/1589)

1. Separate TimePanel's display into TimeWidget and LogPlayWidget.
    * [Pull request #1564](https://bitbucket.org/osrf/gazebo/pull-request/1564)

1. Display confirmation message after log is saved
    * [Pull request #1646](https://bitbucket.org/osrf/gazebo/pull-request/1646)

1. Added LogPlayView to display timeline and LogPlaybackStatistics message type.
    * [Pull request #1724](https://bitbucket.org/osrf/gazebo/pull-request/1724)

1. Added Time::FormattedString and removed all other FormatTime functions.
    * [Pull request #1710](https://bitbucket.org/osrf/gazebo/pull-request/1710)

1. Added support for Oculus DK2
    * [Pull request #1526](https://bitbucket.org/osrf/gazebo/pull-request/1526)

1. Use collide_bitmask from SDF to perform collision filtering
    * [Pull request #1470](https://bitbucket.org/osrf/gazebo/pull-request/1470)

1. Pass Coulomb surface friction parameters to DART.
    * [Pull request #1420](https://bitbucket.org/osrf/gazebo/pull-request/1420)

1. Added ModelAlign::SetHighlighted
    * [Pull request #1598](https://bitbucket.org/osrf/gazebo/pull-request/1598)

1. Added various Get functions to Visual. Also added a ConvertGeometryType function to msgs.
    * [Pull request #1402](https://bitbucket.org/osrf/gazebo/pull-request/1402)

1. Get and Set visibility of SelectionObj's handles, with unit test.
    * [Pull request #1417](https://bitbucket.org/osrf/gazebo/pull-request/1417)

1. Set material of SelectionObj's handles.
    * [Pull request #1472](https://bitbucket.org/osrf/gazebo/pull-request/1472)

1. Add SelectionObj::Fini with tests and make Visual::Fini virtual
    * [Pull request #1685](https://bitbucket.org/osrf/gazebo/pull-request/1685)

1. Allow link selection with the mouse if parent model already selected.
    * [Pull request #1409](https://bitbucket.org/osrf/gazebo/pull-request/1409)

1. Added ModelRightMenu::EntityTypes.
    * [Pull request #1414](https://bitbucket.org/osrf/gazebo/pull-request/1414)

1. Scale joint visuals according to link size.
    * [Pull request #1591](https://bitbucket.org/osrf/gazebo/pull-request/1591)
    * [Issue #1563](https://bitbucket.org/osrf/gazebo/issue/1563)

1. Added Gazebo/CoM material.
    * [Pull request #1439](https://bitbucket.org/osrf/gazebo/pull-request/1439)

1. Added arc parameter to MeshManager::CreateTube
    * [Pull request #1436](https://bitbucket.org/osrf/gazebo/pull-request/1436)

1. Added View Inertia and InertiaVisual, changed COMVisual to sphere proportional to mass.
    * [Pull request #1445](https://bitbucket.org/osrf/gazebo/pull-request/1445)

1. Added View Link Frame and LinkFrameVisual. Visual::SetTransparency goes into texture_unit.
    * [Pull request #1762](https://bitbucket.org/osrf/gazebo/pull-request/1762)
    * [Issue #853](https://bitbucket.org/osrf/gazebo/issue/853)

1. Changed the position of Save and Cancel buttons on editor dialogs
    * [Pull request #1442](https://bitbucket.org/osrf/gazebo/pull-request/1442)
    * [Issue #1377](https://bitbucket.org/osrf/gazebo/issue/1377)

1. Fixed Visual material updates
    * [Pull request #1454](https://bitbucket.org/osrf/gazebo/pull-request/1454)
    * [Issue #1455](https://bitbucket.org/osrf/gazebo/issue/1455)

1. Added Matrix3::Inverse() and tests
    * [Pull request #1481](https://bitbucket.org/osrf/gazebo/pull-request/1481)

1. Implemented AddLinkForce for ODE.
    * [Pull request #1456](https://bitbucket.org/osrf/gazebo/pull-request/1456)

1. Updated ConfigWidget class to parse enum values.
    * [Pull request #1518](https://bitbucket.org/osrf/gazebo/pull-request/1518)

1. Added PresetManager to physics libraries and corresponding integration test.
    * [Pull request #1471](https://bitbucket.org/osrf/gazebo/pull-request/1471)

1. Sync name and location on SaveDialog.
    * [Pull request #1563](https://bitbucket.org/osrf/gazebo/pull-request/1563)

1. Added Apply Force/Torque dialog
    * [Pull request #1600](https://bitbucket.org/osrf/gazebo/pull-request/1600)

1. Added Apply Force/Torque visuals
    * [Pull request #1619](https://bitbucket.org/osrf/gazebo/pull-request/1619)

1. Added Apply Force/Torque OnMouseRelease and ActivateWindow
    * [Pull request #1699](https://bitbucket.org/osrf/gazebo/pull-request/1699)

1. Added Apply Force/Torque mouse interactions, modes, activation
    * [Pull request #1731](https://bitbucket.org/osrf/gazebo/pull-request/1731)

1. Added inertia pose getter for COMVisual and COMVisual_TEST
    * [Pull request #1581](https://bitbucket.org/osrf/gazebo/pull-request/1581)

1. Model editor updates
    1. Joint preview using JointVisuals.
        * [Pull request #1369](https://bitbucket.org/osrf/gazebo/pull-request/1369)

    1. Added inspector for configuring link, visual, and collision properties.
        * [Pull request #1408](https://bitbucket.org/osrf/gazebo/pull-request/1408)

    1. Saving, exiting, generalizing SaveDialog.
        * [Pull request #1401](https://bitbucket.org/osrf/gazebo/pull-request/1401)

    1. Inspectors redesign
        * [Pull request #1586](https://bitbucket.org/osrf/gazebo/pull-request/1586)

    1. Edit existing model.
        * [Pull request #1425](https://bitbucket.org/osrf/gazebo/pull-request/1425)

    1. Add joint inspector to link's context menu.
        * [Pull request #1449](https://bitbucket.org/osrf/gazebo/pull-request/1449)
        * [Issue #1443](https://bitbucket.org/osrf/gazebo/issue/1443)

    1. Added button to select mesh file on inspector.
        * [Pull request #1460](https://bitbucket.org/osrf/gazebo/pull-request/1460)
        * [Issue #1450](https://bitbucket.org/osrf/gazebo/issue/1450)

    1. Renamed Part to Link.
        * [Pull request #1478](https://bitbucket.org/osrf/gazebo/pull-request/1478)

    1. Fix snapping inside editor.
        * [Pull request #1489](https://bitbucket.org/osrf/gazebo/pull-request/1489)
        * [Issue #1457](https://bitbucket.org/osrf/gazebo/issue/1457)

    1. Moved DataLogger from Window menu to the toolbar and moved screenshot button to the right.
        * [Pull request #1665](https://bitbucket.org/osrf/gazebo/pull-request/1665)

    1. Keep loaded model's name.
        * [Pull request #1516](https://bitbucket.org/osrf/gazebo/pull-request/1516)
        * [Issue #1504](https://bitbucket.org/osrf/gazebo/issue/1504)

    1. Added ExtrudeDialog.
        * [Pull request #1483](https://bitbucket.org/osrf/gazebo/pull-request/1483)

    1. Hide time panel inside editor and keep main window's paused state.
        * [Pull request #1500](https://bitbucket.org/osrf/gazebo/pull-request/1500)

    1. Fixed pose issues and added ModelCreator_TEST.
        * [Pull request #1509](https://bitbucket.org/osrf/gazebo/pull-request/1509)
        * [Issue #1497](https://bitbucket.org/osrf/gazebo/issue/1497)
        * [Issue #1509](https://bitbucket.org/osrf/gazebo/issue/1509)

    1. Added list of links and joints.
        * [Pull request #1515](https://bitbucket.org/osrf/gazebo/pull-request/1515)
        * [Issue #1418](https://bitbucket.org/osrf/gazebo/issue/1418)

    1. Expose API to support adding items to the palette.
        * [Pull request #1565](https://bitbucket.org/osrf/gazebo/pull-request/1565)

    1. Added menu for toggling joint visualization
        * [Pull request #1551](https://bitbucket.org/osrf/gazebo/pull-request/1551)
        * [Issue #1483](https://bitbucket.org/osrf/gazebo/issue/1483)

    1. Add schematic view to model editor
        * [Pull request #1562](https://bitbucket.org/osrf/gazebo/pull-request/1562)

1. Building editor updates
    1. Make palette tips tooltip clickable to open.
        * [Pull request #1519](https://bitbucket.org/osrf/gazebo/pull-request/1519)
        * [Issue #1370](https://bitbucket.org/osrf/gazebo/issue/1370)

    1. Add measurement unit to building inspectors.
        * [Pull request #1741](https://bitbucket.org/osrf/gazebo/pull-request/1741)
        * [Issue #1363](https://bitbucket.org/osrf/gazebo/issue/1363)

    1. Add `BaseInspectorDialog` as a base class for inspectors.
        * [Pull request #1749](https://bitbucket.org/osrf/gazebo/pull-request/1749)

## Gazebo 5.0

### Gazebo 5.4.0 (2017-01-17)

1. Check FSAA support when creating camera render textures
    * [Pull request 2442](https://bitbucket.org/osrf/gazebo/pull-request/2442)
    * [Issue #1837](https://bitbucket.org/osrf/gazebo/issue/1837)

1. Fix mouse picking with transparent visuals
    * [Pull request 2305](https://bitbucket.org/osrf/gazebo/pull-request/2305)
    * [Issue #1956](https://bitbucket.org/osrf/gazebo/issue/1956)

1. Backport fix for DepthCamera visibility mask
    * [Pull request 2286](https://bitbucket.org/osrf/gazebo/pull-request/2286)
    * [Pull request 2287](https://bitbucket.org/osrf/gazebo/pull-request/2287)

1. Backport sensor reset fix
    * [Pull request 2272](https://bitbucket.org/osrf/gazebo/pull-request/2272)
    * [Issue #1917](https://bitbucket.org/osrf/gazebo/issue/1917)

1. Fix model snap tool highlighting
    * [Pull request 2293](https://bitbucket.org/osrf/gazebo/pull-request/2293)
    * [Issue #1955](https://bitbucket.org/osrf/gazebo/issue/1955)

### Gazebo 5.3.0 (2015-04-07)

1. fix: remove back projection
    * [Pull request 2201](https://bitbucket.org/osrf/gazebo/pull-request/2201)
    * A contribution from Yuki Furuta

1. Backport depth camera OSX fix and test
    * [Pull request 2230](https://bitbucket.org/osrf/gazebo/pull-request/2230)

1. Add missing tinyxml includes
    * [Pull request 2216](https://bitbucket.org/osrf/gazebo/pull-request/2216)

1. backport fixes for ffmpeg3 to gazebo5 (from pull request #2154)
    * [Pull request 2161](https://bitbucket.org/osrf/gazebo/pull-request/2161)

1. Check for valid display using xwininfo -root
    * [Pull request 2111](https://bitbucket.org/osrf/gazebo/pull-request/2111)

1. Don't search for sdformat4 on gazebo5, since gazebo5 can't handle sdformat protocol 1.6
    * [Pull request 2092](https://bitbucket.org/osrf/gazebo/pull-request/2092)

1. Fix minimum window height
    * [Pull request 2002](https://bitbucket.org/osrf/gazebo/pull-request/2002)

1. Relax physics tolerances for single-precision bullet
    * [Pull request 1976](https://bitbucket.org/osrf/gazebo/pull-request/1976)

1. Try finding sdformat 4 in gazebo5 branch
    * [Pull request 1972](https://bitbucket.org/osrf/gazebo/pull-request/1972)

1. Fix_send_message (backport of pull request #1951)
    * [Pull request 1964](https://bitbucket.org/osrf/gazebo/pull-request/1964)
    * A contribution from Samuel Lekieffre

1. Export the media path in the cmake config file.
    * [Pull request 1933](https://bitbucket.org/osrf/gazebo/pull-request/1933)

1. Shorten gearbox test since it is failing via timeout on osx
    * [Pull request 1937](https://bitbucket.org/osrf/gazebo/pull-request/1937)

### Gazebo 5.2.1 (2015-10-02)

1. Fix minimum window height
    * Backport of [pull request #1977](https://bitbucket.org/osrf/gazebo/pull-request/1977)
    * [Pull request #2002](https://bitbucket.org/osrf/gazebo/pull-request/2002)
    * [Issue #1706](https://bitbucket.org/osrf/gazebo/issue/1706)

### Gazebo 5.2.0 (2015-10-02)

1. Initialize sigact struct fields that valgrind said were being used uninitialized
    * [Pull request #1809](https://bitbucket.org/osrf/gazebo/pull-request/1809)

1. Add missing ogre includes to ensure macros are properly defined
    * [Pull request #1813](https://bitbucket.org/osrf/gazebo/pull-request/1813)

1. Use ToSDF functions to simplify physics_friction test
    * [Pull request #1808](https://bitbucket.org/osrf/gazebo/pull-request/1808)

1. Added lines to laser sensor visualization
    * [Pull request #1742](https://bitbucket.org/osrf/gazebo/pull-request/1742)
    * [Issue #935](https://bitbucket.org/osrf/gazebo/issue/935)

1. Fix BulletSliderJoint friction for bullet 2.83
    * [Pull request #1686](https://bitbucket.org/osrf/gazebo/pull-request/1686)

1. Fix heightmap model texture loading.
    * [Pull request #1592](https://bitbucket.org/osrf/gazebo/pull-request/1592)

1. Disable failing pr2 test for dart
    * [Pull request #1540](https://bitbucket.org/osrf/gazebo/pull-request/1540)
    * [Issue #1435](https://bitbucket.org/osrf/gazebo/issue/1435)

### Gazebo 5.1.0 (2015-03-20)
1. Backport pull request #1527 (FindOGRE.cmake for non-Debian systems)
  * [Pull request #1532](https://bitbucket.org/osrf/gazebo/pull-request/1532)

1. Respect system cflags when not using USE_UPSTREAM_CFLAGS
  * [Pull request #1531](https://bitbucket.org/osrf/gazebo/pull-request/1531)

1. Allow light manipulation
  * [Pull request #1529](https://bitbucket.org/osrf/gazebo/pull-request/1529)

1. Allow sdformat 2.3.1+ or 3+ and fix tests
  * [Pull request #1484](https://bitbucket.org/osrf/gazebo/pull-request/1484)

1. Add Link::GetWorldAngularMomentum function and test.
  * [Pull request #1482](https://bitbucket.org/osrf/gazebo/pull-request/1482)

1. Preserve previous GAZEBO_MODEL_PATH values when sourcing setup.sh
  * [Pull request #1430](https://bitbucket.org/osrf/gazebo/pull-request/1430)

1. Implement Coulomb joint friction for DART
  * [Pull request #1427](https://bitbucket.org/osrf/gazebo/pull-request/1427)
  * [Issue #1281](https://bitbucket.org/osrf/gazebo/issue/1281)

1. Fix simple shape normals.
    * [Pull request #1477](https://bitbucket.org/osrf/gazebo/pull-request/1477)
    * [Issue #1369](https://bitbucket.org/osrf/gazebo/issue/1369)

1. Use Msg-to-SDF conversion functions in tests, add ServerFixture::SpawnModel(msgs::Model).
    * [Pull request #1466](https://bitbucket.org/osrf/gazebo/pull-request/1466)

1. Added Model Msg-to-SDF conversion functions and test.
    * [Pull request #1429](https://bitbucket.org/osrf/gazebo/pull-request/1429)

1. Added Joint Msg-to-SDF conversion functions and test.
    * [Pull request #1419](https://bitbucket.org/osrf/gazebo/pull-request/1419)

1. Added Visual, Material Msg-to-SDF conversion functions and ShaderType to string conversion functions.
    * [Pull request #1415](https://bitbucket.org/osrf/gazebo/pull-request/1415)

1. Implement Coulomb joint friction for BulletSliderJoint
  * [Pull request #1452](https://bitbucket.org/osrf/gazebo/pull-request/1452)
  * [Issue #1348](https://bitbucket.org/osrf/gazebo/issue/1348)

### Gazebo 5.0.0 (2015-01-27)
1. Support for using [digital elevation maps](http://gazebosim.org/tutorials?tut=dem) has been added to debian packages.

1. C++11 support (C++11 compatible compiler is now required)
    * [Pull request #1340](https://bitbucket.org/osrf/gazebo/pull-request/1340)

1. Implemented private data pointer for the World class.
    * [Pull request #1383](https://bitbucket.org/osrf/gazebo/pull-request/1383)

1. Implemented private data pointer for the Scene class.
    * [Pull request #1385](https://bitbucket.org/osrf/gazebo/pull-request/1385)

1. Added a events::Event::resetWorld event that is triggered when World::Reset is called.
    * [Pull request #1332](https://bitbucket.org/osrf/gazebo/pull-request/1332)
    * [Issue #1375](https://bitbucket.org/osrf/gazebo/issue/1375)

1. Fixed `math::Box::GetCenter` functionality.
    * [Pull request #1278](https://bitbucket.org/osrf/gazebo/pull-request/1278)
    * [Issue #1327](https://bitbucket.org/osrf/gazebo/issue/1327)

1. Added a GUI timer plugin that facilitates the display and control a timer inside the Gazebo UI.
    * [Pull request #1270](https://bitbucket.org/osrf/gazebo/pull-request/1270)

1. Added ability to load plugins via SDF.
    * [Pull request #1261](https://bitbucket.org/osrf/gazebo/pull-request/1261)

1. Added GUIEvent to hide/show the left GUI pane.
    * [Pull request #1269](https://bitbucket.org/osrf/gazebo/pull-request/1269)

1. Modified KeyEventHandler and GLWidget so that hotkeys can be suppressed by custom KeyEvents set up by developers
    * [Pull request #1251](https://bitbucket.org/osrf/gazebo/pull-request/1251)

1. Added ability to read the directory where the log files are stored.
    * [Pull request #1277](https://bitbucket.org/osrf/gazebo/pull-request/1277)

1. Implemented a simulation cloner
    * [Pull request #1180](https://bitbucket.org/osrf/gazebo/pull-request/1180/clone-a-simulation)

1. Added GUI overlay plugins. Users can now write a Gazebo + QT plugin that displays widgets over the render window.
  * [Pull request #1181](https://bitbucket.org/osrf/gazebo/pull-request/1181)

1. Change behavior of Joint::SetVelocity, add Joint::SetVelocityLimit(unsigned int, double)
  * [Pull request #1218](https://bitbucket.org/osrf/gazebo/pull-request/1218)
  * [Issue #964](https://bitbucket.org/osrf/gazebo/issue/964)

1. Implement Coulomb joint friction for ODE
  * [Pull request #1221](https://bitbucket.org/osrf/gazebo/pull-request/1221)
  * [Issue #381](https://bitbucket.org/osrf/gazebo/issue/381)

1. Implement Coulomb joint friction for BulletHingeJoint
  * [Pull request #1317](https://bitbucket.org/osrf/gazebo/pull-request/1317)
  * [Issue #1348](https://bitbucket.org/osrf/gazebo/issue/1348)

1. Implemented camera lens distortion.
  * [Pull request #1213](https://bitbucket.org/osrf/gazebo/pull-request/1213)

1. Kill rogue gzservers left over from failed INTEGRATION_world_clone tests
   and improve robustness of `UNIT_gz_TEST`
  * [Pull request #1232](https://bitbucket.org/osrf/gazebo/pull-request/1232)
  * [Issue #1299](https://bitbucket.org/osrf/gazebo/issue/1299)

1. Added RenderWidget::ShowToolbar to toggle visibility of top toolbar.
  * [Pull request #1248](https://bitbucket.org/osrf/gazebo/pull-request/1248)

1. Fix joint axis visualization.
  * [Pull request #1258](https://bitbucket.org/osrf/gazebo/pull-request/1258)

1. Change UserCamera view control via joysticks. Clean up rate control vs. pose control.
   see UserCamera::OnJoyPose and UserCamera::OnJoyTwist. Added view twist control toggle
   with joystick button 1.
  * [Pull request #1249](https://bitbucket.org/osrf/gazebo/pull-request/1249)

1. Added RenderWidget::GetToolbar to get the top toolbar and change its actions on ModelEditor.
    * [Pull request #1263](https://bitbucket.org/osrf/gazebo/pull-request/1263)

1. Added accessor for MainWindow graphical widget to GuiIface.
    * [Pull request #1250](https://bitbucket.org/osrf/gazebo/pull-request/1250)

1. Added a ConfigWidget class that takes in a google protobuf message and generates widgets for configuring the fields in the message
    * [Pull request #1285](https://bitbucket.org/osrf/gazebo/pull-request/1285)

1. Added GLWidget::OnModelEditor when model editor is triggered, and MainWindow::OnEditorGroup to manually uncheck editor actions.
    * [Pull request #1283](https://bitbucket.org/osrf/gazebo/pull-request/1283)

1. Added Collision, Geometry, Inertial, Surface Msg-to-SDF conversion functions.
    * [Pull request #1315](https://bitbucket.org/osrf/gazebo/pull-request/1315)

1. Added "button modifier" fields (control, shift, and alt) to common::KeyEvent.
    * [Pull request #1325](https://bitbucket.org/osrf/gazebo/pull-request/1325)

1. Added inputs for environment variable GAZEBO_GUI_INI_FILE for reading a custom .ini file.
    * [Pull request #1252](https://bitbucket.org/osrf/gazebo/pull-request/1252)

1. Fixed crash on "permission denied" bug, added insert_model integration test.
    * [Pull request #1329](https://bitbucket.org/osrf/gazebo/pull-request/1329/)

1. Enable simbody joint tests, implement `SimbodyJoint::GetParam`, create
   `Joint::GetParam`, fix bug in `BulletHingeJoint::SetParam`.
    * [Pull request #1404](https://bitbucket.org/osrf/gazebo/pull-request/1404/)

1. Building editor updates
    1. Fixed inspector resizing.
        * [Pull request #1230](https://bitbucket.org/osrf/gazebo/pull-request/1230)
        * [Issue #395](https://bitbucket.org/osrf/gazebo/issue/395)

    1. Doors and windows move proportionally with wall.
        * [Pull request #1231](https://bitbucket.org/osrf/gazebo/pull-request/1231)
        * [Issue #368](https://bitbucket.org/osrf/gazebo/issue/368)

    1. Inspector dialogs stay on top.
        * [Pull request #1229](https://bitbucket.org/osrf/gazebo/pull-request/1229)
        * [Issue #417](https://bitbucket.org/osrf/gazebo/issue/417)

    1. Make model name editable on palette.
        * [Pull request #1239](https://bitbucket.org/osrf/gazebo/pull-request/1239)

    1. Import background image and improve add/delete levels.
        * [Pull request #1214](https://bitbucket.org/osrf/gazebo/pull-request/1214)
        * [Issue #422](https://bitbucket.org/osrf/gazebo/issue/422)
        * [Issue #361](https://bitbucket.org/osrf/gazebo/issue/361)

    1. Fix changing draw mode.
        * [Pull request #1233](https://bitbucket.org/osrf/gazebo/pull-request/1233)
        * [Issue #405](https://bitbucket.org/osrf/gazebo/issue/405)

    1. Tips on palette's top-right corner.
        * [Pull request #1241](https://bitbucket.org/osrf/gazebo/pull-request/1241)

    1. New buttons and layout for the palette.
        * [Pull request #1242](https://bitbucket.org/osrf/gazebo/pull-request/1242)

    1. Individual wall segments instead of polylines.
        * [Pull request #1246](https://bitbucket.org/osrf/gazebo/pull-request/1246)
        * [Issue #389](https://bitbucket.org/osrf/gazebo/issue/389)
        * [Issue #415](https://bitbucket.org/osrf/gazebo/issue/415)

    1. Fix exiting and saving, exiting when there's nothing drawn, fix text on popups.
        * [Pull request #1296](https://bitbucket.org/osrf/gazebo/pull-request/1296)

    1. Display measure for selected wall segment.
        * [Pull request #1291](https://bitbucket.org/osrf/gazebo/pull-request/1291)
        * [Issue #366](https://bitbucket.org/osrf/gazebo/issue/366)

    1. Highlight selected item's 3D visual.
        * [Pull request #1292](https://bitbucket.org/osrf/gazebo/pull-request/1292)

    1. Added color picker to inspector dialogs.
        * [Pull request #1298](https://bitbucket.org/osrf/gazebo/pull-request/1298)

    1. Snapping on by default, off holding Shift. Improved snapping.
        * [Pull request #1304](https://bitbucket.org/osrf/gazebo/pull-request/1304)

    1. Snap walls to length increments, moved scale to SegmentItem and added Get/SetScale, added SegmentItem::SnapAngle and SegmentItem::SnapLength.
        * [Pull request #1311](https://bitbucket.org/osrf/gazebo/pull-request/1311)

    1. Make buildings available in "Insert Models" tab, improve save flow.
        * [Pull request #1312](https://bitbucket.org/osrf/gazebo/pull-request/1312)

    1. Added EditorItem::SetHighlighted.
        * [Pull request #1308](https://bitbucket.org/osrf/gazebo/pull-request/1308)

    1. Current level is transparent, lower levels opaque, higher levels invisible.
        * [Pull request #1303](https://bitbucket.org/osrf/gazebo/pull-request/1303)

    1. Detach all child manips when item is deleted, added BuildingMaker::DetachAllChildren.
        * [Pull request #1316](https://bitbucket.org/osrf/gazebo/pull-request/1316)

    1. Added texture picker to inspector dialogs.
        * [Pull request #1306](https://bitbucket.org/osrf/gazebo/pull-request/1306)

    1. Measures for doors and windows. Added RectItem::angleOnWall and related Get/Set.
        * [Pull request #1322](https://bitbucket.org/osrf/gazebo/pull-request/1322)
        * [Issue #370](https://bitbucket.org/osrf/gazebo/issue/370)

    1. Added Gazebo/BuildingFrame material to display holes for doors and windows on walls.
        * [Pull request #1338](https://bitbucket.org/osrf/gazebo/pull-request/1338)

    1. Added Gazebo/Bricks material to be used as texture on the building editor.
        * [Pull request #1333](https://bitbucket.org/osrf/gazebo/pull-request/1333)

    1. Pick colors from the palette and assign on 3D view. Added mouse and key event handlers to BuildingMaker, and events to communicate from BuildingModelManip to EditorItem.
        * [Pull request #1336](https://bitbucket.org/osrf/gazebo/pull-request/1336)

    1. Pick textures from the palette and assign in 3D view.
        * [Pull request #1368](https://bitbucket.org/osrf/gazebo/pull-request/1368)

1. Model editor updates
    1. Fix adding/removing event filters .
        * [Pull request #1279](https://bitbucket.org/osrf/gazebo/pull-request/1279)

    1. Enabled multi-selection and align tool inside model editor.
        * [Pull request #1302](https://bitbucket.org/osrf/gazebo/pull-request/1302)
        * [Issue #1323](https://bitbucket.org/osrf/gazebo/issue/1323)

    1. Enabled snap mode inside model editor.
        * [Pull request #1331](https://bitbucket.org/osrf/gazebo/pull-request/1331)
        * [Issue #1318](https://bitbucket.org/osrf/gazebo/issue/1318)

    1. Implemented copy/pasting of links.
        * [Pull request #1330](https://bitbucket.org/osrf/gazebo/pull-request/1330)

1. GUI publishes model selection information on ~/selection topic.
    * [Pull request #1318](https://bitbucket.org/osrf/gazebo/pull-request/1318)

## Gazebo 4.0

### Gazebo 4.x.x (2015-xx-xx)

1. Fix build for Bullet 2.83, enable angle wrapping for BulletHingeJoint
    * [Pull request #1664](https://bitbucket.org/osrf/gazebo/pull-request/1664)

### Gazebo 4.1.3 (2015-05-07)

1. Fix saving visual geom SDF values
    * [Pull request #1597](https://bitbucket.org/osrf/gazebo/pull-request/1597)
1. Fix heightmap model texture loading.
    * [Pull request #1595](https://bitbucket.org/osrf/gazebo/pull-request/1595)
1. Fix visual collision scale on separate client
    * [Pull request #1585](https://bitbucket.org/osrf/gazebo/pull-request/1585)
1. Fix several clang compiler warnings
    * [Pull request #1594](https://bitbucket.org/osrf/gazebo/pull-request/1594)
1. Fix blank save / browse dialogs
    * [Pull request #1544](https://bitbucket.org/osrf/gazebo/pull-request/1544)

### Gazebo 4.1.2 (2015-03-20)

1. Fix quaternion documentation: target Gazebo_4.1
    * [Pull request #1525](https://bitbucket.org/osrf/gazebo/pull-request/1525)
1. Speed up World::Step in loops
    * [Pull request #1492](https://bitbucket.org/osrf/gazebo/pull-request/1492)
1. Reduce selection buffer updates -> 4.1
    * [Pull request #1494](https://bitbucket.org/osrf/gazebo/pull-request/1494)
1. Fix loading of SimbodyPhysics parameters
    * [Pull request #1474](https://bitbucket.org/osrf/gazebo/pull-request/1474)
1. Fix heightmap on OSX -> 4.1
    * [Pull request #1455](https://bitbucket.org/osrf/gazebo/pull-request/1455)
1. Remove extra pose tag in a world file that should not be there
    * [Pull request #1458](https://bitbucket.org/osrf/gazebo/pull-request/1458)
1. Better fix for #236 for IMU that doesn't require ABI changes
    * [Pull request #1448](https://bitbucket.org/osrf/gazebo/pull-request/1448)
1. Fix regression of #236 for ImuSensor in 4.1
    * [Pull request #1446](https://bitbucket.org/osrf/gazebo/pull-request/1446)
1. Preserve previous GAZEBO_MODEL_PATH values when sourcing setup.sh
    * [Pull request #1430](https://bitbucket.org/osrf/gazebo/pull-request/1430)
1. issue #857: fix segfault for simbody screw joint when setting limits due to uninitialized limitForce.
    * [Pull request #1423](https://bitbucket.org/osrf/gazebo/pull-request/1423)
1. Allow multiple contact sensors per link (#960)
    * [Pull request #1413](https://bitbucket.org/osrf/gazebo/pull-request/1413)
1. Fix for issue #351, ODE World Step
    * [Pull request #1406](https://bitbucket.org/osrf/gazebo/pull-request/1406)
1. Disable failing InelasticCollision/0 test (#1394)
    * [Pull request #1405](https://bitbucket.org/osrf/gazebo/pull-request/1405)
1. Prevent out of bounds array access in SkidSteerDrivePlugin (found by cppcheck 1.68)
    * [Pull request #1379](https://bitbucket.org/osrf/gazebo/pull-request/1379)

### Gazebo 4.1.1 (2015-01-15)

1. Fix BulletPlaneShape bounding box (#1265)
    * [Pull request #1367](https://bitbucket.org/osrf/gazebo/pull-request/1367)
1. Fix dart linking errors on osx
    * [Pull request #1372](https://bitbucket.org/osrf/gazebo/pull-request/1372)
1. Update to player interfaces
    * [Pull request #1324](https://bitbucket.org/osrf/gazebo/pull-request/1324)
1. Handle GpuLaser name collisions (#1403)
    * [Pull request #1360](https://bitbucket.org/osrf/gazebo/pull-request/1360)
1. Add checks for handling array's with counts of zero, and read specular values
    * [Pull request #1339](https://bitbucket.org/osrf/gazebo/pull-request/1339)
1. Fix model list widget test
    * [Pull request #1327](https://bitbucket.org/osrf/gazebo/pull-request/1327)
1. Fix ogre includes
    * [Pull request #1323](https://bitbucket.org/osrf/gazebo/pull-request/1323)

### Gazebo 4.1.0 (2014-11-20)

1. Modified GUI rendering to improve the rendering update rate.
    * [Pull request #1487](https://bitbucket.org/osrf/gazebo/pull-request/1487)
1. Add ArrangePlugin for arranging groups of models.
   Also add Model::ResetPhysicsStates to call Link::ResetPhysicsStates
   recursively on all links in model.
    * [Pull request #1208](https://bitbucket.org/osrf/gazebo/pull-request/1208)
1. The `gz model` command line tool will output model info using either `-i` for complete info, or `-p` for just the model pose.
    * [Pull request #1212](https://bitbucket.org/osrf/gazebo/pull-request/1212)
    * [DRCSim Issue #389](https://bitbucket.org/osrf/drcsim/issue/389)
1. Added SignalStats class for computing incremental signal statistics.
    * [Pull request #1198](https://bitbucket.org/osrf/gazebo/pull-request/1198)
1. Add InitialVelocityPlugin to setting the initial state of links
    * [Pull request #1237](https://bitbucket.org/osrf/gazebo/pull-request/1237)
1. Added Quaternion::Integrate function.
    * [Pull request #1255](https://bitbucket.org/osrf/gazebo/pull-request/1255)
1. Added ConvertJointType functions, display more joint info on model list.
    * [Pull request #1259](https://bitbucket.org/osrf/gazebo/pull-request/1259)
1. Added ModelListWidget::AddProperty, removed unnecessary checks on ModelListWidget.
    * [Pull request #1271](https://bitbucket.org/osrf/gazebo/pull-request/1271)
1. Fix loading collada meshes with unsupported input semantics.
    * [Pull request #1319](https://bitbucket.org/osrf/gazebo/pull-request/1319)

### Gazebo 4.0.2 (2014-09-23)

1. Fix and improve mechanism to generate pkgconfig libs
    * [Pull request #1207](https://bitbucket.org/osrf/gazebo/pull-request/1207)
    * [Issue #1284](https://bitbucket.org/osrf/gazebo/issue/1284)
1. Added arat.world
    * [Pull request #1205](https://bitbucket.org/osrf/gazebo/pull-request/1205)
1. Update gzprop to output zip files.
    * [Pull request #1197](https://bitbucket.org/osrf/gazebo/pull-request/1197)
1. Make Collision::GetShape a const function
    * [Pull requset #1189](https://bitbucket.org/osrf/gazebo/pull-request/1189)
1. Install missing physics headers
    * [Pull requset #1183](https://bitbucket.org/osrf/gazebo/pull-request/1183)
1. Remove SimbodyLink::AddTorque console message
    * [Pull requset #1185](https://bitbucket.org/osrf/gazebo/pull-request/1185)
1. Fix log xml
    * [Pull requset #1188](https://bitbucket.org/osrf/gazebo/pull-request/1188)

### Gazebo 4.0.0 (2014-08-08)

1. Added lcov support to cmake
    * [Pull request #1047](https://bitbucket.org/osrf/gazebo/pull-request/1047)
1. Fixed memory leak in image conversion
    * [Pull request #1057](https://bitbucket.org/osrf/gazebo/pull-request/1057)
1. Removed deprecated function
    * [Pull request #1067](https://bitbucket.org/osrf/gazebo/pull-request/1067)
1. Improved collada loading performance
    * [Pull request #1066](https://bitbucket.org/osrf/gazebo/pull-request/1066)
    * [Pull request #1082](https://bitbucket.org/osrf/gazebo/pull-request/1082)
    * [Issue #1134](https://bitbucket.org/osrf/gazebo/issue/1134)
1. Implemented a collada exporter
    * [Pull request #1064](https://bitbucket.org/osrf/gazebo/pull-request/1064)
1. Force torque sensor now makes use of sensor's pose.
    * [Pull request #1076](https://bitbucket.org/osrf/gazebo/pull-request/1076)
    * [Issue #940](https://bitbucket.org/osrf/gazebo/issue/940)
1. Fix Model::GetLinks segfault
    * [Pull request #1093](https://bitbucket.org/osrf/gazebo/pull-request/1093)
1. Fix deleting and saving lights in gzserver
    * [Pull request #1094](https://bitbucket.org/osrf/gazebo/pull-request/1094)
    * [Issue #1182](https://bitbucket.org/osrf/gazebo/issue/1182)
    * [Issue #346](https://bitbucket.org/osrf/gazebo/issue/346)
1. Fix Collision::GetWorldPose. The pose of a collision would not update properly.
    * [Pull request #1049](https://bitbucket.org/osrf/gazebo/pull-request/1049)
    * [Issue #1124](https://bitbucket.org/osrf/gazebo/issue/1124)
1. Fixed the animate_box and animate_joints examples
    * [Pull request #1086](https://bitbucket.org/osrf/gazebo/pull-request/1086)
1. Integrated Oculus Rift functionality
    * [Pull request #1074](https://bitbucket.org/osrf/gazebo/pull-request/1074)
    * [Pull request #1136](https://bitbucket.org/osrf/gazebo/pull-request/1136)
    * [Pull request #1139](https://bitbucket.org/osrf/gazebo/pull-request/1139)
1. Updated Base::GetScopedName
    * [Pull request #1104](https://bitbucket.org/osrf/gazebo/pull-request/1104)
1. Fix collada loader from adding duplicate materials into a Mesh
    * [Pull request #1105](https://bitbucket.org/osrf/gazebo/pull-request/1105)
    * [Issue #1180](https://bitbucket.org/osrf/gazebo/issue/1180)
1. Integrated Razer Hydra functionality
    * [Pull request #1083](https://bitbucket.org/osrf/gazebo/pull-request/1083)
    * [Pull request #1109](https://bitbucket.org/osrf/gazebo/pull-request/1109)
1. Added ability to copy and paste models in the GUI
    * [Pull request #1103](https://bitbucket.org/osrf/gazebo/pull-request/1103)
1. Removed unnecessary inclusion of gazebo.hh and common.hh in plugins
    * [Pull request #1111](https://bitbucket.org/osrf/gazebo/pull-request/1111)
1. Added ability to specify custom road textures
    * [Pull request #1027](https://bitbucket.org/osrf/gazebo/pull-request/1027)
1. Added support for DART 4.1
    * [Pull request #1113](https://bitbucket.org/osrf/gazebo/pull-request/1113)
    * [Pull request #1132](https://bitbucket.org/osrf/gazebo/pull-request/1132)
    * [Pull request #1134](https://bitbucket.org/osrf/gazebo/pull-request/1134)
    * [Pull request #1154](https://bitbucket.org/osrf/gazebo/pull-request/1154)
1. Allow position of joints to be directly set.
    * [Pull request #1097](https://bitbucket.org/osrf/gazebo/pull-request/1097)
    * [Issue #1138](https://bitbucket.org/osrf/gazebo/issue/1138)
1. Added extruded polyline geometry
    * [Pull request #1026](https://bitbucket.org/osrf/gazebo/pull-request/1026)
1. Fixed actor animation
    * [Pull request #1133](https://bitbucket.org/osrf/gazebo/pull-request/1133)
    * [Pull request #1141](https://bitbucket.org/osrf/gazebo/pull-request/1141)
1. Generate a versioned cmake config file
    * [Pull request #1153](https://bitbucket.org/osrf/gazebo/pull-request/1153)
    * [Issue #1226](https://bitbucket.org/osrf/gazebo/issue/1226)
1. Added KMeans class
    * [Pull request #1147](https://bitbucket.org/osrf/gazebo/pull-request/1147)
1. Added --summary-range feature to bitbucket pullrequest tool
    * [Pull request #1156](https://bitbucket.org/osrf/gazebo/pull-request/1156)
1. Updated web links
    * [Pull request #1159](https://bitbucket.org/osrf/gazebo/pull-request/1159)
1. Update tests
    * [Pull request #1155](https://bitbucket.org/osrf/gazebo/pull-request/1155)
    * [Pull request #1143](https://bitbucket.org/osrf/gazebo/pull-request/1143)
    * [Pull request #1138](https://bitbucket.org/osrf/gazebo/pull-request/1138)
    * [Pull request #1140](https://bitbucket.org/osrf/gazebo/pull-request/1140)
    * [Pull request #1127](https://bitbucket.org/osrf/gazebo/pull-request/1127)
    * [Pull request #1115](https://bitbucket.org/osrf/gazebo/pull-request/1115)
    * [Pull request #1102](https://bitbucket.org/osrf/gazebo/pull-request/1102)
    * [Pull request #1087](https://bitbucket.org/osrf/gazebo/pull-request/1087)
    * [Pull request #1084](https://bitbucket.org/osrf/gazebo/pull-request/1084)

## Gazebo 3.0

### Gazebo 3.x.x (yyyy-mm-dd)

1. Fixed sonar and wireless sensor visualization
    * [Pull request #1254](https://bitbucket.org/osrf/gazebo/pull-request/1254)
1. Update visual bounding box when model is selected
    * [Pull request #1280](https://bitbucket.org/osrf/gazebo/pull-request/1280)

### Gazebo 3.1.0 (2014-08-08)

1. Implemented Simbody::Link::Set*Vel
    * [Pull request #1160](https://bitbucket.org/osrf/gazebo/pull-request/1160)
    * [Issue #1012](https://bitbucket.org/osrf/gazebo/issue/1012)
1. Added World::RemoveModel function
    * [Pull request #1106](https://bitbucket.org/osrf/gazebo/pull-request/1106)
    * [Issue #1177](https://bitbucket.org/osrf/gazebo/issue/1177)
1. Fix exit from camera follow mode using the escape key
    * [Pull request #1137](https://bitbucket.org/osrf/gazebo/pull-request/1137)
    * [Issue #1220](https://bitbucket.org/osrf/gazebo/issue/1220)
1. Added support for SDF joint spring stiffness and reference positions
    * [Pull request #1117](https://bitbucket.org/osrf/gazebo/pull-request/1117)
1. Removed the gzmodel_create script
    * [Pull request #1130](https://bitbucket.org/osrf/gazebo/pull-request/1130)
1. Added Vector2 dot product
    * [Pull request #1101](https://bitbucket.org/osrf/gazebo/pull-request/1101)
1. Added SetPositionPID and SetVelocityPID to JointController
    * [Pull request #1091](https://bitbucket.org/osrf/gazebo/pull-request/1091)
1. Fix gzclient startup crash with ogre 1.9
    * [Pull request #1098](https://bitbucket.org/osrf/gazebo/pull-request/1098)
    * [Issue #996](https://bitbucket.org/osrf/gazebo/issue/996)
1. Update the bitbucket_pullrequests tool
    * [Pull request #1108](https://bitbucket.org/osrf/gazebo/pull-request/1108)
1. Light properties now remain in place after move by the user via the GUI.
    * [Pull request #1110](https://bitbucket.org/osrf/gazebo/pull-request/1110)
    * [Issue #1211](https://bitbucket.org/osrf/gazebo/issue/1211)
1. Allow position of joints to be directly set.
    * [Pull request #1096](https://bitbucket.org/osrf/gazebo/pull-request/1096)
    * [Issue #1138](https://bitbucket.org/osrf/gazebo/issue/1138)

### Gazebo 3.0.0 (2014-04-11)

1. Fix bug when deleting the sun light
    * [Pull request #1088](https://bitbucket.org/osrf/gazebo/pull-request/1088)
    * [Issue #1133](https://bitbucket.org/osrf/gazebo/issue/1133)
1. Fix ODE screw joint
    * [Pull request #1078](https://bitbucket.org/osrf/gazebo/pull-request/1078)
    * [Issue #1167](https://bitbucket.org/osrf/gazebo/issue/1167)
1. Update joint integration tests
    * [Pull request #1081](https://bitbucket.org/osrf/gazebo/pull-request/1081)
1. Fixed false positives in cppcheck.
    * [Pull request #1061](https://bitbucket.org/osrf/gazebo/pull-request/1061)
1. Made joint axis reference frame relative to child, and updated simbody and dart accordingly.
    * [Pull request #1069](https://bitbucket.org/osrf/gazebo/pull-request/1069)
    * [Issue #494](https://bitbucket.org/osrf/gazebo/issue/494)
    * [Issue #1143](https://bitbucket.org/osrf/gazebo/issue/1143)
1. Added ability to pass vector of strings to SetupClient and SetupServer
    * [Pull request #1068](https://bitbucket.org/osrf/gazebo/pull-request/1068)
    * [Issue #1132](https://bitbucket.org/osrf/gazebo/issue/1132)
1. Fix error correction in screw constraints for ODE
    * [Pull request #1070](https://bitbucket.org/osrf/gazebo/pull-request/1070)
    * [Issue #1159](https://bitbucket.org/osrf/gazebo/issue/1159)
1. Improved pkgconfig with SDF
    * [Pull request #1062](https://bitbucket.org/osrf/gazebo/pull-request/1062)
1. Added a plugin to simulate aero dynamics
    * [Pull request #905](https://bitbucket.org/osrf/gazebo/pull-request/905)
1. Updated bullet support
    * [Issue #1069](https://bitbucket.org/osrf/gazebo/issue/1069)
    * [Pull request #1011](https://bitbucket.org/osrf/gazebo/pull-request/1011)
    * [Pull request #996](https://bitbucket.org/osrf/gazebo/pull-request/966)
    * [Pull request #1024](https://bitbucket.org/osrf/gazebo/pull-request/1024)
1. Updated simbody support
    * [Pull request #995](https://bitbucket.org/osrf/gazebo/pull-request/995)
1. Updated worlds to SDF 1.5
    * [Pull request #1021](https://bitbucket.org/osrf/gazebo/pull-request/1021)
1. Improvements to ODE
    * [Pull request #1001](https://bitbucket.org/osrf/gazebo/pull-request/1001)
    * [Pull request #1014](https://bitbucket.org/osrf/gazebo/pull-request/1014)
    * [Pull request #1015](https://bitbucket.org/osrf/gazebo/pull-request/1015)
    * [Pull request #1016](https://bitbucket.org/osrf/gazebo/pull-request/1016)
1. New command line tool
    * [Pull request #972](https://bitbucket.org/osrf/gazebo/pull-request/972)
1. Graphical user interface improvements
    * [Pull request #971](https://bitbucket.org/osrf/gazebo/pull-request/971)
    * [Pull request #1013](https://bitbucket.org/osrf/gazebo/pull-request/1013)
    * [Pull request #989](https://bitbucket.org/osrf/gazebo/pull-request/989)
1. Created a friction pyramid class
    * [Pull request #935](https://bitbucket.org/osrf/gazebo/pull-request/935)
1. Added GetWorldEnergy functions to Model, Joint, and Link
    * [Pull request #1017](https://bitbucket.org/osrf/gazebo/pull-request/1017)
1. Preparing Gazebo for admission into Ubuntu
    * [Pull request #969](https://bitbucket.org/osrf/gazebo/pull-request/969)
    * [Pull request #998](https://bitbucket.org/osrf/gazebo/pull-request/998)
    * [Pull request #1002](https://bitbucket.org/osrf/gazebo/pull-request/1002)
1. Add method for querying if useImplicitStiffnessDamping flag is set for a given joint
    * [Issue #629](https://bitbucket.org/osrf/gazebo/issue/629)
    * [Pull request #1006](https://bitbucket.org/osrf/gazebo/pull-request/1006)
1. Fix joint axis frames
    * [Issue #494](https://bitbucket.org/osrf/gazebo/issue/494)
    * [Pull request #963](https://bitbucket.org/osrf/gazebo/pull-request/963)
1. Compute joint anchor pose relative to parent
    * [Issue #1029](https://bitbucket.org/osrf/gazebo/issue/1029)
    * [Pull request #982](https://bitbucket.org/osrf/gazebo/pull-request/982)
1. Cleanup the installed worlds
    * [Issue #1036](https://bitbucket.org/osrf/gazebo/issue/1036)
    * [Pull request #984](https://bitbucket.org/osrf/gazebo/pull-request/984)
1. Update to the GPS sensor
    * [Issue #1059](https://bitbucket.org/osrf/gazebo/issue/1059)
    * [Pull request #978](https://bitbucket.org/osrf/gazebo/pull-request/978)
1. Removed libtool from plugin loading
    * [Pull request #981](https://bitbucket.org/osrf/gazebo/pull-request/981)
1. Added functions to get inertial information for a link in the world frame.
    * [Pull request #1005](https://bitbucket.org/osrf/gazebo/pull-request/1005)

## Gazebo 2.0

### Gazebo 2.2.6 (2015-09-28)

1. Backport fixes to setup.sh from pull request #1430 to 2.2 branch
    * [Pull request 1889](https://bitbucket.org/osrf/gazebo/pull-request/1889)
1. Fix heightmap texture loading (2.2)
    * [Pull request 1596](https://bitbucket.org/osrf/gazebo/pull-request/1596)
1. Prevent out of bounds array access in SkidSteerDrivePlugin (found by cppcheck 1.68)
    * [Pull request 1379](https://bitbucket.org/osrf/gazebo/pull-request/1379)
1. Fix build with boost 1.57 for 2.2 branch (#1399)
    * [Pull request 1358](https://bitbucket.org/osrf/gazebo/pull-request/1358)
1. Fix manpage test failures by incrementing year to 2015
    * [Pull request 1361](https://bitbucket.org/osrf/gazebo/pull-request/1361)
1. Fix build for OS X 10.10 (#1304, #1289)
    * [Pull request 1346](https://bitbucket.org/osrf/gazebo/pull-request/1346)
1. Restore ODELink ABI, use Link variables instead (#1354)
    * [Pull request 1347](https://bitbucket.org/osrf/gazebo/pull-request/1347)
1. Fix inertia_ratio test
    * [Pull request 1344](https://bitbucket.org/osrf/gazebo/pull-request/1344)
1. backport collision visual fix -> 2.2
    * [Pull request 1343](https://bitbucket.org/osrf/gazebo/pull-request/1343)
1. Fix two code_check errors on 2.2
    * [Pull request 1314](https://bitbucket.org/osrf/gazebo/pull-request/1314)
1. issue #243 fix Link::GetWorldLinearAccel and Link::GetWorldAngularAccel for ODE
    * [Pull request 1284](https://bitbucket.org/osrf/gazebo/pull-request/1284)

### Gazebo 2.2.3 (2014-04-29)

1. Removed redundant call to World::Init
    * [Pull request #1107](https://bitbucket.org/osrf/gazebo/pull-request/1107)
    * [Issue #1208](https://bitbucket.org/osrf/gazebo/issue/1208)
1. Return proper error codes when gazebo exits
    * [Pull request #1085](https://bitbucket.org/osrf/gazebo/pull-request/1085)
    * [Issue #1178](https://bitbucket.org/osrf/gazebo/issue/1178)
1. Fixed Camera::GetWorldRotation().
    * [Pull request #1071](https://bitbucket.org/osrf/gazebo/pull-request/1071)
    * [Issue #1087](https://bitbucket.org/osrf/gazebo/issue/1087)
1. Fixed memory leak in image conversion
    * [Pull request #1073](https://bitbucket.org/osrf/gazebo/pull-request/1073)

### Gazebo 2.2.1 (xxxx-xx-xx)

1. Fix heightmap model texture loading.
    * [Pull request #1596](https://bitbucket.org/osrf/gazebo/pull-request/1596)

### Gazebo 2.2.0 (2014-01-10)

1. Fix compilation when using OGRE-1.9 (full support is being worked on)
    * [Issue #994](https://bitbucket.org/osrf/gazebo/issue/994)
    * [Issue #995](https://bitbucket.org/osrf/gazebo/issue/995)
    * [Issue #996](https://bitbucket.org/osrf/gazebo/issue/996)
    * [Pull request #883](https://bitbucket.org/osrf/gazebo/pull-request/883)
1. Added unit test for issue 624.
    * [Issue #624](https://bitbucket.org/osrf/gazebo/issue/624).
    * [Pull request #889](https://bitbucket.org/osrf/gazebo/pull-request/889)
1. Use 3x3 PCF shadows for smoother shadows.
    * [Pull request #887](https://bitbucket.org/osrf/gazebo/pull-request/887)
1. Update manpage copyright to 2014.
    * [Pull request #893](https://bitbucket.org/osrf/gazebo/pull-request/893)
1. Added friction integration test .
    * [Pull request #885](https://bitbucket.org/osrf/gazebo/pull-request/885)
1. Fix joint anchor when link pose is not specified.
    * [Issue #978](https://bitbucket.org/osrf/gazebo/issue/978)
    * [Pull request #862](https://bitbucket.org/osrf/gazebo/pull-request/862)
1. Added (ESC) tooltip for GUI Selection Mode icon.
    * [Issue #993](https://bitbucket.org/osrf/gazebo/issue/993)
    * [Pull request #888](https://bitbucket.org/osrf/gazebo/pull-request/888)
1. Removed old comment about resolved issue.
    * [Issue #837](https://bitbucket.org/osrf/gazebo/issue/837)
    * [Pull request #880](https://bitbucket.org/osrf/gazebo/pull-request/880)
1. Made SimbodyLink::Get* function thread-safe
    * [Issue #918](https://bitbucket.org/osrf/gazebo/issue/918)
    * [Pull request #872](https://bitbucket.org/osrf/gazebo/pull-request/872)
1. Suppressed spurious gzlog messages in ODE::Body
    * [Issue #983](https://bitbucket.org/osrf/gazebo/issue/983)
    * [Pull request #875](https://bitbucket.org/osrf/gazebo/pull-request/875)
1. Fixed Force Torque Sensor Test by properly initializing some values.
    * [Issue #982](https://bitbucket.org/osrf/gazebo/issue/982)
    * [Pull request #869](https://bitbucket.org/osrf/gazebo/pull-request/869)
1. Added breakable joint plugin to support breakable walls.
    * [Pull request #865](https://bitbucket.org/osrf/gazebo/pull-request/865)
1. Used different tuple syntax to fix compilation on OSX mavericks.
    * [Issue #947](https://bitbucket.org/osrf/gazebo/issue/947)
    * [Pull request #858](https://bitbucket.org/osrf/gazebo/pull-request/858)
1. Fixed sonar test and deprecation warning.
    * [Pull request #856](https://bitbucket.org/osrf/gazebo/pull-request/856)
1. Speed up test compilation.
    * Part of [Issue #955](https://bitbucket.org/osrf/gazebo/issue/955)
    * [Pull request #846](https://bitbucket.org/osrf/gazebo/pull-request/846)
1. Added Joint::SetEffortLimit API
    * [Issue #923](https://bitbucket.org/osrf/gazebo/issue/923)
    * [Pull request #808](https://bitbucket.org/osrf/gazebo/pull-request/808)
1. Made bullet output less verbose.
    * [Pull request #839](https://bitbucket.org/osrf/gazebo/pull-request/839)
1. Convergence acceleration and stability tweak to make atlas_v3 stable
    * [Issue #895](https://bitbucket.org/osrf/gazebo/issue/895)
    * [Pull request #772](https://bitbucket.org/osrf/gazebo/pull-request/772)
1. Added colors, textures and world files for the SPL RoboCup environment
    * [Pull request #838](https://bitbucket.org/osrf/gazebo/pull-request/838)
1. Fixed bitbucket_pullrequests tool to work with latest BitBucket API.
    * [Issue #933](https://bitbucket.org/osrf/gazebo/issue/933)
    * [Pull request #841](https://bitbucket.org/osrf/gazebo/pull-request/841)
1. Fixed cppcheck warnings.
    * [Pull request #842](https://bitbucket.org/osrf/gazebo/pull-request/842)

### Gazebo 2.1.0 (2013-11-08)
1. Fix mainwindow unit test
    * [Pull request #752](https://bitbucket.org/osrf/gazebo/pull-request/752)
1. Visualize moment of inertia
    * Pull request [#745](https://bitbucket.org/osrf/gazebo/pull-request/745), [#769](https://bitbucket.org/osrf/gazebo/pull-request/769), [#787](https://bitbucket.org/osrf/gazebo/pull-request/787)
    * [Issue #203](https://bitbucket.org/osrf/gazebo/issue/203)
1. Update tool to count lines of code
    * [Pull request #758](https://bitbucket.org/osrf/gazebo/pull-request/758)
1. Implement World::Clear
    * Pull request [#785](https://bitbucket.org/osrf/gazebo/pull-request/785), [#804](https://bitbucket.org/osrf/gazebo/pull-request/804)
1. Improve Bullet support
    * [Pull request #805](https://bitbucket.org/osrf/gazebo/pull-request/805)
1. Fix doxygen spacing
    * [Pull request #740](https://bitbucket.org/osrf/gazebo/pull-request/740)
1. Add tool to generate model images for thepropshop.org
    * [Pull request #734](https://bitbucket.org/osrf/gazebo/pull-request/734)
1. Added paging support for terrains
    * [Pull request #707](https://bitbucket.org/osrf/gazebo/pull-request/707)
1. Added plugin path to LID_LIBRARY_PATH in setup.sh
    * [Pull request #750](https://bitbucket.org/osrf/gazebo/pull-request/750)
1. Fix for OSX
    * [Pull request #766](https://bitbucket.org/osrf/gazebo/pull-request/766)
    * [Pull request #786](https://bitbucket.org/osrf/gazebo/pull-request/786)
    * [Issue #906](https://bitbucket.org/osrf/gazebo/issue/906)
1. Update copyright information
    * [Pull request #771](https://bitbucket.org/osrf/gazebo/pull-request/771)
1. Enable screen dependent tests
    * [Pull request #764](https://bitbucket.org/osrf/gazebo/pull-request/764)
    * [Issue #811](https://bitbucket.org/osrf/gazebo/issue/811)
1. Fix gazebo command line help message
    * [Pull request #775](https://bitbucket.org/osrf/gazebo/pull-request/775)
    * [Issue #898](https://bitbucket.org/osrf/gazebo/issue/898)
1. Fix man page test
    * [Pull request #774](https://bitbucket.org/osrf/gazebo/pull-request/774)
1. Improve load time by reducing calls to RTShader::Update
    * [Pull request #773](https://bitbucket.org/osrf/gazebo/pull-request/773)
    * [Issue #877](https://bitbucket.org/osrf/gazebo/issue/877)
1. Fix joint visualization
    * [Pull request #776](https://bitbucket.org/osrf/gazebo/pull-request/776)
    * [Pull request #802](https://bitbucket.org/osrf/gazebo/pull-request/802)
    * [Issue #464](https://bitbucket.org/osrf/gazebo/issue/464)
1. Add helpers to fix NaN
    * [Pull request #742](https://bitbucket.org/osrf/gazebo/pull-request/742)
1. Fix model resizing via the GUI
    * [Pull request #763](https://bitbucket.org/osrf/gazebo/pull-request/763)
    * [Issue #885](https://bitbucket.org/osrf/gazebo/issue/885)
1. Simplify gzlog test by using sha1
    * [Pull request #781](https://bitbucket.org/osrf/gazebo/pull-request/781)
    * [Issue #837](https://bitbucket.org/osrf/gazebo/issue/837)
1. Enable cppcheck for header files
    * [Pull request #782](https://bitbucket.org/osrf/gazebo/pull-request/782)
    * [Issue #907](https://bitbucket.org/osrf/gazebo/issue/907)
1. Fix broken regression test
    * [Pull request #784](https://bitbucket.org/osrf/gazebo/pull-request/784)
    * [Issue #884](https://bitbucket.org/osrf/gazebo/issue/884)
1. All simbody and dart to pass tests
    * [Pull request #790](https://bitbucket.org/osrf/gazebo/pull-request/790)
    * [Issue #873](https://bitbucket.org/osrf/gazebo/issue/873)
1. Fix camera rotation from SDF
    * [Pull request #789](https://bitbucket.org/osrf/gazebo/pull-request/789)
    * [Issue #920](https://bitbucket.org/osrf/gazebo/issue/920)
1. Fix bitbucket pullrequest command line tool to match new API
    * [Pull request #803](https://bitbucket.org/osrf/gazebo/pull-request/803)
1. Fix transceiver spawn errors in tests
    * [Pull request #811](https://bitbucket.org/osrf/gazebo/pull-request/811)
    * [Pull request #814](https://bitbucket.org/osrf/gazebo/pull-request/814)

### Gazebo 2.0.0 (2013-10-08)
1. Refactor code check tool.
    * [Pull Request #669](https://bitbucket.org/osrf/gazebo/pull-request/669)
1. Added pull request tool for Bitbucket.
    * [Pull Request #670](https://bitbucket.org/osrf/gazebo/pull-request/670)
    * [Pull Request #691](https://bitbucket.org/osrf/gazebo/pull-request/671)
1. New wireless receiver and transmitter sensor models.
    * [Pull Request #644](https://bitbucket.org/osrf/gazebo/pull-request/644)
    * [Pull Request #675](https://bitbucket.org/osrf/gazebo/pull-request/675)
    * [Pull Request #727](https://bitbucket.org/osrf/gazebo/pull-request/727)
1. Audio support using OpenAL.
    * [Pull Request #648](https://bitbucket.org/osrf/gazebo/pull-request/648)
    * [Pull Request #704](https://bitbucket.org/osrf/gazebo/pull-request/704)
1. Simplify command-line parsing of gztopic echo output.
    * [Pull Request #674](https://bitbucket.org/osrf/gazebo/pull-request/674)
    * Resolves: [Issue #795](https://bitbucket.org/osrf/gazebo/issue/795)
1. Use UNIX directories through the user of GNUInstallDirs cmake module.
    * [Pull Request #676](https://bitbucket.org/osrf/gazebo/pull-request/676)
    * [Pull Request #681](https://bitbucket.org/osrf/gazebo/pull-request/681)
1. New GUI interactions for object manipulation.
    * [Pull Request #634](https://bitbucket.org/osrf/gazebo/pull-request/634)
1. Fix for OSX menubar.
    * [Pull Request #677](https://bitbucket.org/osrf/gazebo/pull-request/677)
1. Remove internal SDF directories and dependencies.
    * [Pull Request #680](https://bitbucket.org/osrf/gazebo/pull-request/680)
1. Add minimum version for sdformat.
    * [Pull Request #682](https://bitbucket.org/osrf/gazebo/pull-request/682)
    * Resolves: [Issue #818](https://bitbucket.org/osrf/gazebo/issue/818)
1. Allow different gtest parameter types with ServerFixture
    * [Pull Request #686](https://bitbucket.org/osrf/gazebo/pull-request/686)
    * Resolves: [Issue #820](https://bitbucket.org/osrf/gazebo/issue/820)
1. GUI model scaling when using Bullet.
    * [Pull Request #683](https://bitbucket.org/osrf/gazebo/pull-request/683)
1. Fix typo in cmake config.
    * [Pull Request #694](https://bitbucket.org/osrf/gazebo/pull-request/694)
    * Resolves: [Issue #824](https://bitbucket.org/osrf/gazebo/issue/824)
1. Remove gazebo include subdir from pkgconfig and cmake config.
    * [Pull Request #691](https://bitbucket.org/osrf/gazebo/pull-request/691)
1. Torsional spring demo
    * [Pull Request #693](https://bitbucket.org/osrf/gazebo/pull-request/693)
1. Remove repeated call to SetAxis in Joint.cc
    * [Pull Request #695](https://bitbucket.org/osrf/gazebo/pull-request/695)
    * Resolves: [Issue #823](https://bitbucket.org/osrf/gazebo/issue/823)
1. Add test for rotational joints.
    * [Pull Request #697](https://bitbucket.org/osrf/gazebo/pull-request/697)
    * Resolves: [Issue #820](https://bitbucket.org/osrf/gazebo/issue/820)
1. Fix compilation of tests using Joint base class
    * [Pull Request #701](https://bitbucket.org/osrf/gazebo/pull-request/701)
1. Terrain paging implemented.
    * [Pull Request #687](https://bitbucket.org/osrf/gazebo/pull-request/687)
1. Improve timeout error reporting in ServerFixture
    * [Pull Request #705](https://bitbucket.org/osrf/gazebo/pull-request/705)
1. Fix mouse picking for cases where visuals overlap with the laser
    * [Pull Request #709](https://bitbucket.org/osrf/gazebo/pull-request/709)
1. Fix string literals for OSX
    * [Pull Request #712](https://bitbucket.org/osrf/gazebo/pull-request/712)
    * Resolves: [Issue #803](https://bitbucket.org/osrf/gazebo/issue/803)
1. Support for ENABLE_TESTS_COMPILATION cmake parameter
    * [Pull Request #708](https://bitbucket.org/osrf/gazebo/pull-request/708)
1. Updated system gui plugin
    * [Pull Request #702](https://bitbucket.org/osrf/gazebo/pull-request/702)
1. Fix force torque unit test issue
    * [Pull Request #673](https://bitbucket.org/osrf/gazebo/pull-request/673)
    * Resolves: [Issue #813](https://bitbucket.org/osrf/gazebo/issue/813)
1. Use variables to control auto generation of CFlags
    * [Pull Request #699](https://bitbucket.org/osrf/gazebo/pull-request/699)
1. Remove deprecated functions.
    * [Pull Request #715](https://bitbucket.org/osrf/gazebo/pull-request/715)
1. Fix typo in `Camera.cc`
    * [Pull Request #719](https://bitbucket.org/osrf/gazebo/pull-request/719)
    * Resolves: [Issue #846](https://bitbucket.org/osrf/gazebo/issue/846)
1. Performance improvements
    * [Pull Request #561](https://bitbucket.org/osrf/gazebo/pull-request/561)
1. Fix gripper model.
    * [Pull Request #713](https://bitbucket.org/osrf/gazebo/pull-request/713)
    * Resolves: [Issue #314](https://bitbucket.org/osrf/gazebo/issue/314)
1. First part of Simbody integration
    * [Pull Request #716](https://bitbucket.org/osrf/gazebo/pull-request/716)

## Gazebo 1.9

### Gazebo 1.9.6 (2014-04-29)

1. Refactored inertia ratio reduction for ODE
    * [Pull request #1114](https://bitbucket.org/osrf/gazebo/pull-request/1114)
1. Improved collada loading performance
    * [Pull request #1075](https://bitbucket.org/osrf/gazebo/pull-request/1075)

### Gazebo 1.9.3 (2014-01-10)

1. Add thickness to plane to remove shadow flickering.
    * [Pull request #886](https://bitbucket.org/osrf/gazebo/pull-request/886)
1. Temporary GUI shadow toggle fix.
    * [Issue #925](https://bitbucket.org/osrf/gazebo/issue/925)
    * [Pull request #868](https://bitbucket.org/osrf/gazebo/pull-request/868)
1. Fix memory access bugs with libc++ on mavericks.
    * [Issue #965](https://bitbucket.org/osrf/gazebo/issue/965)
    * [Pull request #857](https://bitbucket.org/osrf/gazebo/pull-request/857)
    * [Pull request #881](https://bitbucket.org/osrf/gazebo/pull-request/881)
1. Replaced printf with cout in gztopic hz.
    * [Issue #969](https://bitbucket.org/osrf/gazebo/issue/969)
    * [Pull request #854](https://bitbucket.org/osrf/gazebo/pull-request/854)
1. Add Dark grey material and fix indentation.
    * [Pull request #851](https://bitbucket.org/osrf/gazebo/pull-request/851)
1. Fixed sonar sensor unit test.
    * [Pull request #848](https://bitbucket.org/osrf/gazebo/pull-request/848)
1. Convergence acceleration and stability tweak to make atlas_v3 stable.
    * [Pull request #845](https://bitbucket.org/osrf/gazebo/pull-request/845)
1. Update gtest to 1.7.0 to resolve problems with libc++.
    * [Issue #947](https://bitbucket.org/osrf/gazebo/issue/947)
    * [Pull request #827](https://bitbucket.org/osrf/gazebo/pull-request/827)
1. Fixed LD_LIBRARY_PATH for plugins.
    * [Issue #957](https://bitbucket.org/osrf/gazebo/issue/957)
    * [Pull request #844](https://bitbucket.org/osrf/gazebo/pull-request/844)
1. Fix transceiver sporadic errors.
    * Backport of [pull request #811](https://bitbucket.org/osrf/gazebo/pull-request/811)
    * [Pull request #836](https://bitbucket.org/osrf/gazebo/pull-request/836)
1. Modified the MsgTest to be deterministic with time checks.
    * [Pull request #843](https://bitbucket.org/osrf/gazebo/pull-request/843)
1. Fixed seg fault in LaserVisual.
    * [Issue #950](https://bitbucket.org/osrf/gazebo/issue/950)
    * [Pull request #832](https://bitbucket.org/osrf/gazebo/pull-request/832)
1. Implemented the option to disable tests that need a working screen to run properly.
    * Backport of [Pull request #764](https://bitbucket.org/osrf/gazebo/pull-request/764)
    * [Pull request #837](https://bitbucket.org/osrf/gazebo/pull-request/837)
1. Cleaned up gazebo shutdown.
    * [Pull request #829](https://bitbucket.org/osrf/gazebo/pull-request/829)
1. Fixed bug associated with loading joint child links.
    * [Issue #943](https://bitbucket.org/osrf/gazebo/issue/943)
    * [Pull request #820](https://bitbucket.org/osrf/gazebo/pull-request/820)

### Gazebo 1.9.2 (2013-11-08)
1. Fix enable/disable sky and clouds from SDF
    * [Pull request #809](https://bitbucket.org/osrf/gazebo/pull-request/809])
1. Fix occasional blank GUI screen on startup
    * [Pull request #815](https://bitbucket.org/osrf/gazebo/pull-request/815])
1. Fix GPU laser when interacting with heightmaps
    * [Pull request #796](https://bitbucket.org/osrf/gazebo/pull-request/796])
1. Added API/ABI checker command line tool
    * [Pull request #765](https://bitbucket.org/osrf/gazebo/pull-request/765])
1. Added gtest version information
    * [Pull request #801](https://bitbucket.org/osrf/gazebo/pull-request/801])
1. Fix GUI world saving
    * [Pull request #806](https://bitbucket.org/osrf/gazebo/pull-request/806])
1. Enable anti-aliasing for camera sensor
    * [Pull request #800](https://bitbucket.org/osrf/gazebo/pull-request/800])
1. Make sensor noise deterministic
    * [Pull request #788](https://bitbucket.org/osrf/gazebo/pull-request/788])
1. Fix build problem
    * [Issue #901](https://bitbucket.org/osrf/gazebo/issue/901)
    * [Pull request #778](https://bitbucket.org/osrf/gazebo/pull-request/778])
1. Fix a typo in Camera.cc
    * [Pull request #720](https://bitbucket.org/osrf/gazebo/pull-request/720])
    * [Issue #846](https://bitbucket.org/osrf/gazebo/issue/846)
1. Fix OSX menu bar
    * [Pull request #688](https://bitbucket.org/osrf/gazebo/pull-request/688])
1. Fix gazebo::init by calling sdf::setFindCallback() before loading the sdf in gzfactory.
    * [Pull request #678](https://bitbucket.org/osrf/gazebo/pull-request/678])
    * [Issue #817](https://bitbucket.org/osrf/gazebo/issue/817)

### Gazebo 1.9.1 (2013-08-20)
* Deprecate header files that require case-sensitive filesystem (e.g. Common.hh, Physics.hh) [https://bitbucket.org/osrf/gazebo/pull-request/638/fix-for-775-deprecate-headers-that-require]
* Initial support for building on Mac OS X [https://bitbucket.org/osrf/gazebo/pull-request/660/osx-support-for-gazebo-19] [https://bitbucket.org/osrf/gazebo/pull-request/657/cmake-fixes-for-osx]
* Fixes for various issues [https://bitbucket.org/osrf/gazebo/pull-request/635/fix-for-issue-792/diff] [https://bitbucket.org/osrf/gazebo/pull-request/628/allow-scoped-and-non-scoped-joint-names-to/diff] [https://bitbucket.org/osrf/gazebo/pull-request/636/fix-build-dependency-in-message-generation/diff] [https://bitbucket.org/osrf/gazebo/pull-request/639/make-the-unversioned-setupsh-a-copy-of-the/diff] [https://bitbucket.org/osrf/gazebo/pull-request/650/added-missing-lib-to-player-client-library/diff] [https://bitbucket.org/osrf/gazebo/pull-request/656/install-gzmode_create-without-sh-suffix/diff]

### Gazebo 1.9.0 (2013-07-23)
* Use external package [sdformat](https://bitbucket.org/osrf/sdformat) for sdf parsing, refactor the `Element::GetValue*` function calls, and deprecate Gazebo's internal sdf parser [https://bitbucket.org/osrf/gazebo/pull-request/627]
* Improved ROS support ([[Tutorials#ROS_Integration |documentation here]]) [https://bitbucket.org/osrf/gazebo/pull-request/559]
* Added Sonar, Force-Torque, and Tactile Pressure sensors [https://bitbucket.org/osrf/gazebo/pull-request/557], [https://bitbucket.org/osrf/gazebo/pull-request/567]
* Add compile-time defaults for environment variables so that sourcing setup.sh is unnecessary in most cases [https://bitbucket.org/osrf/gazebo/pull-request/620]
* Enable user camera to follow objects in client window [https://bitbucket.org/osrf/gazebo/pull-request/603]
* Install protobuf message files for use in custom messages [https://bitbucket.org/osrf/gazebo/pull-request/614]
* Change default compilation flags to improve debugging [https://bitbucket.org/osrf/gazebo/pull-request/617]
* Change to supported relative include paths [https://bitbucket.org/osrf/gazebo/pull-request/594]
* Fix display of laser scans when sensor is rotated [https://bitbucket.org/osrf/gazebo/pull-request/599]

## Gazebo 1.8

### Gazebo 1.8.7 (2013-07-16)
* Fix bug in URDF parsing of Vector3 elements [https://bitbucket.org/osrf/gazebo/pull-request/613]
* Fix compilation errors with newest libraries [https://bitbucket.org/osrf/gazebo/pull-request/615]

### Gazebo 1.8.6 (2013-06-07)
* Fix inertia lumping in the URDF parser[https://bitbucket.org/osrf/gazebo/pull-request/554]
* Fix for ODEJoint CFM damping sign error [https://bitbucket.org/osrf/gazebo/pull-request/586]
* Fix transport memory growth[https://bitbucket.org/osrf/gazebo/pull-request/584]
* Reduce log file data in order to reduce buffer growth that results in out of memory kernel errors[https://bitbucket.org/osrf/gazebo/pull-request/587]

### Gazebo 1.8.5 (2013-06-04)
* Fix Gazebo build for machines without a valid display.[https://bitbucket.org/osrf/gazebo/commits/37f00422eea03365b839a632c1850431ee6a1d67]

### Gazebo 1.8.4 (2013-06-03)
* Fix UDRF to SDF converter so that URDF gazebo extensions are applied to all collisions in a link.[https://bitbucket.org/osrf/gazebo/pull-request/579]
* Prevent transport layer from locking when a gzclient connects to a gzserver over a connection with high latency.[https://bitbucket.org/osrf/gazebo/pull-request/572]
* Improve performance and fix uninitialized conditional jumps.[https://bitbucket.org/osrf/gazebo/pull-request/571]

### Gazebo 1.8.3 (2013-06-03)
* Fix for gzlog hanging when gzserver is not present or not responsive[https://bitbucket.org/osrf/gazebo/pull-request/577]
* Fix occasional segfault when generating log files[https://bitbucket.org/osrf/gazebo/pull-request/575]
* Performance improvement to ODE[https://bitbucket.org/osrf/gazebo/pull-request/556]
* Fix node initialization[https://bitbucket.org/osrf/gazebo/pull-request/570]
* Fix GPU laser Hz rate reduction when sensor moved away from world origin[https://bitbucket.org/osrf/gazebo/pull-request/566]
* Fix incorrect lighting in camera sensors when GPU laser is subscribe to[https://bitbucket.org/osrf/gazebo/pull-request/563]

### Gazebo 1.8.2 (2013-05-28)
* ODE performance improvements[https://bitbucket.org/osrf/gazebo/pull-request/535][https://bitbucket.org/osrf/gazebo/pull-request/537]
* Fixed tests[https://bitbucket.org/osrf/gazebo/pull-request/538][https://bitbucket.org/osrf/gazebo/pull-request/541][https://bitbucket.org/osrf/gazebo/pull-request/542]
* Fixed sinking vehicle bug[https://bitbucket.org/osrf/drcsim/issue/300] in pull-request[https://bitbucket.org/osrf/gazebo/pull-request/538]
* Fix GPU sensor throttling[https://bitbucket.org/osrf/gazebo/pull-request/536]
* Reduce string comparisons for better performance[https://bitbucket.org/osrf/gazebo/pull-request/546]
* Contact manager performance improvements[https://bitbucket.org/osrf/gazebo/pull-request/543]
* Transport performance improvements[https://bitbucket.org/osrf/gazebo/pull-request/548]
* Reduce friction noise[https://bitbucket.org/osrf/gazebo/pull-request/545]

### Gazebo 1.8.1 (2013-05-22)
* Please note that 1.8.1 contains a bug[https://bitbucket.org/osrf/drcsim/issue/300] that causes interpenetration between objects in resting contact to grow slowly.  Please update to 1.8.2 for the patch.
* Added warm starting[https://bitbucket.org/osrf/gazebo/pull-request/529]
* Reduced console output[https://bitbucket.org/osrf/gazebo/pull-request/533]
* Improved off screen rendering performance[https://bitbucket.org/osrf/gazebo/pull-request/530]
* Performance improvements [https://bitbucket.org/osrf/gazebo/pull-request/535] [https://bitbucket.org/osrf/gazebo/pull-request/537]

### Gazebo 1.8.0 (2013-05-17)
* Fixed slider axis [https://bitbucket.org/osrf/gazebo/pull-request/527]
* Fixed heightmap shadows [https://bitbucket.org/osrf/gazebo/pull-request/525]
* Fixed model and canonical link pose [https://bitbucket.org/osrf/gazebo/pull-request/519]
* Fixed OSX message header[https://bitbucket.org/osrf/gazebo/pull-request/524]
* Added zlib compression for logging [https://bitbucket.org/osrf/gazebo/pull-request/515]
* Allow clouds to be disabled in cameras [https://bitbucket.org/osrf/gazebo/pull-request/507]
* Camera rendering performance [https://bitbucket.org/osrf/gazebo/pull-request/528]


## Gazebo 1.7

### Gazebo 1.7.3 (2013-05-08)
* Fixed log cleanup (again) [https://bitbucket.org/osrf/gazebo/pull-request/511/fix-log-cleanup-logic]

### Gazebo 1.7.2 (2013-05-07)
* Fixed log cleanup [https://bitbucket.org/osrf/gazebo/pull-request/506/fix-gzlog-stop-command-line]
* Minor documentation fix [https://bitbucket.org/osrf/gazebo/pull-request/488/minor-documentation-fix]

### Gazebo 1.7.1 (2013-04-19)
* Fixed tests
* IMU sensor receives time stamped data from links
* Fix saving image frames [https://bitbucket.org/osrf/gazebo/pull-request/466/fix-saving-frames/diff]
* Wireframe rendering in GUI [https://bitbucket.org/osrf/gazebo/pull-request/414/allow-rendering-of-models-in-wireframe]
* Improved logging performance [https://bitbucket.org/osrf/gazebo/pull-request/457/improvements-to-gzlog-filter-and-logging]
* Viscous mud model [https://bitbucket.org/osrf/gazebo/pull-request/448/mud-plugin/diff]

## Gazebo 1.6

### Gazebo 1.6.3 (2013-04-15)
* Fixed a [critical SDF bug](https://bitbucket.org/osrf/gazebo/pull-request/451)
* Fixed a [laser offset bug](https://bitbucket.org/osrf/gazebo/pull-request/449)

### Gazebo 1.6.2 (2013-04-14)
* Fix for fdir1 physics property [https://bitbucket.org/osrf/gazebo/pull-request/429/fixes-to-treat-fdir1-better-1-rotate-into/diff]
* Fix for force torque sensor [https://bitbucket.org/osrf/gazebo/pull-request/447]
* SDF documentation fix [https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match]

### Gazebo 1.6.1 (2013-04-05)
* Switch default build type to Release.

### Gazebo 1.6.0 (2013-04-05)
* Improvements to inertia in rubble pile
* Various Bullet integration advances.
* Noise models for ray, camera, and imu sensors.
* SDF 1.4, which accommodates more physics engine parameters and also some sensor noise models.
* Initial support for making movies from within Gazebo.
* Many performance improvements.
* Many bug fixes.
* Progress toward to building on OS X.

## Gazebo 1.5

### Gazebo 1.5.0 (2013-03-11)
* Partial integration of Bullet
  * Includes: cubes, spheres, cylinders, planes, meshes, revolute joints, ray sensors
* GUI Interface for log writing.
* Threaded sensors.
* Multi-camera sensor.

* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/236 Issue #236]
 * [https://bitbucket.org/osrf/gazebo/issue/507 Issue #507]
 * [https://bitbucket.org/osrf/gazebo/issue/530 Issue #530]
 * [https://bitbucket.org/osrf/gazebo/issue/279 Issue #279]
 * [https://bitbucket.org/osrf/gazebo/issue/529 Issue #529]
 * [https://bitbucket.org/osrf/gazebo/issue/239 Issue #239]
 * [https://bitbucket.org/osrf/gazebo/issue/5 Issue #5]

## Gazebo 1.4

### Gazebo 1.4.0 (2013-02-01)
* New Features:
 * GUI elements to display messages from the server.
 * Multi-floor building editor and creator.
 * Improved sensor visualizations.
 * Improved mouse interactions

* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/16 Issue #16]
 * [https://bitbucket.org/osrf/gazebo/issue/142 Issue #142]
 * [https://bitbucket.org/osrf/gazebo/issue/229 Issue #229]
 * [https://bitbucket.org/osrf/gazebo/issue/277 Issue #277]
 * [https://bitbucket.org/osrf/gazebo/issue/291 Issue #291]
 * [https://bitbucket.org/osrf/gazebo/issue/310 Issue #310]
 * [https://bitbucket.org/osrf/gazebo/issue/320 Issue #320]
 * [https://bitbucket.org/osrf/gazebo/issue/329 Issue #329]
 * [https://bitbucket.org/osrf/gazebo/issue/333 Issue #333]
 * [https://bitbucket.org/osrf/gazebo/issue/334 Issue #334]
 * [https://bitbucket.org/osrf/gazebo/issue/335 Issue #335]
 * [https://bitbucket.org/osrf/gazebo/issue/341 Issue #341]
 * [https://bitbucket.org/osrf/gazebo/issue/350 Issue #350]
 * [https://bitbucket.org/osrf/gazebo/issue/384 Issue #384]
 * [https://bitbucket.org/osrf/gazebo/issue/431 Issue #431]
 * [https://bitbucket.org/osrf/gazebo/issue/433 Issue #433]
 * [https://bitbucket.org/osrf/gazebo/issue/453 Issue #453]
 * [https://bitbucket.org/osrf/gazebo/issue/456 Issue #456]
 * [https://bitbucket.org/osrf/gazebo/issue/457 Issue #457]
 * [https://bitbucket.org/osrf/gazebo/issue/459 Issue #459]

## Gazebo 1.3

### Gazebo 1.3.1 (2012-12-14)
* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/297 Issue #297]
* Other bugs fixed:
 * [https://bitbucket.org/osrf/gazebo/pull-request/164/ Fix light bounding box to disable properly when deselected]
 * [https://bitbucket.org/osrf/gazebo/pull-request/169/ Determine correct local IP address, to make remote clients work properly]
 * Various test fixes

### Gazebo 1.3.0 (2012-12-03)
* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/233 Issue #233]
 * [https://bitbucket.org/osrf/gazebo/issue/238 Issue #238]
 * [https://bitbucket.org/osrf/gazebo/issue/2 Issue #2]
 * [https://bitbucket.org/osrf/gazebo/issue/95 Issue #95]
 * [https://bitbucket.org/osrf/gazebo/issue/97 Issue #97]
 * [https://bitbucket.org/osrf/gazebo/issue/90 Issue #90]
 * [https://bitbucket.org/osrf/gazebo/issue/253 Issue #253]
 * [https://bitbucket.org/osrf/gazebo/issue/163 Issue #163]
 * [https://bitbucket.org/osrf/gazebo/issue/91 Issue #91]
 * [https://bitbucket.org/osrf/gazebo/issue/245 Issue #245]
 * [https://bitbucket.org/osrf/gazebo/issue/242 Issue #242]
 * [https://bitbucket.org/osrf/gazebo/issue/156 Issue #156]
 * [https://bitbucket.org/osrf/gazebo/issue/78 Issue #78]
 * [https://bitbucket.org/osrf/gazebo/issue/36 Issue #36]
 * [https://bitbucket.org/osrf/gazebo/issue/104 Issue #104]
 * [https://bitbucket.org/osrf/gazebo/issue/249 Issue #249]
 * [https://bitbucket.org/osrf/gazebo/issue/244 Issue #244]

* New features:
 * Default camera view changed to look down at the origin from a height of 2 meters at location (5, -5, 2).
 * Record state data using the '-r' command line option, playback recorded state data using the '-p' command line option
 * Adjust placement of lights using the mouse.
 * Reduced the startup time.
 * Added visual reference for GUI mouse movements.
 * SDF version 1.3 released (changes from 1.2 listed below):
     - added `name` to `<camera name="cam_name"/>`
     - added `pose` to `<camera><pose>...</pose></camera>`
     - removed `filename` from `<mesh><filename>...</filename><mesh>`, use uri only.
     - recovered `provide_feedback` under `<joint>`, allowing calling `physics::Joint::GetForceTorque` in plugins.
     - added `imu` under `<sensor>`.

## Gazebo 1.2

### Gazebo 1.2.6 (2012-11-08)
* Fixed a transport issue with the GUI. Fixed saving the world via the GUI. Added more documentation. ([https://bitbucket.org/osrf/gazebo/pull-request/43/fixed-a-transport-issue-with-the-gui-fixed/diff pull request #43])
* Clean up mutex usage. ([https://bitbucket.org/osrf/gazebo/pull-request/54/fix-mutex-in-modellistwidget-using-boost/diff pull request #54])
* Fix OGRE path determination ([https://bitbucket.org/osrf/gazebo/pull-request/58/fix-ogre-paths-so-this-also-works-with/diff pull request #58], [https://bitbucket.org/osrf/gazebo/pull-request/68/fix-ogre-plugindir-determination/diff pull request #68])
* Fixed a couple of crashes and model selection/dragging problems ([https://bitbucket.org/osrf/gazebo/pull-request/59/fixed-a-couple-of-crashes-and-model/diff pull request #59])

### Gazebo 1.2.5 (2012-10-22)
* Step increment update while paused fixed ([https://bitbucket.org/osrf/gazebo/pull-request/45/fix-proper-world-stepinc-count-we-were/diff pull request #45])
* Actually call plugin destructors on shutdown ([https://bitbucket.org/osrf/gazebo/pull-request/51/fixed-a-bug-which-prevent-a-plugin/diff pull request #51])
* Don't crash on bad SDF input ([https://bitbucket.org/osrf/gazebo/pull-request/52/fixed-loading-of-bad-sdf-files/diff pull request #52])
* Fix cleanup of ray sensors on model deletion ([https://bitbucket.org/osrf/gazebo/pull-request/53/deleting-a-model-with-a-ray-sensor-did/diff pull request #53])
* Fix loading / deletion of improperly specified models ([https://bitbucket.org/osrf/gazebo/pull-request/56/catch-when-loading-bad-models-joint/diff pull request #56])

### Gazebo 1.2.4 (10-19-2012:08:00:52)
*  Style fixes ([https://bitbucket.org/osrf/gazebo/pull-request/30/style-fixes/diff pull request #30]).
*  Fix joint position control ([https://bitbucket.org/osrf/gazebo/pull-request/49/fixed-position-joint-control/diff pull request #49])

### Gazebo 1.2.3 (10-16-2012:18:39:54)
*  Disabled selection highlighting due to bug ([https://bitbucket.org/osrf/gazebo/pull-request/44/disabled-selection-highlighting-fixed/diff pull request #44]).
*  Fixed saving a world via the GUI.

### Gazebo 1.2.2 (10-16-2012:15:12:22)
*  Skip search for system install of libccd, use version inside gazebo ([https://bitbucket.org/osrf/gazebo/pull-request/39/skip-search-for-system-install-of-libccd/diff pull request #39]).
*  Fixed sensor initialization race condition ([https://bitbucket.org/osrf/gazebo/pull-request/42/fix-sensor-initializaiton-race-condition pull request #42]).

### Gazebo 1.2.1 (10-15-2012:21:32:55)
*  Properly removed projectors attached to deleted models ([https://bitbucket.org/osrf/gazebo/pull-request/37/remove-projectors-that-are-attached-to/diff pull request #37]).
*  Fix model plugin loading bug ([https://bitbucket.org/osrf/gazebo/pull-request/31/moving-bool-first-in-model-and-world pull request #31]).
*  Fix light insertion and visualization of models prior to insertion ([https://bitbucket.org/osrf/gazebo/pull-request/35/fixed-light-insertion-and-visualization-of/diff pull request #35]).
*  Fixed GUI manipulation of static objects ([https://bitbucket.org/osrf/gazebo/issue/63/moving-static-objects-does-not-move-the issue #63] [https://bitbucket.org/osrf/gazebo/pull-request/38/issue-63-bug-patch-moving-static-objects/diff pull request #38]).
*  Fixed GUI selection bug ([https://bitbucket.org/osrf/gazebo/pull-request/40/fixed-selection-of-multiple-objects-at/diff pull request #40])

### Gazebo 1.2.0 (10-04-2012:20:01:20)
*  Updated GUI: new style, improved mouse controls, and removal of non-functional items.
*  Model database: An online repository of models.
*  Numerous bug fixes
*  APT repository hosted at [http://osrfoundation.org OSRF]
*  Improved process control prevents zombie processes
