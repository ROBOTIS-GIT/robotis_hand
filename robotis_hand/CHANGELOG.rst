^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotis_hand
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-04-13)
------------------
* Added tactile sensor topic broadcaster
* Updated controller and input topic names for consistency
* Adjusted joint2 position limits to better emulate human range of motion
* Added VR support for HX5
* Contributors: Wonho Yun, Howon Kim, Hyunwoo Nam

0.0.3 (2026-03-18)
------------------
* Updated Docker image tag
* Added version print notice to Docker container
* Removed rmw_zenoh from Dockerfile
* Contributors: Hyunwoo Nam

0.0.2 (2026-01-14)
------------------
* Fixed moveit_config srdf files for correct disable_collisions settings
* Fixed robot description files for hx5_d20_rev2
* Contributors: Hyunwoo Nam

0.0.1 (2025-11-27)
------------------
* Added bringup scripts for system initialization
* Added robot description files for visualization and planning
* Added MoveIt for motion planning support
* Contributors: Woojin Wie, Hyunwoo Nam
