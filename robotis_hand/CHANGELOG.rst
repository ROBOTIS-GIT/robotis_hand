^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotis_hand
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2026-04-29)
------------------
* Added CoP-based optimization grasping controller
* Added total force-based holding controller
* Added per-finger IK solver
* Added tactile pressure parsing, baseline compensation, filtering, and CoP calculation
* Added YAML configuration for key controller parameters
* Added tactile visualization script
* Added simple topic publish node for operation
* Updated controller and input topic names for consistency
* Contributors: Howon Kim

0.1.1 (2026-04-28)
------------------
* Added VR support for HX5
* Update macro names
* Contributors: Hyunwoo Nam

0.1.0 (2026-04-20)
------------------
* Added tactile sensor topic broadcaster
* Updated controller and input topic names for consistency
* Adjusted HX5 joint2 position limits
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
