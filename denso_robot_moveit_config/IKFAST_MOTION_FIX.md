# Camera-arm motion investigation

## What was wrong

The left camera robot made unnecessarily large movements for simple pose goals. J1 could take the long direction, while J4/J6 sometimes selected equivalent wrist configurations such as `-270°` instead of `+90°`.

This was not caused by the TF alignment calculation alone. There were three contributing issues:

1. The numerical KDL IK solver could return different valid joint branches for the same Cartesian pose.
2. MoveIt's OMPL pose sampling supplied randomized IK seeds, so ranking a solution only against the supplied seed did not reliably select the branch closest to the robot's real state.
3. The alignment goal requested camera roll `0°`. Because of the camera's mounting transform, that exact orientation required approximately `90°` of J6 rotation even when the viewing direction looked correct.

There was also an unrelated ROS discovery timing problem: short-lived alignment processes sometimes failed while reading `robot_description` from `/move_group/get_parameters`.

## What changed

- Both VS050 arms now instantiate the same analytic IKFast solver with separate link prefixes.
- IKFast ranks all valid branches against the actual `/joint_states` position, with extra cost on large J1, J4 and J6 travel.
- OMPL explicitly uses the configured `RRTConnect` profile for both arms.
- The three-point alignment test now requests the correct mounted camera roll of `-90°`.
- Both alignment executables retry transient `move_group` parameter-service failures.
- RViz uses the live left-camera topic, and the simulated camera rate was reduced to improve performance.

## Result

Repeated plans no longer alternate between equivalent `+90°` and `-270°` wrist branches. In the three-point simulation, the left-arm J6 endpoints were approximately `28.8°`, `89.6°`, and `31.4°`.

Some wrist movement is still expected when the phone holder orientation changes. If image roll does not matter, the next improvement should be a direction-only camera constraint that keeps the optical axis aimed at the phone while leaving roll free.
