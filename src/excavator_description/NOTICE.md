# Model asset attribution and licensing

This package is migrated from the upstream repository:

- https://github.com/Polaris-zhj/ws_excavator
- Upstream source path: `src/excavator_description`
- Upstream `package.xml` declares `BSD-3-Clause`.
- The upstream Xacro comments identify the CAD geometry as originating from the ROS 1 `pudong` package in the project `ROS-based-autonomous-navigation-and-excavation-control-for-excavators`.

The upstream repository did not expose a root `LICENSE` file when this migration was prepared. The package-level license declaration and source attribution are preserved here. Verify the original CAD asset copyright and license text before redistributing this model outside the upstream project's stated terms.

The robot description is installed as the `excavator_description` ROS 2 package. Planner and simulation code should reference the installed package URI (`package://excavator_description/meshes/...`) rather than duplicate mesh files.
