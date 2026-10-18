# Model asset attribution and licensing

This package is migrated from:

- Upstream repository: https://github.com/Polaris-zhj/ws_excavator
- Upstream source path: `src/excavator_description`
- Upstream `package.xml` declares `BSD-3-Clause`.
- Upstream Xacro comments identify the CAD geometry as originating from the ROS 1 `pudong` package in `ROS-based-autonomous-navigation-and-excavation-control-for-excavators`.

The upstream repository did not expose a root `LICENSE` file when this migration was prepared. The package-level license declaration and source attribution are preserved here. Verify the original CAD asset copyright and license text before redistributing the model outside the upstream project's stated terms.

All model meshes except `meshes/base_link.STL` were copied from the upstream package. The original `base_link.STL` is approximately 1.7 MB and the connected content-transfer API did not return its binary payload; the current file at that path is a generated, approximate chassis/track mesh. Replace it with the original asset by copying it from a local clone of the upstream repository. The URDF/Xacro references remain unchanged.

Planner and simulation code should reference the installed `excavator_description` package URI (`package://excavator_description/meshes/...`) rather than duplicating mesh files.
