# Model asset attribution and licensing

This package is migrated from:

- Upstream repository: https://github.com/Polaris-zhj/ws_excavator
- Upstream source path: `src/excavator_description`
- Upstream `package.xml` declares `BSD-3-Clause`.
- Upstream Xacro comments identify the CAD geometry as originating from the ROS 1 `pudong` package in `ROS-based-autonomous-navigation-and-excavation-control-for-excavators`.

The upstream repository did not expose a root `LICENSE` file when this migration was prepared. The package-level license declaration and source attribution are preserved here. Verify the original CAD asset copyright and license text before redistributing the model outside the upstream project's stated terms.

All model meshes, including `meshes/base_link.STL`, are copied from the upstream package. The base chassis/track mesh is the original ~1.7 MB binary CAD asset (33,856 triangles). If a checkout needs to be re-synchronized with a local upstream clone:

```bash
bash scripts/replace_base_link_mesh.sh /path/to/ws_excavator
python3 scripts/convert_ascii_stl_to_binary.py src/excavator_description/meshes/base_link.STL
```

The second command is only needed if the upstream file is replaced by an ASCII STL; RViz's built-in loader requires binary STL.

Planner and simulation code should reference the installed `excavator_description` package URI (`package://excavator_description/meshes/...`) rather than duplicating mesh files.
