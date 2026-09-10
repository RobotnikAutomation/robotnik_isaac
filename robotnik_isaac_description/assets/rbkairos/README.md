# RB-KAIROS USD assets

## Imported USD assets

The only runtime assets are complete importer output trees:

- `usd/rbkairos_base/rbkairos_base.usda`
- `usd/rbkairos_plus/rbkairos_plus.usda`

Each root must remain beside its `payloads/` directory.  Never copy, move or
reference a root USD independently of that directory: materials, geometries
and physics layers use relative USD references.

## Pure URDF assets

`urdf/rbkairos_base.urdf`, `urdf/ur10e.urdf` and
`urdf/rbkairos_plus.urdf` are xacro-free import inputs. Their mesh URIs are
relative and point to the copied `urdf/meshes/` directory, so Isaac Sim can
import them without resolving `package://` URIs.

Regenerate them after changing an upstream reference xacro or mesh:

```bash
python3 src/robotnik_isaac_sim/robotnik_isaac_description/scripts/generate_urdfs.py \
  --workspace "$PWD"
```
