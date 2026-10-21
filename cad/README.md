# CAD

SolidWorks files are binary, so git can't merge them. Tell the team before you edit a shared part, and pull before you open anything.

Each assembly is in the same folder as its parts. SolidWorks looks for referenced parts in the assembly's own folder, so **move an assembly and its parts together** or the references break.

| Folder | Contents |
|---|---|
| `rocket_assembly/` | `Rocket_Assembly.SLDASM`: body tube, pivot, bearing. `CAD_Equations.txt` holds the global dimensions (inches). |
| `gimbal_prototype/` | `TVCSimplifiedTLA.SLDASM`: simplified gimbal prototype with the DS3218MG servo, EDF mock, and McMaster hardware. |
| `edf_coupling_prototype/` | `edfassembly prototype.SLDASM`: EDF model, motor end piece, servo mock-up, and the EDF-to-servo coupling. Its STL exports are in this folder too, not in `stl/`. |
| `flat_body_prototype/` | `FlatRocketPrototype.SLDASM`: flat-plate body prototype. |
| `stl/` | Exported STLs for 3D printing. |
