# factory_materials

PBR textures used by the station and parcel models (`station_params.yaml → finishes`).
Referenced as `model://factory_materials/materials/textures/<name>_{albedo,normal,roughness}.jpg`.

All textures come from [ambientCG](https://ambientcg.com) and are licensed
**CC0 1.0 Universal** (public domain). They were downscaled and re-encoded by
`scripts/fetch_factory_textures.py`; run it again to regenerate them.

| Name          | ambientCG asset   | Size  | Used for                         |
|---------------|-------------------|-------|----------------------------------|
| `tread_plate` | `DiamondPlate001` | 1024² | station base plate               |
| `aluminium`   | `Metal009`        | 512²  | back plate and profile frame     |
| `rubber`      | `Rubber004`       | 512²  | ledges                           |
| `cardboard`   | `Cardboard004`    | 512²  | parcel body                      |

Maps: `Color` → albedo, `NormalGL` → normal (OpenGL convention, as ogre2 expects),
`Roughness` → roughness. Metalness maps are not used: without an environment map,
ogre2 renders metallic surfaces almost black, so metalness is a constant per finish.
