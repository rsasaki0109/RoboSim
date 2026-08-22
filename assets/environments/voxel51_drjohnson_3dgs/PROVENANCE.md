# Voxel51 Dr Johnson real-capture 3DGS provenance

## Source

- Dataset: `Voxel51/gaussian_splatting`
- Dataset author: Paula Ramos
- Dataset page: <https://huggingface.co/datasets/Voxel51/gaussian_splatting>
- Scene: `FO_dataset/drjohnson`
- Upstream model: `iteration_30000/point_cloud.ply`
- Upstream URL: <https://huggingface.co/datasets/Voxel51/gaussian_splatting/resolve/main/FO_dataset/drjohnson/point_cloud/iteration_30000/point_cloud.ply>
- License declared by the dataset page: Apache-2.0
- Upstream bytes: `788,034,924`
- Upstream Gaussian records: `3,177,554`
- Upstream SHA-256: `92f4898839ec4ad7f197cf6c74b89918b35ea712b4e41435593ccb152d22b7f5`

Dr Johnson is a real indoor Deep Blending capture reconstructed with the
official 3D Gaussian Splatting method. Published frames `IMG_6292.jpg` and
`IMG_6293.jpg` show the same green-walled room, wood floor, rug, radiator,
chairs, window, and doors represented by the committed splats.

## Deterministic derivative

`tools/prepare_voxel51_drjohnson_3dgs.py` verifies the complete upstream byte
length and SHA-256. It retains records whose zero-based index is divisible by
ten and copies position, DC colour, opacity, anisotropic scale, and rotation
floats byte-for-byte. No splat geometry or colour is synthesized.

- Derivative: `drjohnson_dc_every10.ply`
- Derivative Gaussian records: `317,756`
- Derivative bytes: `17,794,698`
- Derivative SHA-256: `f357a929801db2be75574c47205479c53a6bf71686af3f4bf8c1641db3688663`

## Calibration and simulation frame

The published Deep Blending COLMAP bundle supplies measured poses for
`IMG_6292.jpg` and `IMG_6293.jpg`. The manifest rotates the second camera's
measured up vector onto RNE `+Y`. The dominant captured wood-floor plane is
translated to `y = 0 m`; the reconstruction scale is retained because its
camera-to-floor height is already metric-scale. Robot bodies, task furniture,
collision proxies, and splats consequently share one world frame.

## Reproduction

```text
python tools/prepare_voxel51_drjohnson_3dgs.py
python tools/prepare_voxel51_drjohnson_3dgs.py --check
```
