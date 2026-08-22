# Voxel51 Playroom real-capture 3DGS provenance

## Source

- Dataset: `Voxel51/gaussian_splatting`
- Dataset author: Paula Ramos
- Dataset page: <https://huggingface.co/datasets/Voxel51/gaussian_splatting>
- Scene: `FO_dataset/playroom`
- Upstream model: `iteration_30000/point_cloud.ply`
- Upstream URL: <https://huggingface.co/datasets/Voxel51/gaussian_splatting/resolve/main/FO_dataset/playroom/point_cloud/iteration_30000/point_cloud.ply>
- License declared by the dataset page: Apache-2.0
- Upstream bytes: `475,263,524`
- Upstream Gaussian records: `1,916,379`
- Upstream SHA-256: `c6fddedf6c7b412d078bbbaa1826e7a1b258f75f862c5190dc50a646243d7d9e`

The scene is the real-world Deep Blending Playroom represented by the official
3D Gaussian Splatting method. Reference images `DSC05572.jpg` and
`DSC05573.jpg` on the dataset page show the desk, bookshelves, wall, and floor.

## Deterministic derivative

`tools/prepare_voxel51_playroom_3dgs.py` verifies the complete upstream byte
length and SHA-256. It retains records whose zero-based index is divisible by
six. For every retained Gaussian it copies the following little-endian float
values byte-for-byte:

- position (`x`, `y`, `z`)
- DC colour (`f_dc_0` through `f_dc_2`)
- opacity
- anisotropic scale
- rotation quaternion

Normals and the 45 view-dependent spherical-harmonic floats are omitted. The
RNE loader deterministically supplies zero for those optional properties. No
position, colour, opacity, scale, or rotation is synthesized or fitted.

- Derivative: `playroom_dc_every6.ply`
- Derivative Gaussian records: `319,397`
- Derivative bytes: `17,886,594`
- Derivative SHA-256: `88f4ebffee1fdb1f558625b23fb93ad4c257a1d7dae5dc00443596c390717022`

## Calibration and simulation frame

The published Graphdeco Tanks & Temples + Deep Blending COLMAP bundle supplies
the Playroom camera calibration. Cameras `DSC05572.jpg` and `DSC05573.jpg`
anchor the reconstruction orientation and scale; their poses are converted
from COLMAP's right/down/forward camera frame to RNE's right/up/back frame.

The manifest rotates the second camera's measured up vector onto RNE `+Y`,
scales the reconstruction by `0.35`, and translates it by `+1.4 m` in Y. The
resulting floor proxy is `y = 0 m`; the robot, payload, ground contact, and room
collision proxies all use that same simulation frame.

## Reproduction

```text
python tools/prepare_voxel51_playroom_3dgs.py
python tools/prepare_voxel51_playroom_3dgs.py --check
```
