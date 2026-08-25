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

The official INRIA Deep Blending source archive supplies the COLMAP camera and
point reconstruction used to register `IMG_6292.jpg` and `IMG_6293.jpg`:

- Archive: <https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/datasets/input/tandt_db.zip>
- Archive bytes: `682,628,995`
- Archive SHA-256: `816e62f22a161abbfe841d2a6b10cdf036e297c9fa289b3bfeee9c6ec526d7e1`
- Camera model: `PINHOLE`, `1332 x 876`, `fx = 1035.496599 px`,
  `fy = 1034.971864 px`, `cx = 666 px`, `cy = 438 px`

`drjohnson.validation.json` binds those source hashes, two retained real
reference frames, their COLMAP intrinsics/extrinsics, six registered semantic
landmarks, the dominant floor plane, the splat manifest, and the derivative
PLY. The manifest rotates the registered camera up direction onto RNE `+Y` and
translates the dominant floor plane to `y = 0` reconstruction units. Its
337-point inlier plane has a claimed height of `0.01894` and RMSE `0.01606`
against the declared `0.03` tolerance.

The pickup support and payload are centered at the registered
`rug_front_center` landmark. The support top center projects to the manually
retained rug polygon in `IMG_6293.reference.jpg`, so collision geometry no
longer occupies arbitrary empty room space.

This is deliberately **not yet a qualifying metric calibration**. COLMAP
reconstruction units are only defined up to scale, and the archive contains no
independently measured physical length. Plausible camera height is not accepted
as a scale anchor. A registered RNE render versus the real reference image is
also still missing. The fixture therefore passes four of six contracts and
reports these two as missing:

- `independent_metric_scale_anchor`
- `real_sim_observation_comparison`

## Reproduction

```text
python tools/prepare_voxel51_drjohnson_3dgs.py
python tools/prepare_voxel51_drjohnson_3dgs.py --check
python tools/prepare_drjohnson_validation_fixture.py --source-archive E:\RNE-tools\tandt_db.zip
python tools/prepare_drjohnson_validation_fixture.py --source-archive E:\RNE-tools\tandt_db.zip --check
```
