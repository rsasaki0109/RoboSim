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
four and copies position, DC colour, opacity, anisotropic scale, and rotation
floats byte-for-byte. No splat geometry or colour is synthesized. The 4:1
derivative is the smallest tested density that satisfies the registered
structural-observation limits; the previous 10:1 derivative did not.

- Derivative: `drjohnson_dc_every4.ply`
- Derivative Gaussian records: `794,389`
- Derivative bytes: `44,486,146`
- Derivative SHA-256: `9e1c89c18b6dd70f3f77ef1463983d86d34d859e118aec56d77394b36a41458f`

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

The retained same-camera RNE observation is generated at `1332 x 876` from the
exact `IMG_6293` pose and intrinsics. The validator re-decodes both images and
recomputes raw RGB PSNR (`13.046 dB`), luminance Pearson correlation (`0.9267`),
and gradient-magnitude Pearson correlation (`0.6879`) against fixed limits of
`12 dB`, `0.90`, and `0.65`. The report and rendered PNG are content-addressed;
the comparison cannot pass by editing the reported numbers alone.
The retained render identifies its execution adapter as an NVIDIA GeForce GTX
1660 Ti using Vulkan and NVIDIA driver `596.36`; GPU byte-level output is not
claimed portable, while the registered metric limits are.

`IMG_6293.depth.json` independently runs the checked-in Gaussian PLY through
RNE's deterministic proxy-depth path at the same registered camera. It binds
the PLY and camera calibration hashes plus the full `1332 x 876` depth-frame
hash. Proxy depth covers `80.08%` of the image and matches all six semantic
COLMAP landmarks with `0.179327` mean and `0.652182` maximum absolute error in
reconstruction units. The report deliberately stores no duplicate full depth
image and explicitly rejects a metre claim before physical scale exists.

This is deliberately **not yet a qualifying metric calibration**. COLMAP
reconstruction units are only defined up to scale, and the archive contains no
independently measured physical length. Plausible camera height is not accepted
as a scale anchor. The fixture therefore passes six of seven contracts and
reports one as missing:

- `independent_metric_scale_anchor`

The original Deep Blending reconstruction archive was also inspected by its
ZIP central directory: its 1,865 entries contain COLMAP inputs/outputs, source
images, refined depth maps, and a RealityCapture mesh, but no survey, control
point, GPS/XMP, measurement, or scale record. The missing result is therefore
explicit rather than inferred from an unavailable file.

`docs/3DGS_METRIC_ANCHOR.md` defines the field procedure and the fail-closed
intake path. `--metric-anchor` accepts only an independently authored record
whose two endpoints exactly match retained COLMAP observations, whose evidence
files are content-addressed, and whose measured scale agrees with the manifest
inside declared uncertainty. No placeholder anchor is committed.

## Reproduction

```text
python tools/prepare_voxel51_drjohnson_3dgs.py
python tools/prepare_voxel51_drjohnson_3dgs.py --check
cargo run -p rne_render_3dgs --example registered_splat_depth -- --manifest assets/environments/voxel51_drjohnson_3dgs/voxel51_drjohnson.rne.splat.toml --camera colmap.IMG_6293.jpg --output assets/environments/voxel51_drjohnson_3dgs/IMG_6293.depth.json
python tools/prepare_drjohnson_validation_fixture.py --source-archive E:\RNE-tools\tandt_db.zip
python tools/prepare_drjohnson_validation_fixture.py --source-archive E:\RNE-tools\tandt_db.zip --check
```
