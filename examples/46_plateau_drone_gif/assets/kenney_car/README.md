# Kenney Car Kit subset

This directory contains a reproducible subset of **Kenney Car Kit 3.1**:

- `sedan-body.obj`: the `body` group extracted from `sedan.obj`;
- `wheel.obj`: the separate `wheel-default.obj`;
- `vehicle.mtl` and `colormap.png`: the upstream palette material;
- `colormap-red.png` and `colormap-blue.png`: deterministic body-palette variants.

The source archive is distributed by Kenney under Creative Commons Zero
(CC0). The upstream notice is preserved in `LICENSE.txt` with trailing
whitespace normalized.

- Source: <https://www.kenney.nl/assets/car-kit>
- Archive: `kenney_car-kit.zip`
- SHA-256: `fac7dacac5c7874348cf19729af3ef205f3d366493edaf0a827d93f4fdf3d0c4`

Rebuild the checked-in subset with Pillow installed:

```bash
python prepare_assets.py path/to/kenney_car-kit.zip
```
