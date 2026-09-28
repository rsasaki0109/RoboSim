# Poly Haven racing props

Scanned props and surface textures used to build the race circuit
(`examples/128_car_race`).

- Source: <https://polyhaven.com> (models and textures), fetched through the
  public API at 1k texture resolution in glTF form.
- License: **CC0 1.0** (public domain dedication) for every asset here, as
  Poly Haven publishes all of its assets. Authors are credited per asset in
  `manifest.json`.
- `manifest.json` pins the SHA-256 of every downloaded file. Nothing is edited
  after download.

Rebuild or verify:

```bash
python3 tools/prepare_polyhaven_warehouse.py --set racing          # verify
python3 tools/prepare_polyhaven_warehouse.py --set racing --pin    # re-pin after changing the list
```
