# Poly Haven warehouse props

Scanned props and one surface texture used to dress the warehouse examples
(`examples/123_warehouse_logistics`, `examples/125_warehouse_relay`).

- Source: <https://polyhaven.com> (models and textures), fetched through the
  public API at 1k texture resolution in glTF form.
- License: **CC0 1.0** (public domain dedication) for every asset here, as
  Poly Haven publishes all of its assets. Authors are credited per model in
  `manifest.json`.
- `manifest.json` pins the SHA-256 of every downloaded file.
- One edit after download: `rollershutter_door` ships a second, graffiti-
  covered copy two meters to the side of the door; its node is removed from
  the glTF scene. The pinned hash is of the file as downloaded.

Rebuild or verify:

```bash
python3 tools/prepare_polyhaven_warehouse.py          # verify against the manifest
python3 tools/prepare_polyhaven_warehouse.py --pin    # re-pin after changing the list
```
