# WakuFactory Sakura real-capture 3DGS provenance

`sakura1_every4.ply` is derived exclusively from a real scene captured by the
WakuFactory sample author with an iPad Pro (3rd generation) and Scaniverse. The
sample page identifies the PLY exports as unmodified Scaniverse output and
releases the data under CC0.

- Sample page: <https://www.wakufactory.jp/wxr/splats/sample.html>
- Upstream PLY: <https://www.wakufactory.jp/wxr/splats/data/sakura1.ply>
- Upstream bytes: `58,573,675`
- Upstream SHA-256: `9c508561fac30ca9f4a154b21efa3262cbe2cabcfc4c2c9cdb58ec26508ea016`
- Upstream Gaussian records: `236,178`
- Capture method: iPad Pro (3rd generation) + Scaniverse
- License: CC0 1.0 Universal

To keep the repository clone and README capture practical,
`tools/prepare_wakufactory_sakura_3dgs.py` retains records whose zero-based
upstream index is divisible by four. Each selected 248-byte Gaussian record is
copied byte-for-byte; only the PLY vertex count changes. The derivative has
59,045 records, is 14,644,690 bytes, and has SHA-256
`ac0cee7f06f2cebf9d912bf211bc87cd8f3229a0ebd59e0389daadf530389298`.

Reproduce from the network:

```text
python tools/prepare_wakufactory_sakura_3dgs.py
```

Verify the committed derivative without network access:

```text
python tools/prepare_wakufactory_sakura_3dgs.py --check
```
