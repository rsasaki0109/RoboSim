"""Acquire the pinned F1TENTH hard-r1 bag to external storage with strict bounds."""

import argparse
import hashlib
import json
import os
import urllib.request
from pathlib import Path

RECORD_ID = 12_536_536
DOI = "10.5281/zenodo.12536536"
FILE_NAME = "ex-hard-r1_2023-06-12-19-50-37.bag"
CONTENT_URL = (
    "https://zenodo.org/api/records/12536536/files/"
    "ex-hard-r1_2023-06-12-19-50-37.bag/content"
)
EXPECTED_BYTES = 92_374_784
EXPECTED_MD5 = "6f3feda005530bfe586b8dda3545301a"
CHUNK_BYTES = 1024 * 1024


def acquire_response(response, output, expected_bytes=EXPECTED_BYTES, expected_md5=EXPECTED_MD5):
    """Stream one response into a create-new partial and atomically retain valid bytes."""
    output = Path(output)
    partial = output.with_name(output.name + ".partial")
    if output.exists() or partial.exists():
        raise FileExistsError("F1TENTH destination or partial already exists")
    status = getattr(response, "status", 200)
    if status != 200:
        raise ValueError(f"unexpected F1TENTH HTTP status {status}")
    length = response.headers.get("Content-Length")
    if length is not None and int(length) != expected_bytes:
        raise ValueError("F1TENTH Content-Length differs from pinned metadata")

    md5 = hashlib.md5(usedforsecurity=False)
    sha256 = hashlib.sha256()
    total = 0
    with partial.open("xb") as stream:
        while block := response.read(CHUNK_BYTES):
            total += len(block)
            if total > expected_bytes:
                raise ValueError("F1TENTH response exceeds pinned byte count")
            md5.update(block)
            sha256.update(block)
            stream.write(block)
        stream.flush()
        os.fsync(stream.fileno())
    if total != expected_bytes:
        raise ValueError("F1TENTH response is truncated")
    if md5.hexdigest() != expected_md5:
        raise ValueError("F1TENTH response checksum differs from Zenodo metadata")
    partial.rename(output)
    return {
        "record_id": RECORD_ID,
        "doi": DOI,
        "file_name": FILE_NAME,
        "bytes": total,
        "md5": md5.hexdigest(),
        "sha256": sha256.hexdigest(),
    }


def acquire(output):
    """Open the pinned Zenodo content endpoint and acquire its exact bytes."""
    request = urllib.request.Request(
        CONTENT_URL,
        headers={"User-Agent": "RNE bounded F1TENTH acquisition/1"},
    )
    with urllib.request.urlopen(request, timeout=60) as response:
        return acquire_response(response, output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(json.dumps(acquire(args.output), indent=2, sort_keys=True))
