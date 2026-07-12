"""Convert vendored Unitree COLLADA visuals to RNE-supported binary STL.

Development dependency: ``python -m pip install trimesh pycollada``.
The upstream DAE and URDF files remain unchanged; this writes derived ``stl``
meshes and an ``*.rne.urdf`` whose package URIs point at those meshes.
"""

from pathlib import Path

import trimesh


def convert(package: Path, urdf_name: str) -> None:
    output = package / "stl"
    output.mkdir(exist_ok=True)
    for source in sorted((package / "dae").glob("*.dae")):
        loaded = trimesh.load(source, force="scene")
        mesh = loaded.to_geometry() if isinstance(loaded, trimesh.Scene) else loaded
        mesh.export(output / f"{source.stem}.stl")

    source_urdf = package / urdf_name
    text = source_urdf.read_text(encoding="utf-8")
    text = text.replace(
        f"package://{package.name}/dae/", f"package://{package.name}/stl/"
    ).replace(".dae", ".stl")
    source_urdf.with_suffix(".rne.urdf").write_text(text, encoding="utf-8")


if __name__ == "__main__":
    repo = Path(__file__).resolve().parents[1]
    convert(repo / "assets/robots/go2_description", "go2_description.urdf")
