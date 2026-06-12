//! Triangle mesh loading for render backends.

use std::fs;
use std::path::Path;
use thiserror::Error;

/// CPU-side triangle mesh with per-vertex normals.
#[derive(Clone, Debug, PartialEq)]
pub struct TriangleMesh {
    /// Positions in meters.
    pub positions: Vec<[f32; 3]>,
    /// Unit normals aligned with `positions`.
    pub normals: Vec<[f32; 3]>,
    /// Triangle indices.
    pub indices: Vec<u32>,
}

impl TriangleMesh {
    /// Returns the number of indexed triangles.
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }
}

/// Mesh loading error.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum MeshLoadError {
    /// The file could not be read.
    #[error("failed to read {path}: {message}")]
    Io {
        /// File path.
        path: String,
        /// OS error message.
        message: String,
    },
    /// The file contents are invalid.
    #[error("invalid mesh {path}: {message}")]
    Invalid {
        /// File path.
        path: String,
        /// Parse error message.
        message: String,
    },
}

/// Loads an STL mesh from disk.
pub fn load_stl(path: &Path) -> Result<TriangleMesh, MeshLoadError> {
    let bytes = fs::read(path).map_err(|error| MeshLoadError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    parse_stl_bytes(path, &bytes)
}

fn parse_stl_bytes(path: &Path, bytes: &[u8]) -> Result<TriangleMesh, MeshLoadError> {
    let path_str = path.display().to_string();
    if is_binary_stl(bytes) {
        parse_binary_stl(&path_str, bytes)
    } else {
        parse_ascii_stl(&path_str, bytes)
    }
}

fn is_binary_stl(bytes: &[u8]) -> bool {
    if bytes.len() < 84 {
        return false;
    }
    let triangle_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    bytes.len() == 84 + triangle_count * 50
}

fn parse_binary_stl(_path: &str, bytes: &[u8]) -> Result<TriangleMesh, MeshLoadError> {
    let triangle_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    let mut positions = Vec::with_capacity(triangle_count * 3);
    let mut normals = Vec::with_capacity(triangle_count * 3);
    let mut indices = Vec::with_capacity(triangle_count * 3);

    let mut offset = 84;
    for triangle_index in 0..triangle_count {
        let normal = read_f32_triplet(bytes, offset);
        offset += 12;
        let base = (triangle_index * 3) as u32;
        for vertex_index in 0..3 {
            positions.push(read_f32_triplet(bytes, offset));
            normals.push(normal);
            indices.push(base + vertex_index);
            offset += 12;
        }
        offset += 2;
    }

    Ok(TriangleMesh {
        positions,
        normals,
        indices,
    })
}

fn parse_ascii_stl(path: &str, bytes: &[u8]) -> Result<TriangleMesh, MeshLoadError> {
    let text = std::str::from_utf8(bytes).map_err(|error| MeshLoadError::Invalid {
        path: path.into(),
        message: error.to_string(),
    })?;

    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    let mut current_normal = [0.0, 0.0, 1.0];
    let mut triangle = [[0.0; 3]; 3];
    let mut vertex_in_facet = 0;

    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let mut parts = line.split_whitespace();
        let tag = parts.next().unwrap_or_default();
        match tag {
            "facet" => {
                if parts.next() != Some("normal") {
                    return Err(invalid_mesh(path, "expected facet normal"));
                }
                current_normal = parse_vec3_parts(&mut parts, path, "facet normal")?;
                vertex_in_facet = 0;
            }
            "vertex" => {
                if vertex_in_facet >= 3 {
                    return Err(invalid_mesh(path, "facet has more than three vertices"));
                }
                triangle[vertex_in_facet] = parse_vec3_parts(&mut parts, path, "vertex")?;
                vertex_in_facet += 1;
                if vertex_in_facet == 3 {
                    let base = positions.len() as u32;
                    for vertex in triangle {
                        positions.push(vertex);
                        normals.push(current_normal);
                    }
                    indices.extend_from_slice(&[base, base + 1, base + 2]);
                }
            }
            "outer" | "endloop" | "endfacet" | "solid" | "endsolid" => {}
            other => {
                return Err(invalid_mesh(path, format!("unexpected token '{other}'")));
            }
        }
    }

    if positions.is_empty() {
        return Err(invalid_mesh(path, "no triangles"));
    }

    Ok(TriangleMesh {
        positions,
        normals,
        indices,
    })
}

fn parse_vec3_parts<'a, I>(
    parts: &mut I,
    path: &str,
    field: &str,
) -> Result<[f32; 3], MeshLoadError>
where
    I: Iterator<Item = &'a str>,
{
    let x = parts
        .next()
        .ok_or_else(|| invalid_mesh(path, format!("missing {field}.x")))?
        .parse::<f32>()
        .map_err(|_| invalid_mesh(path, format!("invalid {field}.x")))?;
    let y = parts
        .next()
        .ok_or_else(|| invalid_mesh(path, format!("missing {field}.y")))?
        .parse::<f32>()
        .map_err(|_| invalid_mesh(path, format!("invalid {field}.y")))?;
    let z = parts
        .next()
        .ok_or_else(|| invalid_mesh(path, format!("missing {field}.z")))?
        .parse::<f32>()
        .map_err(|_| invalid_mesh(path, format!("invalid {field}.z")))?;
    Ok([x, y, z])
}

fn read_f32_triplet(bytes: &[u8], offset: usize) -> [f32; 3] {
    [
        f32::from_le_bytes(bytes[offset..offset + 4].try_into().expect("f32")),
        f32::from_le_bytes(bytes[offset + 4..offset + 8].try_into().expect("f32")),
        f32::from_le_bytes(bytes[offset + 8..offset + 12].try_into().expect("f32")),
    ]
}

fn invalid_mesh(path: &str, message: impl Into<String>) -> MeshLoadError {
    MeshLoadError::Invalid {
        path: path.into(),
        message: message.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    const BOX_ASCII_STL: &str = r#"solid box
facet normal 0 0 -1
  outer loop
    vertex -0.25 -0.15 -0.2
    vertex 0.25 -0.15 -0.2
    vertex 0.25 0.15 -0.2
  endloop
endfacet
facet normal 0 0 -1
  outer loop
    vertex -0.25 -0.15 -0.2
    vertex 0.25 0.15 -0.2
    vertex -0.25 0.15 -0.2
  endloop
endfacet
endsolid box
"#;

    #[test]
    fn ascii_stl_loads_triangles() {
        let path = PathBuf::from("/tmp/test_box.stl");
        let mesh = parse_stl_bytes(&path, BOX_ASCII_STL.as_bytes()).expect("parse ascii stl");
        assert_eq!(mesh.triangle_count(), 2);
        assert_eq!(mesh.positions.len(), 6);
        assert_eq!(mesh.indices, vec![0, 1, 2, 3, 4, 5]);
    }

    #[test]
    fn fixture_stl_loads() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/mesh_diff_drive/meshes/base_link.stl");
        let mesh = load_stl(&path).expect("load fixture stl");
        assert!(mesh.triangle_count() >= 12);
    }
}
