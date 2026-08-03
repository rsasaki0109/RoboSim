//! Triangle mesh loading for render backends.

use crate::{ImageFrame, PbrMaterial};
use rne_math::{DVec3, Mat4};
use std::collections::HashMap;
use std::fs;
use std::path::Path;
use std::sync::Arc;
use thiserror::Error;

/// CPU-side triangle mesh with per-vertex normals and optional UV coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct TriangleMesh {
    /// Positions in meters.
    pub positions: Vec<[f32; 3]>,
    /// Unit normals aligned with `positions`.
    pub normals: Vec<[f32; 3]>,
    /// UV coordinates aligned with `positions`; absent mappings contain `[0, 0]`.
    pub texcoords: Vec<[f32; 2]>,
    /// Triangle indices.
    pub indices: Vec<u32>,
}

impl TriangleMesh {
    /// Returns the number of indexed triangles.
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }
}

/// One material-homogeneous mesh part loaded from an asset.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadedMeshPart {
    /// Triangle geometry for this material part.
    pub mesh: TriangleMesh,
    /// Optional decoded sRGB base-color texture.
    pub base_color_texture: Option<ImageFrame>,
    /// Optional material diffuse color and opacity.
    pub base_color_rgba: Option<[f32; 4]>,
    /// Metallic-roughness parameters decoded from the asset or filled with defaults.
    pub material: PbrMaterial,
}

/// Loads a supported triangle mesh based on its file extension.
///
/// STL, Wavefront OBJ, and glTF 2.0 files are supported. Extension matching
/// is case-insensitive.
pub fn load_mesh(path: &Path) -> Result<TriangleMesh, MeshLoadError> {
    let parts = load_mesh_parts(path)?;
    merge_mesh_parts(parts)
}

/// Loads material-homogeneous mesh parts and their optional PBR textures.
///
/// STL produces one untextured part. OBJ and glTF object/material boundaries
/// are preserved in source order so callers can draw each material
/// independently.
pub fn load_mesh_parts(path: &Path) -> Result<Vec<LoadedMeshPart>, MeshLoadError> {
    match path
        .extension()
        .and_then(|extension| extension.to_str())
        .map(str::to_ascii_lowercase)
        .as_deref()
    {
        Some("stl") => Ok(vec![LoadedMeshPart {
            mesh: load_stl(path)?,
            base_color_texture: None,
            base_color_rgba: None,
            material: PbrMaterial::default(),
        }]),
        Some("obj") => load_obj_parts(path),
        Some("gltf") | Some("glb") => load_gltf_parts(path),
        _ => Err(invalid_mesh(
            &path.display().to_string(),
            "unsupported mesh extension; expected .stl, .obj, .gltf, or .glb",
        )),
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
    load_stl_bytes(path, &bytes)
}

fn load_obj_parts(path: &Path) -> Result<Vec<LoadedMeshPart>, MeshLoadError> {
    let (models, materials) =
        tobj::load_obj(path, &tobj::GPU_LOAD_OPTIONS).map_err(|error| MeshLoadError::Invalid {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
    let materials = materials.map_err(|error| MeshLoadError::Invalid {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    let mut texture_cache = HashMap::<String, ImageFrame>::new();
    let mut parts = Vec::new();
    for model in models {
        let tobj::Mesh {
            positions: raw_positions,
            normals: raw_normals,
            texcoords: raw_texcoords,
            indices,
            material_id,
            ..
        } = model.mesh;
        let positions: Vec<[f32; 3]> = raw_positions
            .chunks_exact(3)
            .map(|value| [value[0], value[1], value[2]])
            .collect();
        let texcoords = if raw_texcoords.len() == positions.len() * 2 {
            raw_texcoords
                .chunks_exact(2)
                .map(|value| [value[0], value[1]])
                .collect()
        } else {
            vec![[0.0, 0.0]; positions.len()]
        };
        let triangle_mesh = if raw_normals.len() == raw_positions.len() {
            TriangleMesh {
                positions,
                normals: raw_normals
                    .chunks_exact(3)
                    .map(|value| [value[0], value[1], value[2]])
                    .collect(),
                texcoords,
                indices,
            }
        } else {
            mesh_with_flat_normals(&positions, &texcoords, &indices)
        };
        validate_triangle_mesh(path, &triangle_mesh)?;
        let material = material_id.and_then(|index| materials.get(index));
        let base_color_rgba = material.and_then(|material| {
            material.diffuse.map(|diffuse| {
                [
                    diffuse[0],
                    diffuse[1],
                    diffuse[2],
                    material.dissolve.unwrap_or(1.0),
                ]
            })
        });
        let base_color_texture = material
            .and_then(|material| material.diffuse_texture.as_deref())
            .map(|texture_path| load_material_texture(path, texture_path, &mut texture_cache))
            .transpose()?;
        let normal_texture = material
            .and_then(|material| material.normal_texture.as_deref())
            .map(|texture_path| load_material_texture(path, texture_path, &mut texture_cache))
            .transpose()?;
        let roughness_texture = material
            .and_then(|material| material.shininess_texture.as_deref())
            .map(|texture_path| load_material_texture(path, texture_path, &mut texture_cache))
            .transpose()?
            .map(invert_shininess_texture);
        let roughness = material
            .and_then(|material| material.shininess)
            .map(|shininess| (2.0 / (shininess.max(0.0) + 2.0)).sqrt())
            .unwrap_or(PbrMaterial::default().roughness);
        let emissive_rgb = material
            .and_then(|material| material.emissive)
            .unwrap_or([0.0; 3]);
        let material = PbrMaterial::new(
            base_color_rgba.unwrap_or([1.0; 4]),
            roughness,
            0.0,
            emissive_rgb,
        )
        .with_texture_maps(
            normal_texture.map(Arc::new),
            roughness_texture.map(Arc::new),
        );
        parts.push(LoadedMeshPart {
            mesh: triangle_mesh,
            base_color_texture,
            base_color_rgba,
            material,
        });
    }
    if parts.is_empty() {
        return Err(invalid_mesh(
            &path.display().to_string(),
            "OBJ contains no triangles",
        ));
    }
    Ok(parts)
}

fn load_gltf_parts(path: &Path) -> Result<Vec<LoadedMeshPart>, MeshLoadError> {
    let (document, buffers, images) =
        gltf::import(path).map_err(|error| MeshLoadError::Invalid {
            path: path.display().to_string(),
            message: format!("could not import glTF: {error}"),
        })?;
    let scene = document
        .default_scene()
        .or_else(|| document.scenes().next())
        .ok_or_else(|| invalid_mesh(&path.display().to_string(), "glTF contains no scene"))?;
    let mut texture_cache = HashMap::<usize, ImageFrame>::new();
    let mut parts = Vec::new();
    for node in scene.nodes() {
        append_gltf_node(
            path,
            node,
            Mat4::IDENTITY,
            &buffers,
            &images,
            &mut texture_cache,
            &mut parts,
        )?;
    }
    if parts.is_empty() {
        return Err(invalid_mesh(
            &path.display().to_string(),
            "glTF scene contains no triangle primitives",
        ));
    }
    Ok(parts)
}

fn append_gltf_node(
    path: &Path,
    node: gltf::Node<'_>,
    parent_transform: Mat4,
    buffers: &[gltf::buffer::Data],
    images: &[gltf::image::Data],
    texture_cache: &mut HashMap<usize, ImageFrame>,
    parts: &mut Vec<LoadedMeshPart>,
) -> Result<(), MeshLoadError> {
    let transform = parent_transform * gltf_matrix_to_mat4(node.transform().matrix());
    if let Some(mesh) = node.mesh() {
        for primitive in mesh.primitives() {
            if primitive.mode() != gltf::mesh::Mode::Triangles {
                return Err(invalid_mesh(
                    &path.display().to_string(),
                    format!(
                        "unsupported glTF primitive mode {:?}; only triangles are supported",
                        primitive.mode()
                    ),
                ));
            }
            let reader = primitive
                .reader(|buffer| buffers.get(buffer.index()).map(|data| data.0.as_slice()));
            let positions: Vec<[f32; 3]> = reader
                .read_positions()
                .ok_or_else(|| {
                    invalid_mesh(
                        &path.display().to_string(),
                        "glTF primitive has no positions",
                    )
                })?
                .collect();
            let texcoords: Vec<[f32; 2]> = reader
                .read_tex_coords(0)
                .map(|coords| coords.into_f32().collect())
                .unwrap_or_else(|| vec![[0.0, 0.0]; positions.len()]);
            let indices: Vec<u32> = reader
                .read_indices()
                .map(|indices| indices.into_u32().collect())
                .unwrap_or_else(|| (0..positions.len() as u32).collect());
            if texcoords.len() != positions.len() {
                return Err(invalid_mesh(
                    &path.display().to_string(),
                    "glTF TEXCOORD_0 count does not match POSITION count",
                ));
            }
            let mut triangle_mesh = if let Some(normals) = reader.read_normals() {
                let normals: Vec<[f32; 3]> = normals.collect();
                if normals.len() != positions.len() {
                    return Err(invalid_mesh(
                        &path.display().to_string(),
                        "glTF NORMAL count does not match POSITION count",
                    ));
                }
                TriangleMesh {
                    positions,
                    normals,
                    texcoords,
                    indices,
                }
            } else {
                mesh_with_flat_normals(&positions, &texcoords, &indices)
            };
            transform_triangle_mesh(&mut triangle_mesh, transform);
            validate_triangle_mesh(path, &triangle_mesh)?;

            let material = primitive.material();
            let pbr = material.pbr_metallic_roughness();
            let base_color_texture = pbr
                .base_color_texture()
                .map(|info| load_gltf_texture(path, info.texture(), images, texture_cache))
                .transpose()?;
            let metallic_roughness_texture = pbr
                .metallic_roughness_texture()
                .map(|info| load_gltf_texture(path, info.texture(), images, texture_cache))
                .transpose()?;
            let normal_info = material.normal_texture();
            let normal_strength = normal_info.as_ref().map(|info| info.scale()).unwrap_or(1.0);
            let normal_texture = normal_info
                .map(|info| load_gltf_texture(path, info.texture(), images, texture_cache))
                .transpose()?;
            let occlusion_info = material.occlusion_texture();
            let occlusion_strength = occlusion_info
                .as_ref()
                .map(|info| info.strength())
                .unwrap_or(1.0);
            let occlusion_texture = occlusion_info
                .map(|info| load_gltf_texture(path, info.texture(), images, texture_cache))
                .transpose()?;
            let emissive_texture = material
                .emissive_texture()
                .map(|info| load_gltf_texture(path, info.texture(), images, texture_cache))
                .transpose()?;
            let base_color_rgba = pbr.base_color_factor();
            let material = PbrMaterial::new(
                base_color_rgba,
                pbr.roughness_factor(),
                pbr.metallic_factor(),
                material.emissive_factor(),
            )
            .with_texture_maps(normal_texture.map(Arc::new), None)
            .with_normal_strength(normal_strength)
            .with_pbr_texture_maps(
                metallic_roughness_texture.map(Arc::new),
                emissive_texture.map(Arc::new),
                occlusion_texture.map(Arc::new),
                occlusion_strength,
            );
            parts.push(LoadedMeshPart {
                mesh: triangle_mesh,
                base_color_texture,
                base_color_rgba: Some(base_color_rgba),
                material,
            });
        }
    }
    for child in node.children() {
        append_gltf_node(
            path,
            child,
            transform,
            buffers,
            images,
            texture_cache,
            parts,
        )?;
    }
    Ok(())
}

fn gltf_matrix_to_mat4(matrix: [[f32; 4]; 4]) -> Mat4 {
    Mat4::from_cols_array(&[
        matrix[0][0] as f64,
        matrix[0][1] as f64,
        matrix[0][2] as f64,
        matrix[0][3] as f64,
        matrix[1][0] as f64,
        matrix[1][1] as f64,
        matrix[1][2] as f64,
        matrix[1][3] as f64,
        matrix[2][0] as f64,
        matrix[2][1] as f64,
        matrix[2][2] as f64,
        matrix[2][3] as f64,
        matrix[3][0] as f64,
        matrix[3][1] as f64,
        matrix[3][2] as f64,
        matrix[3][3] as f64,
    ])
}

fn transform_triangle_mesh(mesh: &mut TriangleMesh, transform: Mat4) {
    for position in &mut mesh.positions {
        let transformed = transform.transform_point3(DVec3::new(
            f64::from(position[0]),
            f64::from(position[1]),
            f64::from(position[2]),
        ));
        *position = [
            transformed.x as f32,
            transformed.y as f32,
            transformed.z as f32,
        ];
    }
    let normal_transform = transform.inverse().transpose();
    for normal in &mut mesh.normals {
        let transformed = normal_transform
            .transform_vector3(DVec3::new(
                f64::from(normal[0]),
                f64::from(normal[1]),
                f64::from(normal[2]),
            ))
            .normalize_or_zero();
        *normal = [
            transformed.x as f32,
            transformed.y as f32,
            transformed.z as f32,
        ];
    }
}

fn load_gltf_texture(
    path: &Path,
    texture: gltf::Texture<'_>,
    images: &[gltf::image::Data],
    cache: &mut HashMap<usize, ImageFrame>,
) -> Result<ImageFrame, MeshLoadError> {
    let image_index = texture.source().index();
    if let Some(image) = cache.get(&image_index) {
        return Ok(image.clone());
    }
    let image = images.get(image_index).ok_or_else(|| {
        invalid_mesh(
            &path.display().to_string(),
            format!("glTF texture references missing image {image_index}"),
        )
    })?;
    let decoded = gltf_image_to_frame(path, image_index, image)?;
    cache.insert(image_index, decoded.clone());
    Ok(decoded)
}

fn gltf_image_to_frame(
    path: &Path,
    image_index: usize,
    image: &gltf::image::Data,
) -> Result<ImageFrame, MeshLoadError> {
    let (channels, bytes_per_channel, is_float) = match image.format {
        gltf::image::Format::R8 => (1, 1, false),
        gltf::image::Format::R8G8 => (2, 1, false),
        gltf::image::Format::R8G8B8 => (3, 1, false),
        gltf::image::Format::R8G8B8A8 => (4, 1, false),
        gltf::image::Format::R16 => (1, 2, false),
        gltf::image::Format::R16G16 => (2, 2, false),
        gltf::image::Format::R16G16B16 => (3, 2, false),
        gltf::image::Format::R16G16B16A16 => (4, 2, false),
        gltf::image::Format::R32G32B32FLOAT => (3, 4, true),
        gltf::image::Format::R32G32B32A32FLOAT => (4, 4, true),
    };
    let pixel_stride = channels * bytes_per_channel;
    let expected_len = image.width as usize * image.height as usize * pixel_stride;
    if image.pixels.len() != expected_len {
        return Err(invalid_mesh(
            &path.display().to_string(),
            format!(
                "glTF image {image_index} has {} bytes; expected {expected_len}",
                image.pixels.len()
            ),
        ));
    }
    let mut rgba8 = Vec::with_capacity(image.width as usize * image.height as usize * 4);
    for pixel in image.pixels.chunks_exact(pixel_stride) {
        let mut values = [0.0_f32; 4];
        for (channel, value) in values.iter_mut().enumerate().take(channels) {
            let offset = channel * bytes_per_channel;
            *value = if is_float {
                f32::from_le_bytes(pixel[offset..offset + 4].try_into().expect("f32 channel"))
            } else if bytes_per_channel == 2 {
                f32::from(u16::from_le_bytes(
                    pixel[offset..offset + 2].try_into().expect("u16 channel"),
                )) / 65535.0
            } else {
                f32::from(pixel[offset]) / 255.0
            };
        }
        let alpha = match channels {
            2 | 4 => values[channels - 1],
            _ => 1.0,
        };
        let red = values[0];
        let green = if channels >= 3 { values[1] } else { red };
        let blue = if channels >= 3 { values[2] } else { red };
        rgba8.extend([
            normalized_channel_to_u8(red),
            normalized_channel_to_u8(green),
            normalized_channel_to_u8(blue),
            normalized_channel_to_u8(alpha),
        ]);
    }
    Ok(ImageFrame::from_rgba8(image.width, image.height, rgba8))
}

fn normalized_channel_to_u8(value: f32) -> u8 {
    if value.is_finite() {
        (value.clamp(0.0, 1.0) * 255.0).round() as u8
    } else {
        0
    }
}

fn merge_mesh_parts(parts: Vec<LoadedMeshPart>) -> Result<TriangleMesh, MeshLoadError> {
    if parts.is_empty() {
        return Err(invalid_mesh("<mesh parts>", "mesh contains no parts"));
    }
    let mut merged = TriangleMesh {
        positions: Vec::new(),
        normals: Vec::new(),
        texcoords: Vec::new(),
        indices: Vec::new(),
    };
    for part in parts {
        let vertex_offset = merged.positions.len() as u32;
        merged.positions.extend(part.mesh.positions);
        merged.normals.extend(part.mesh.normals);
        merged.texcoords.extend(part.mesh.texcoords);
        merged.indices.extend(
            part.mesh
                .indices
                .into_iter()
                .map(|index| vertex_offset + index),
        );
    }
    Ok(merged)
}

fn validate_triangle_mesh(path: &Path, mesh: &TriangleMesh) -> Result<(), MeshLoadError> {
    if mesh.positions.is_empty() || mesh.indices.is_empty() || !mesh.indices.len().is_multiple_of(3)
    {
        return Err(invalid_mesh(
            &path.display().to_string(),
            "mesh contains no triangles",
        ));
    }
    if mesh.normals.len() != mesh.positions.len() || mesh.texcoords.len() != mesh.positions.len() {
        return Err(invalid_mesh(
            &path.display().to_string(),
            "mesh vertex attributes have different lengths",
        ));
    }
    if mesh
        .indices
        .iter()
        .any(|index| *index as usize >= mesh.positions.len())
    {
        return Err(invalid_mesh(
            &path.display().to_string(),
            "mesh index is out of bounds",
        ));
    }
    Ok(())
}

fn load_material_texture(
    obj_path: &Path,
    texture_path: &str,
    cache: &mut HashMap<String, ImageFrame>,
) -> Result<ImageFrame, MeshLoadError> {
    if let Some(texture) = cache.get(texture_path) {
        return Ok(texture.clone());
    }
    let path = obj_path
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join(texture_path);
    let decoded = image::open(&path).map_err(|error| MeshLoadError::Invalid {
        path: path.display().to_string(),
        message: format!("could not decode material texture: {error}"),
    })?;
    let rgba8 = decoded.to_rgba8();
    let texture = ImageFrame::from_rgba8(rgba8.width(), rgba8.height(), rgba8.into_raw());
    cache.insert(texture_path.to_owned(), texture.clone());
    Ok(texture)
}

fn invert_shininess_texture(mut texture: ImageFrame) -> ImageFrame {
    for pixel in texture.rgba8.chunks_exact_mut(4) {
        let roughness = 255_u8.saturating_sub(pixel[0]);
        pixel[0] = roughness;
        pixel[1] = roughness;
        pixel[2] = roughness;
    }
    texture
}

fn mesh_with_flat_normals(
    positions: &[[f32; 3]],
    texcoords: &[[f32; 2]],
    indices: &[u32],
) -> TriangleMesh {
    let mut flat_positions = Vec::with_capacity(indices.len());
    let mut flat_normals = Vec::with_capacity(indices.len());
    let mut flat_texcoords = Vec::with_capacity(indices.len());
    let mut flat_indices = Vec::with_capacity(indices.len());
    for triangle in indices.chunks_exact(3) {
        let a = positions[triangle[0] as usize];
        let b = positions[triangle[1] as usize];
        let c = positions[triangle[2] as usize];
        let mut normal = cross(subtract(b, a), subtract(c, a));
        let length = length_squared(normal).sqrt();
        if length > f32::EPSILON {
            normal = [normal[0] / length, normal[1] / length, normal[2] / length];
        } else {
            normal = [0.0, 1.0, 0.0];
        }
        let base = flat_positions.len() as u32;
        for position in [a, b, c] {
            flat_positions.push(position);
            flat_normals.push(normal);
        }
        flat_texcoords.extend([
            texcoords[triangle[0] as usize],
            texcoords[triangle[1] as usize],
            texcoords[triangle[2] as usize],
        ]);
        flat_indices.extend_from_slice(&[base, base + 1, base + 2]);
    }
    TriangleMesh {
        positions: flat_positions,
        normals: flat_normals,
        texcoords: flat_texcoords,
        indices: flat_indices,
    }
}

fn subtract(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn cross(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn length_squared(value: [f32; 3]) -> f32 {
    value[0] * value[0] + value[1] * value[1] + value[2] * value[2]
}

/// Parses an STL mesh from in-memory bytes.
///
/// `path` is used only for error messages (for example a virtual `package://` URI).
pub fn load_stl_bytes(path: impl AsRef<Path>, bytes: &[u8]) -> Result<TriangleMesh, MeshLoadError> {
    parse_stl_bytes(path.as_ref(), bytes)
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
        texcoords: vec![[0.0, 0.0]; positions.len()],
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
        texcoords: vec![[0.0, 0.0]; positions.len()],
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

    #[test]
    fn obj_models_merge_and_generate_normals_deterministically() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/two_panels.obj");
        let first = load_mesh(&path).expect("load OBJ fixture");
        let second = load_mesh(&path).expect("replay OBJ fixture");
        assert_eq!(first, second);
        assert_eq!(first.triangle_count(), 2);
        assert_eq!(first.positions.len(), 6);
        assert_eq!(first.normals.len(), first.positions.len());
        assert!(first
            .normals
            .iter()
            .all(|normal| (length_squared(*normal) - 1.0).abs() < 1.0e-5));
    }

    #[test]
    fn obj_material_parts_preserve_uvs_and_decode_diffuse_texture() {
        let root =
            std::env::temp_dir().join(format!("rne-render-textured-obj-{}", std::process::id()));
        if root.exists() {
            fs::remove_dir_all(&root).expect("remove stale textured OBJ test");
        }
        fs::create_dir_all(&root).expect("create textured OBJ test");
        fs::write(
            root.join("panel.obj"),
            "mtllib panel.mtl\no panel\nv 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nusemtl facade\nf 1/1 2/2 3/3\n",
        )
        .expect("write OBJ");
        fs::write(
            root.join("panel.mtl"),
            "newmtl facade\nKd 0.25 0.5 0.75\nNs 32\nmap_Kd facade.png\nmap_Bump facade_normal.png\nmap_Ns facade_roughness.png\n",
        )
        .expect("write MTL");
        image::RgbaImage::from_pixel(2, 1, image::Rgba([24, 80, 160, 255]))
            .save(root.join("facade.png"))
            .expect("write texture");
        image::RgbaImage::from_pixel(2, 1, image::Rgba([128, 128, 255, 255]))
            .save(root.join("facade_normal.png"))
            .expect("write normal texture");
        image::RgbaImage::from_pixel(2, 1, image::Rgba([180, 180, 180, 255]))
            .save(root.join("facade_roughness.png"))
            .expect("write roughness texture");

        let parts = load_mesh_parts(&root.join("panel.obj")).expect("load textured OBJ");
        assert_eq!(parts.len(), 1);
        assert_eq!(
            parts[0].mesh.texcoords,
            vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
        );
        let texture = parts[0]
            .base_color_texture
            .as_ref()
            .expect("diffuse texture");
        assert_eq!((texture.width, texture.height), (2, 1));
        assert_eq!(&texture.rgba8[..4], &[24, 80, 160, 255]);
        assert_eq!(parts[0].material.base_color_rgba, [0.25, 0.5, 0.75, 1.0]);
        assert_eq!(parts[0].material.emissive_rgb, [0.0, 0.0, 0.0]);
        assert!(parts[0].material.normal_texture.is_some());
        let roughness_texture = parts[0]
            .material
            .roughness_texture
            .as_ref()
            .expect("shininess texture converted to roughness");
        assert_eq!(&roughness_texture.rgba8[..4], &[75, 75, 75, 255]);
        assert!((parts[0].material.roughness - (2.0_f32 / 34.0).sqrt()).abs() < 1.0e-6);

        fs::remove_dir_all(root).expect("remove textured OBJ test");
    }

    #[test]
    fn gltf_imports_scene_transforms_and_pbr_material_maps() {
        let root = std::env::temp_dir().join(format!("rne-render-pbr-gltf-{}", std::process::id()));
        if root.exists() {
            fs::remove_dir_all(&root).expect("remove stale glTF test");
        }
        fs::create_dir_all(&root).expect("create glTF test");
        let geometry = test_triangle_geometry();
        fs::write(root.join("mesh.bin"), &geometry).expect("write glTF buffer");
        write_test_texture(&root, "base.png", [24, 80, 160, 255]);
        write_test_texture(&root, "metallic_roughness.png", [0, 128, 200, 255]);
        write_test_texture(&root, "normal.png", [128, 128, 255, 255]);
        write_test_texture(&root, "emissive.png", [12, 24, 48, 255]);
        write_test_texture(&root, "occlusion.png", [160, 160, 160, 255]);
        fs::write(
            root.join("panel.gltf"),
            test_gltf_json(Some("mesh.bin"), geometry.len(), true),
        )
        .expect("write glTF document");

        let parts = load_mesh_parts(&root.join("panel.gltf")).expect("load glTF");
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].mesh.triangle_count(), 1);
        assert_eq!(parts[0].mesh.positions[0], [1.0, 2.0, 3.0]);
        assert_eq!(parts[0].mesh.positions[1], [2.0, 2.0, 3.0]);
        assert_eq!(
            parts[0].mesh.texcoords,
            vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
        );
        assert_eq!(
            parts[0]
                .base_color_texture
                .as_ref()
                .expect("base-color texture")
                .rgba8,
            vec![24, 80, 160, 255]
        );
        assert_eq!(parts[0].material.base_color_rgba, [0.25, 0.5, 0.75, 0.8]);
        assert_eq!(parts[0].material.roughness, 0.35);
        assert_eq!(parts[0].material.metallic, 0.6);
        assert_eq!(parts[0].material.emissive_rgb, [0.1, 0.2, 0.3]);
        assert_eq!(parts[0].material.normal_strength, 0.65);
        assert_eq!(
            parts[0]
                .material
                .normal_texture
                .as_ref()
                .expect("normal texture")
                .rgba8,
            vec![128, 128, 255, 255]
        );
        assert_eq!(
            parts[0]
                .material
                .metallic_roughness_texture
                .as_ref()
                .expect("metallic-roughness texture")
                .rgba8,
            vec![0, 128, 200, 255]
        );
        assert_eq!(
            parts[0]
                .material
                .emissive_texture
                .as_ref()
                .expect("emissive texture")
                .rgba8,
            vec![12, 24, 48, 255]
        );
        assert_eq!(
            parts[0]
                .material
                .occlusion_texture
                .as_ref()
                .expect("occlusion texture")
                .rgba8,
            vec![160, 160, 160, 255]
        );
        assert_eq!(parts[0].material.occlusion_strength, 0.45);

        fs::remove_dir_all(root).expect("remove glTF test");
    }

    #[test]
    fn glb_imports_embedded_triangle_buffer() {
        let root =
            std::env::temp_dir().join(format!("rne-render-embedded-glb-{}", std::process::id()));
        if root.exists() {
            fs::remove_dir_all(&root).expect("remove stale GLB test");
        }
        fs::create_dir_all(&root).expect("create GLB test");
        let geometry = test_triangle_geometry();
        let json = test_gltf_json(None, geometry.len(), false);
        fs::write(root.join("panel.glb"), make_test_glb(&json, &geometry)).expect("write GLB");

        let parts = load_mesh_parts(&root.join("panel.glb")).expect("load GLB");
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].mesh.triangle_count(), 1);
        assert_eq!(parts[0].material.base_color_rgba, [1.0; 4]);
        assert_eq!(parts[0].material.roughness, 1.0);
        assert_eq!(parts[0].material.metallic, 1.0);
        assert!(parts[0].material.normal_texture.is_none());
        assert!(parts[0].material.metallic_roughness_texture.is_none());

        fs::remove_dir_all(root).expect("remove GLB test");
    }

    fn write_test_texture(root: &Path, name: &str, rgba: [u8; 4]) {
        image::RgbaImage::from_pixel(1, 1, image::Rgba(rgba))
            .save(root.join(name))
            .expect("write glTF test texture");
    }

    fn test_triangle_geometry() -> Vec<u8> {
        let mut bytes = Vec::new();
        for position in [[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
            for component in position {
                bytes.extend(component.to_le_bytes());
            }
        }
        for _ in 0..3 {
            bytes.extend(0.0_f32.to_le_bytes());
            bytes.extend(0.0_f32.to_le_bytes());
            bytes.extend(1.0_f32.to_le_bytes());
        }
        for uv in [[0.0_f32, 0.0], [1.0, 0.0], [0.0, 1.0]] {
            for component in uv {
                bytes.extend(component.to_le_bytes());
            }
        }
        for index in [0_u16, 1, 2] {
            bytes.extend(index.to_le_bytes());
        }
        bytes
    }

    fn test_gltf_json(buffer_uri: Option<&str>, buffer_len: usize, textured: bool) -> String {
        let buffer = buffer_uri
            .map(|uri| format!(r#"{{"uri":"{uri}","byteLength":{buffer_len}}}"#))
            .unwrap_or_else(|| format!(r#"{{"byteLength":{buffer_len}}}"#));
        let material = if textured {
            r#","materials":[{"pbrMetallicRoughness":{"baseColorFactor":[0.25,0.5,0.75,0.8],"baseColorTexture":{"index":0},"metallicFactor":0.6,"roughnessFactor":0.35,"metallicRoughnessTexture":{"index":1}},"normalTexture":{"index":2,"scale":0.65},"occlusionTexture":{"index":4,"strength":0.45},"emissiveFactor":[0.1,0.2,0.3],"emissiveTexture":{"index":3}}],"images":[{"uri":"base.png"},{"uri":"metallic_roughness.png"},{"uri":"normal.png"},{"uri":"emissive.png"},{"uri":"occlusion.png"}],"textures":[{"source":0},{"source":1},{"source":2},{"source":3},{"source":4}]"#
        } else {
            ""
        };
        let primitive_material = if textured { ",\"material\":0" } else { "" };
        format!(
            r#"{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0]}}],"nodes":[{{"mesh":0,"matrix":[1,0,0,0,0,1,0,0,0,0,1,0,1,2,3,1]}}],"meshes":[{{"primitives":[{{"attributes":{{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2}},"indices":3{primitive_material}}}]}}],"buffers":[{buffer}],"bufferViews":[{{"buffer":0,"byteOffset":0,"byteLength":36}},{{"buffer":0,"byteOffset":36,"byteLength":36}},{{"buffer":0,"byteOffset":72,"byteLength":24}},{{"buffer":0,"byteOffset":96,"byteLength":6}}],"accessors":[{{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]}},{{"bufferView":1,"componentType":5126,"count":3,"type":"VEC3"}},{{"bufferView":2,"componentType":5126,"count":3,"type":"VEC2"}},{{"bufferView":3,"componentType":5123,"count":3,"type":"SCALAR"}}]{material}}}"#
        )
    }

    fn make_test_glb(json: &str, binary: &[u8]) -> Vec<u8> {
        let mut json_bytes = json.as_bytes().to_vec();
        while !json_bytes.len().is_multiple_of(4) {
            json_bytes.push(b' ');
        }
        let mut binary = binary.to_vec();
        while !binary.len().is_multiple_of(4) {
            binary.push(0);
        }
        let total_length = 12 + 8 + json_bytes.len() + 8 + binary.len();
        let mut glb = Vec::with_capacity(total_length);
        glb.extend(0x46546c67_u32.to_le_bytes());
        glb.extend(2_u32.to_le_bytes());
        glb.extend((total_length as u32).to_le_bytes());
        glb.extend((json_bytes.len() as u32).to_le_bytes());
        glb.extend(0x4e4f534a_u32.to_le_bytes());
        glb.extend(json_bytes);
        glb.extend((binary.len() as u32).to_le_bytes());
        glb.extend(0x004e4942_u32.to_le_bytes());
        glb.extend(binary);
        glb
    }

    #[test]
    fn generic_mesh_loader_rejects_unknown_extensions() {
        let error = load_mesh(Path::new("factory.ply")).expect_err("unsupported mesh");
        assert!(error
            .to_string()
            .contains("expected .stl, .obj, .gltf, or .glb"));
    }
}
