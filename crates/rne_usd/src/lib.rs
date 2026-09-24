//! Minimal ASCII Universal Scene Description (`.usda`) import.
//!
//! The importer parses a strict, self-contained subset of a single ASCII USD
//! layer and returns world-space triangle meshes. It deliberately mirrors the
//! engine's other minimal importers: unsupported constructs are rejected with a
//! clear error rather than silently dropped, and there is no layer composition,
//! references, payloads, variants, or binary `.usdc`/`.usdz` support.
//!
//! Supported subset:
//!
//! - `#usda 1.0` magic on the first non-empty line;
//! - `def`/`over`/`class` `Xform` and `Mesh` prims nested by braces, with an
//!   optional parenthesized metadata block after the prim name;
//! - `double3`/`float3` `xformOp:translate` and `matrix4d` `xformOp:transform`
//!   (rotation + translation; scale/shear is ignored);
//! - `point3f[] points`, `int[] faceVertexCounts`, `int[] faceVertexIndices`,
//!   and `float3[] primvars:displayColor`.
//!
//! Unknown attributes are skipped by consuming a balanced value, so a file with
//! extra metadata still imports.

#![deny(missing_docs)]

use rne_math::{Quat, Transform3, Vec3};
use std::path::{Path, PathBuf};

/// Maximum accepted `.usda` input size in bytes.
pub const USD_MAX_INPUT_BYTES: usize = 64 * 1024 * 1024;

/// Import failure.
#[derive(Debug, thiserror::Error)]
pub enum UsdError {
    /// Reading the input failed.
    #[error("USD import I/O failed: {0}")]
    Io(#[from] std::io::Error),
    /// The input exceeded [`USD_MAX_INPUT_BYTES`].
    #[error("USD input exceeds the {0} byte limit")]
    TooLarge(usize),
    /// The magic header was missing or malformed.
    #[error("USD input is not an ASCII `#usda` layer")]
    NotUsda,
    /// The document could not be parsed.
    #[error("USD parse error: {0}")]
    Parse(String),
    /// A supported file used an unsupported construct.
    #[error("unsupported USD construct: {0}")]
    Unsupported(String),
    /// A number or point was not finite.
    #[error("USD input contained a non-finite value")]
    NonFinite,
    /// The layer contained no meshes.
    #[error("USD layer contains no meshes")]
    EmptyScene,
}

/// A world-space triangle mesh extracted from a USD layer.
#[derive(Clone, Debug, PartialEq)]
pub struct UsdMesh {
    /// Prim path (for example `/World/Robot/base_link`).
    pub name: String,
    /// World-space triangle vertices.
    pub points: Vec<Vec3>,
    /// Triangle indices (flat triples) into [`Self::points`].
    pub indices: Vec<u32>,
    /// Optional display color (linear RGBA).
    pub color_rgba: Option<[f32; 4]>,
}

impl UsdMesh {
    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// Renders the mesh as a Wavefront OBJ string (1-indexed faces).
    pub fn to_obj(&self) -> String {
        let mut out = String::new();
        out.push_str("# RNE USD import\n");
        for point in &self.points {
            out.push_str(&format!("v {} {} {}\n", point.x, point.y, point.z));
        }
        for triangle in self.indices.chunks_exact(3) {
            out.push_str(&format!(
                "f {} {} {}\n",
                triangle[0] + 1,
                triangle[1] + 1,
                triangle[2] + 1
            ));
        }
        out
    }
}

/// Meshes extracted from a USD layer.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct UsdScene {
    /// World-space meshes in document order.
    pub meshes: Vec<UsdMesh>,
}

impl UsdScene {
    /// Writes one OBJ file per mesh into `directory`, returning the paths.
    pub fn write_obj_files(&self, directory: &Path) -> Result<Vec<PathBuf>, UsdError> {
        std::fs::create_dir_all(directory)?;
        let mut paths = Vec::with_capacity(self.meshes.len());
        for (index, mesh) in self.meshes.iter().enumerate() {
            let base = sanitize(&mesh.name);
            let file = directory.join(format!("{index:03}_{base}.obj"));
            std::fs::write(&file, mesh.to_obj())?;
            paths.push(file);
        }
        Ok(paths)
    }
}

/// Parses an ASCII `.usda` byte slice into world-space meshes.
pub fn parse_usda(bytes: &[u8]) -> Result<UsdScene, UsdError> {
    if bytes.len() > USD_MAX_INPUT_BYTES {
        return Err(UsdError::TooLarge(bytes.len()));
    }
    let source = std::str::from_utf8(bytes)
        .map_err(|_| UsdError::Parse("input is not valid UTF-8".into()))?;
    let tokens = tokenize(source)?;
    let mut parser = Parser {
        tokens,
        position: 0,
    };
    let scene = parser.parse_document()?;
    if scene.meshes.is_empty() {
        return Err(UsdError::EmptyScene);
    }
    Ok(scene)
}

/// Reads and parses an ASCII `.usda` file.
pub fn parse_usda_file(path: &Path) -> Result<UsdScene, UsdError> {
    let bytes = std::fs::read(path)?;
    parse_usda(&bytes)
}

fn sanitize(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    for character in name.chars() {
        if character.is_ascii_alphanumeric() || character == '-' || character == '_' {
            out.push(character);
        } else {
            out.push('_');
        }
    }
    let trimmed = out.trim_matches('_').to_string();
    if trimmed.is_empty() {
        "mesh".to_string()
    } else {
        trimmed
    }
}

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Word(String),
    String(String),
    OpenBrace,
    CloseBrace,
    OpenParen,
    CloseParen,
    OpenBracket,
    CloseBracket,
    Comma,
    Equals,
}

fn tokenize(source: &str) -> Result<Vec<Token>, UsdError> {
    let mut tokens = Vec::new();
    let mut saw_magic = false;
    for line in source.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        if !saw_magic {
            if trimmed.starts_with("#usda") {
                saw_magic = true;
                continue;
            }
            if trimmed.starts_with('#') {
                continue;
            }
        }
        // Strip a trailing inline comment.
        let content = match trimmed.find('#') {
            Some(index) => &trimmed[..index],
            None => trimmed,
        };
        tokenize_line(content, &mut tokens)?;
    }
    if !saw_magic {
        return Err(UsdError::NotUsda);
    }
    Ok(tokens)
}

fn tokenize_line(content: &str, tokens: &mut Vec<Token>) -> Result<(), UsdError> {
    let mut chars = content.chars().peekable();
    while let Some(character) = chars.next() {
        match character {
            ' ' | '\t' | '\r' => {}
            '{' => tokens.push(Token::OpenBrace),
            '}' => tokens.push(Token::CloseBrace),
            '(' => tokens.push(Token::OpenParen),
            ')' => tokens.push(Token::CloseParen),
            '[' => tokens.push(Token::OpenBracket),
            ']' => tokens.push(Token::CloseBracket),
            ',' => tokens.push(Token::Comma),
            '=' => tokens.push(Token::Equals),
            '"' => {
                let mut value = String::new();
                let mut closed = false;
                for inner in chars.by_ref() {
                    if inner == '"' {
                        closed = true;
                        break;
                    }
                    value.push(inner);
                }
                if !closed {
                    return Err(UsdError::Parse("unterminated string literal".into()));
                }
                tokens.push(Token::String(value));
            }
            other => {
                let mut word = String::new();
                word.push(other);
                while let Some(next) = chars.peek() {
                    if next.is_whitespace()
                        || matches!(
                            next,
                            '{' | '}' | '(' | ')' | '[' | ']' | ',' | '=' | '"' | '#'
                        )
                    {
                        break;
                    }
                    word.push(chars.next().expect("peeked"));
                }
                tokens.push(Token::Word(word));
            }
        }
    }
    Ok(())
}

struct Parser {
    tokens: Vec<Token>,
    position: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.position)
    }

    fn next(&mut self) -> Option<Token> {
        let token = self.tokens.get(self.position).cloned();
        if token.is_some() {
            self.position += 1;
        }
        token
    }

    fn expect(&mut self, expected: &Token) -> Result<(), UsdError> {
        match self.next() {
            Some(ref token) if token == expected => Ok(()),
            other => Err(UsdError::Parse(format!(
                "expected {expected:?}, found {other:?}"
            ))),
        }
    }

    fn parse_document(&mut self) -> Result<UsdScene, UsdError> {
        let mut scene = UsdScene::default();
        while self.position < self.tokens.len() {
            if self.is_prim_start() {
                self.parse_prim(Transform3::IDENTITY, "", &mut scene)?;
            } else {
                self.position += 1;
            }
        }
        Ok(scene)
    }

    fn is_prim_start(&self) -> bool {
        matches!(self.peek(), Some(Token::Word(word)) if word == "def" || word == "over" || word == "class")
    }

    fn parse_prim(
        &mut self,
        parent: Transform3,
        parent_path: &str,
        scene: &mut UsdScene,
    ) -> Result<(), UsdError> {
        let keyword = match self.next() {
            Some(Token::Word(word)) => word,
            other => {
                return Err(UsdError::Parse(format!(
                    "expected prim keyword, found {other:?}"
                )))
            }
        };
        let _ = keyword;
        let prim_type = self.expect_word()?;
        let name = match self.next() {
            Some(Token::String(value)) => value,
            other => {
                return Err(UsdError::Parse(format!(
                    "expected prim name, found {other:?}"
                )))
            }
        };
        // Optional metadata block `( ... )` before the body.
        if matches!(self.peek(), Some(Token::OpenParen)) {
            self.skip_balanced(Token::OpenParen, Token::CloseParen)?;
        }
        self.expect(&Token::OpenBrace)?;

        let path = if parent_path.is_empty() {
            format!("/{name}")
        } else {
            format!("{parent_path}/{name}")
        };

        let mut local_translation = Vec3::ZERO;
        let mut local_matrix: Option<Transform3> = None;
        let mut points: Option<Vec<Vec3>> = None;
        let mut counts: Option<Vec<u32>> = None;
        let mut indices: Option<Vec<u32>> = None;
        let mut color: Option<[f32; 4]> = None;
        let mut color_scalar: Option<f64> = None;

        loop {
            match self.peek() {
                None => return Err(UsdError::Parse("unterminated prim body".into())),
                Some(Token::CloseBrace) => {
                    self.position += 1;
                    break;
                }
                Some(Token::Word(_)) if self.is_prim_start() => {
                    let composed =
                        parent.mul_transform(&local_transform(local_translation, local_matrix));
                    self.parse_prim(composed, &path, scene)?;
                }
                Some(Token::Word(_)) => {
                    self.parse_attribute(
                        &mut local_translation,
                        &mut local_matrix,
                        &mut points,
                        &mut counts,
                        &mut indices,
                        &mut color,
                        &mut color_scalar,
                    )?;
                }
                Some(_) => {
                    self.position += 1;
                }
            }
        }

        if prim_type == "Mesh" {
            let points = points
                .ok_or_else(|| UsdError::Parse(format!("mesh `{path}` is missing `points`")))?;
            let transform = parent.mul_transform(&local_transform(local_translation, local_matrix));
            let triangles = triangulate(counts.as_deref(), indices.as_deref(), points.len())
                .ok_or_else(|| UsdError::Parse(format!("mesh `{path}` has invalid faces")))?;
            let world_points = points
                .iter()
                .map(|point| transform.transform_point(*point))
                .collect::<Vec<_>>();
            if world_points.iter().any(|point| !point.is_finite()) {
                return Err(UsdError::NonFinite);
            }
            let color_rgba = color.or_else(|| {
                color_scalar.map(|value| [value as f32, value as f32, value as f32, 1.0])
            });
            scene.meshes.push(UsdMesh {
                name: path,
                points: world_points,
                indices: triangles,
                color_rgba,
            });
        }
        Ok(())
    }

    // Each parameter is an independent named SI-unit quantity; bundling into a config struct here would only relocate the arity, not reduce it.
    #[allow(clippy::too_many_arguments)]
    fn parse_attribute(
        &mut self,
        local_translation: &mut Vec3,
        local_matrix: &mut Option<Transform3>,
        points: &mut Option<Vec<Vec3>>,
        counts: &mut Option<Vec<u32>>,
        indices: &mut Option<Vec<u32>>,
        color: &mut Option<[f32; 4]>,
        color_scalar: &mut Option<f64>,
    ) -> Result<(), UsdError> {
        // Attributes may be prefixed by qualifiers/type words and array
        // brackets, for example `uniform token[] xformOpOrder` or
        // `point3f[] points`. The last word before `=` is the attribute name.
        let mut attribute = String::new();
        let mut guard = 0;
        loop {
            match self.peek() {
                Some(Token::Equals) => break,
                Some(Token::Word(word)) => {
                    attribute = word.clone();
                    self.position += 1;
                }
                Some(Token::OpenBracket) => {
                    self.position += 1;
                    if matches!(self.peek(), Some(Token::CloseBracket)) {
                        self.position += 1;
                    }
                }
                other => {
                    return Err(UsdError::Parse(format!("malformed attribute: {other:?}")));
                }
            }
            guard += 1;
            if guard > 6 {
                return Err(UsdError::Parse("malformed attribute".into()));
            }
        }
        if attribute.is_empty() {
            return Err(UsdError::Parse("missing attribute name".into()));
        }
        self.expect(&Token::Equals)?;
        match attribute.as_str() {
            "xformOp:translate" => {
                let values = self.parse_number_list()?;
                if values.len() != 3 {
                    return Err(UsdError::Parse(
                        "xformOp:translate needs three values".into(),
                    ));
                }
                *local_translation = Vec3::new(values[0], values[1], values[2]);
            }
            "xformOp:transform" => {
                let matrix = self.parse_matrix4()?;
                *local_matrix = Some(transform_from_matrix(&matrix)?);
            }
            "points" => {
                let tuples = self.parse_tuple_array()?;
                let mut parsed = Vec::with_capacity(tuples.len());
                for tuple in tuples {
                    if tuple.len() != 3 {
                        return Err(UsdError::Parse("point needs three coordinates".into()));
                    }
                    parsed.push(Vec3::new(tuple[0], tuple[1], tuple[2]));
                }
                *points = Some(parsed);
            }
            "faceVertexIndices" => {
                *indices = Some(self.parse_u32_array()?);
            }
            "faceVertexCounts" => {
                *counts = Some(self.parse_u32_array()?);
            }
            "primvars:displayColor" => {
                let tuples = self.parse_tuple_array()?;
                if let Some(first) = tuples.first() {
                    if first.len() == 3 {
                        *color = Some([first[0] as f32, first[1] as f32, first[2] as f32, 1.0]);
                    } else if first.len() == 1 {
                        *color_scalar = Some(first[0]);
                    }
                }
            }
            _ => self.skip_value()?,
        }
        Ok(())
    }

    fn expect_word(&mut self) -> Result<String, UsdError> {
        match self.next() {
            Some(Token::Word(word)) => Ok(word),
            other => Err(UsdError::Parse(format!("expected word, found {other:?}"))),
        }
    }

    fn parse_number(&mut self) -> Result<f64, UsdError> {
        let word = self.expect_word()?;
        let value = word
            .parse::<f64>()
            .map_err(|_| UsdError::Parse(format!("expected number, found `{word}`")))?;
        if !value.is_finite() {
            return Err(UsdError::NonFinite);
        }
        Ok(value)
    }

    fn parse_number_list(&mut self) -> Result<Vec<f64>, UsdError> {
        self.expect(&Token::OpenParen)?;
        let mut values = Vec::new();
        loop {
            match self.peek() {
                Some(Token::CloseParen) => {
                    self.position += 1;
                    break;
                }
                Some(Token::Comma) => {
                    self.position += 1;
                }
                Some(Token::Word(_)) => values.push(self.parse_number()?),
                other => return Err(UsdError::Parse(format!("expected number, found {other:?}"))),
            }
        }
        Ok(values)
    }

    fn parse_u32_array(&mut self) -> Result<Vec<u32>, UsdError> {
        self.expect(&Token::OpenBracket)?;
        let mut values = Vec::new();
        loop {
            match self.peek() {
                Some(Token::CloseBracket) => {
                    self.position += 1;
                    break;
                }
                Some(Token::Comma) => {
                    self.position += 1;
                }
                Some(Token::Word(_)) => {
                    let value = self.parse_number()?;
                    if value < 0.0 || value.fract() != 0.0 {
                        return Err(UsdError::Parse("expected a non-negative integer".into()));
                    }
                    values.push(value as u32);
                }
                other => {
                    return Err(UsdError::Parse(format!(
                        "expected integer, found {other:?}"
                    )))
                }
            }
        }
        Ok(values)
    }

    fn parse_tuple_array(&mut self) -> Result<Vec<Vec<f64>>, UsdError> {
        self.expect(&Token::OpenBracket)?;
        let mut tuples = Vec::new();
        loop {
            match self.peek() {
                Some(Token::CloseBracket) => {
                    self.position += 1;
                    break;
                }
                Some(Token::Comma) => {
                    self.position += 1;
                }
                Some(Token::OpenParen) => tuples.push(self.parse_number_list()?),
                other => return Err(UsdError::Parse(format!("expected tuple, found {other:?}"))),
            }
        }
        Ok(tuples)
    }

    fn parse_matrix4(&mut self) -> Result<[f64; 16], UsdError> {
        self.expect(&Token::OpenParen)?;
        let mut rows = Vec::with_capacity(4);
        loop {
            match self.peek() {
                Some(Token::CloseParen) => {
                    self.position += 1;
                    break;
                }
                Some(Token::Comma) => {
                    self.position += 1;
                }
                Some(Token::OpenParen) => rows.push(self.parse_number_list()?),
                other => {
                    return Err(UsdError::Parse(format!(
                        "expected matrix row, found {other:?}"
                    )))
                }
            }
        }
        if rows.len() != 4 || rows.iter().any(|row| row.len() != 4) {
            return Err(UsdError::Parse(
                "matrix4d must have four rows of four".into(),
            ));
        }
        let mut matrix = [0.0; 16];
        for (row, values) in rows.iter().enumerate() {
            for (column, value) in values.iter().enumerate() {
                matrix[row * 4 + column] = *value;
            }
        }
        Ok(matrix)
    }

    fn skip_value(&mut self) -> Result<(), UsdError> {
        match self.peek() {
            Some(Token::OpenParen) => self.skip_balanced(Token::OpenParen, Token::CloseParen),
            Some(Token::OpenBracket) => self.skip_balanced(Token::OpenBracket, Token::CloseBracket),
            Some(_) => {
                self.position += 1;
                Ok(())
            }
            None => Ok(()),
        }
    }

    fn skip_balanced(&mut self, open: Token, close: Token) -> Result<(), UsdError> {
        let mut depth = 0usize;
        while let Some(token) = self.next() {
            if token == open {
                depth += 1;
            } else if token == close {
                depth -= 1;
                if depth == 0 {
                    return Ok(());
                }
            }
        }
        Err(UsdError::Parse("unbalanced delimiters".into()))
    }
}

fn local_transform(translation: Vec3, matrix: Option<Transform3>) -> Transform3 {
    match matrix {
        None => Transform3::from_translation_rotation(translation, Quat::IDENTITY),
        Some(matrix) => {
            Transform3::from_translation_rotation(translation + matrix.translation, matrix.rotation)
        }
    }
}

fn transform_from_matrix(matrix: &[f64; 16]) -> Result<Transform3, UsdError> {
    let rotation = [
        [matrix[0], matrix[1], matrix[2]],
        [matrix[4], matrix[5], matrix[6]],
        [matrix[8], matrix[9], matrix[10]],
    ];
    let translation = Vec3::new(matrix[3], matrix[7], matrix[11]);
    if !translation.is_finite() || rotation.iter().flatten().any(|value| !value.is_finite()) {
        return Err(UsdError::NonFinite);
    }
    let quaternion = quat_from_rotation_matrix(&rotation)?;
    Ok(Transform3::from_translation_rotation(
        translation,
        quaternion,
    ))
}

fn quat_from_rotation_matrix(rotation: &[[f64; 3]; 3]) -> Result<Quat, UsdError> {
    let trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
    let (x, y, z, w);
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        w = 0.25 * s;
        x = (rotation[2][1] - rotation[1][2]) / s;
        y = (rotation[0][2] - rotation[2][0]) / s;
        z = (rotation[1][0] - rotation[0][1]) / s;
    } else if rotation[0][0] > rotation[1][1] && rotation[0][0] > rotation[2][2] {
        let s = (1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]).sqrt() * 2.0;
        w = (rotation[2][1] - rotation[1][2]) / s;
        x = 0.25 * s;
        y = (rotation[0][1] + rotation[1][0]) / s;
        z = (rotation[0][2] + rotation[2][0]) / s;
    } else if rotation[1][1] > rotation[2][2] {
        let s = (1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]).sqrt() * 2.0;
        w = (rotation[0][2] - rotation[2][0]) / s;
        x = (rotation[0][1] + rotation[1][0]) / s;
        y = 0.25 * s;
        z = (rotation[1][2] + rotation[2][1]) / s;
    } else {
        let s = (1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]).sqrt() * 2.0;
        w = (rotation[1][0] - rotation[0][1]) / s;
        x = (rotation[0][2] + rotation[2][0]) / s;
        y = (rotation[1][2] + rotation[2][1]) / s;
        z = 0.25 * s;
    }
    if !x.is_finite() || !y.is_finite() || !z.is_finite() || !w.is_finite() {
        return Err(UsdError::Unsupported("non-rigid xform matrix".into()));
    }
    let quaternion = Quat::from_xyzw(x, y, z, w);
    let length = quaternion.length();
    if !length.is_finite() || length < 1.0e-9 {
        return Err(UsdError::Unsupported("degenerate xform rotation".into()));
    }
    Ok(quaternion / length)
}

/// Triangulates USD polygon faces (fan per face) into flat triples.
fn triangulate(
    counts: Option<&[u32]>,
    indices: Option<&[u32]>,
    vertex_count: usize,
) -> Option<Vec<u32>> {
    let indices = indices?;
    let mut triangles = Vec::new();
    if let Some(counts) = counts {
        let mut offset = 0usize;
        for count in counts {
            let count = *count as usize;
            if count < 3 {
                return None;
            }
            if offset + count > indices.len() {
                return None;
            }
            for i in 1..(count - 1) {
                triangles.push(indices[offset]);
                triangles.push(indices[offset + i]);
                triangles.push(indices[offset + i + 1]);
            }
            offset += count;
        }
        if offset != indices.len() {
            return None;
        }
    } else {
        if indices.len() % 3 != 0 {
            return None;
        }
        triangles.extend_from_slice(indices);
    }
    if triangles
        .iter()
        .any(|index| *index as usize >= vertex_count)
    {
        return None;
    }
    Some(triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const LAYER: &str = r#"
#usda 1.0
(
    defaultPrim = "World"
)

def Xform "World"
{
    double3 xformOp:translate = (1.0, 2.0, 3.0)
    def Xform "Part"
    {
        double3 xformOp:translate = (0.5, 0.0, -0.5)
        def Mesh "Plate"
        {
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)]
            float3[] primvars:displayColor = [(0.2, 0.4, 0.6)]
        }
    }
}
"#;

    #[test]
    fn parses_nested_transforms_and_triangulates() {
        let scene = parse_usda(LAYER.as_bytes()).expect("parse");
        assert_eq!(scene.meshes.len(), 1);
        let mesh = &scene.meshes[0];
        assert_eq!(mesh.name, "/World/Part/Plate");
        assert_eq!(mesh.triangle_count(), 2);
        assert_relative_eq!(mesh.points[0].x, 1.5, epsilon = 1e-9);
        assert_relative_eq!(mesh.points[0].y, 2.0, epsilon = 1e-9);
        assert_relative_eq!(mesh.points[0].z, 2.5, epsilon = 1e-9);
        let color = mesh.color_rgba.expect("color");
        assert_relative_eq!(color[0], 0.2, epsilon = 1e-6);
    }

    #[test]
    fn matrix_transform_supplies_translation_and_rotation() {
        let layer = r#"
#usda 1.0
def Mesh "Tri"
{
    matrix4d xformOp:transform = (
        (0, -1, 0, 1),
        (1, 0, 0, 2),
        (0, 0, 1, 3),
        (0, 0, 0, 1)
    )
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(1, 0, 0), (0, 1, 0), (0, 0, 0)]
}
"#;
        let scene = parse_usda(layer.as_bytes()).expect("parse");
        let mesh = &scene.meshes[0];
        // Point (1,0,0) -> rotate +90 deg about z -> (0,1,0) + (1,2,3).
        assert_relative_eq!(mesh.points[0].x, 1.0, epsilon = 1e-9);
        assert_relative_eq!(mesh.points[0].y, 3.0, epsilon = 1e-9);
        assert_relative_eq!(mesh.points[0].z, 3.0, epsilon = 1e-9);
    }

    #[test]
    fn obj_export_is_one_indexed_and_deterministic() {
        let scene = parse_usda(LAYER.as_bytes()).expect("parse");
        let obj = scene.meshes[0].to_obj();
        assert!(obj.contains("f 1 2 3\n"));
        assert_eq!(obj, scene.meshes[0].to_obj());
    }

    #[test]
    fn rejects_unsupported_or_empty_layers() {
        assert!(matches!(parse_usda(b"not usd\n"), Err(UsdError::NotUsda)));
        assert!(matches!(
            parse_usda(b"#usda 1.0\ndef Xform \"Empty\" {}\n"),
            Err(UsdError::EmptyScene)
        ));
    }
}
