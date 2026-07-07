//! URI resolution for mesh assets referenced from URDF.

use std::path::{Path, PathBuf};

/// Resolves a URDF mesh URI to a filesystem path.
///
/// Supported forms:
/// - `package://package_name/relative/path` → `{package_root}/relative/path`
/// - `file:///absolute/path` → `/absolute/path`
/// - plain relative paths → `{package_root}/relative/path`
/// - absolute paths → used as-is
pub fn resolve_package_uri(uri: &str, package_root: &Path) -> PathBuf {
    if let Some(rest) = uri.strip_prefix("package://") {
        let relative = rest.split_once('/').map(|(_, path)| path).unwrap_or(rest);
        return package_root.join(relative);
    }

    if let Some(path) = uri.strip_prefix("file://") {
        return PathBuf::from(path);
    }

    let path = Path::new(uri);
    if path.is_absolute() {
        return path.to_path_buf();
    }

    package_root.join(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    #[test]
    fn package_uri_strips_package_name() {
        let root = Path::new("/assets/mesh_diff_drive");
        let resolved = resolve_package_uri("package://mesh_diff_drive/meshes/base_link.stl", root);
        assert_eq!(
            resolved,
            PathBuf::from("/assets/mesh_diff_drive/meshes/base_link.stl")
        );
    }

    #[test]
    fn file_uri_maps_to_absolute_path() {
        let resolved = resolve_package_uri("file:///tmp/box.stl", Path::new("/ignored"));
        assert_eq!(resolved, PathBuf::from("/tmp/box.stl"));
    }

    #[test]
    fn relative_uri_joins_package_root() {
        let root = Path::new("/assets/robots/lekiwi");
        let resolved = resolve_package_uri("meshes/base_plate.stl", root);
        assert_eq!(
            resolved,
            PathBuf::from("/assets/robots/lekiwi/meshes/base_plate.stl")
        );
    }
}
