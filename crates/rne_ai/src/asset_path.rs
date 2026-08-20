use std::path::{Path, PathBuf};

pub(crate) fn bundled_asset_path(relative_path: impl AsRef<Path>) -> PathBuf {
    let relative_path = relative_path.as_ref();
    let mut origins = Vec::with_capacity(2);
    if let Ok(current_dir) = std::env::current_dir() {
        origins.push(current_dir);
    }
    if let Ok(current_exe) = std::env::current_exe() {
        if let Some(parent) = current_exe.parent() {
            origins.push(parent.to_path_buf());
        }
    }

    bundled_asset_path_from_origins(relative_path, origins.iter().map(PathBuf::as_path))
}

fn bundled_asset_path_from_origins<'a>(
    relative_path: &Path,
    origins: impl IntoIterator<Item = &'a Path>,
) -> PathBuf {
    debug_assert!(!relative_path.is_absolute());
    for origin in origins {
        for ancestor in origin.ancestors() {
            let candidate = ancestor.join("assets").join(relative_path);
            if candidate.is_file() {
                return candidate;
            }
        }
    }
    PathBuf::from("assets").join(relative_path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn resolves_from_runtime_origin_without_a_compiled_checkout_path() {
        let root = tempfile::tempdir().expect("temporary bundle");
        let nested = root.path().join("bin/tools");
        let expected = root.path().join("assets/scenes/example.rne.scene.toml");
        fs::create_dir_all(&nested).expect("nested runtime directory");
        fs::create_dir_all(expected.parent().unwrap()).expect("asset directory");
        fs::write(&expected, b"scene").expect("asset fixture");

        assert_eq!(
            bundled_asset_path_from_origins(
                Path::new("scenes/example.rne.scene.toml"),
                [nested.as_path()]
            ),
            expected
        );
    }

    #[test]
    fn unresolved_asset_stays_relative_instead_of_baking_a_checkout_path() {
        let root = tempfile::tempdir().expect("temporary origin");
        assert_eq!(
            bundled_asset_path_from_origins(
                Path::new("scenes/missing.rne.scene.toml"),
                [root.path()]
            ),
            PathBuf::from("assets/scenes/missing.rne.scene.toml")
        );
    }
}
