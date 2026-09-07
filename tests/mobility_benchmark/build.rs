use sha2::{Digest, Sha256};
use std::{env, fs, path::Path, process::Command};

fn output(root: &Path, program: &str, args: &[&str]) -> Option<String> {
    let result = Command::new(program)
        .current_dir(root)
        .args(args)
        .output()
        .ok()?;
    result
        .status
        .success()
        .then(|| String::from_utf8_lossy(&result.stdout).trim().to_owned())
}

fn main() {
    let manifest = env::var("CARGO_MANIFEST_DIR").unwrap();
    let root = Path::new(&manifest).join("../..").canonicalize().unwrap();
    for path in [
        "Cargo.lock",
        "Cargo.toml",
        "crates",
        "adapters",
        "tools",
        "tests",
        "xtask",
    ] {
        println!("cargo:rerun-if-changed={}", root.join(path).display());
    }
    for name in ["HEAD".to_owned(), "index".to_owned()]
        .into_iter()
        .chain(output(&root, "git", &["symbolic-ref", "-q", "HEAD"]))
    {
        if let Some(path) = output(&root, "git", &["rev-parse", "--git-path", &name]) {
            println!("cargo:rerun-if-changed={}", root.join(path).display());
        }
    }
    let revision =
        output(&root, "git", &["rev-parse", "HEAD"]).unwrap_or_else(|| "unavailable".into());
    // HANDOFF_CODEX.md is intentionally local-only user documentation.
    let clean = output(
        &root,
        "git",
        &[
            "status",
            "--porcelain",
            "--untracked-files=all",
            "--",
            ".",
            ":(exclude)HANDOFF_CODEX.md",
        ],
    )
    .is_some_and(|status| status.is_empty());
    let rustc = env::var("RUSTC").unwrap();
    let compiler = output(&root, &rustc, &["--version"]).unwrap_or_else(|| "unavailable".into());
    let lock = format!(
        "{:x}",
        Sha256::digest(fs::read(root.join("Cargo.lock")).unwrap())
    );
    for (name, value) in [
        ("RNE_MOBILITY_BUILD_REVISION", revision),
        ("RNE_MOBILITY_BUILD_CLEAN", clean.to_string()),
        ("RNE_MOBILITY_BUILD_COMPILER", compiler),
        ("RNE_MOBILITY_BUILD_TARGET", env::var("TARGET").unwrap()),
        ("RNE_MOBILITY_BUILD_PROFILE", env::var("PROFILE").unwrap()),
        ("RNE_MOBILITY_BUILD_LOCK", lock),
    ] {
        println!("cargo:rustc-env={name}={value}");
    }
}
