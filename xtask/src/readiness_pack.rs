//! Safe authoring helpers for an external RNE 1.0 readiness evidence pack.

use super::{release_readiness, workspace_root};
use anyhow::{bail, Context, Result};
use sha2::{Digest, Sha256};
use std::ffi::OsString;
use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::path::{Component, Path, PathBuf};

const MANIFEST_NAME: &str = "one-zero-readiness.toml";
const BASELINE_MANIFEST: &str = "release/one-zero-readiness.toml";
const COMPATIBILITY_SOURCE: &str = "release/evidence/compatibility-report-v1.json";
const COMPATIBILITY_DESTINATION: &str = "evidence/compatibility-report-v1.json";

#[derive(Debug, Eq, PartialEq)]
struct StagedEvidence {
    relative_path: String,
    sha256: String,
    size_bytes: u64,
}

/// Creates or adds a file to an external 1.0-readiness evidence pack.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = workspace_root()?;
    let command = args
        .next()
        .context("readiness-pack requires a command: init or stage; use readiness-pack --help")?;
    match command.as_str() {
        "init" => {
            let output = parse_init_options(args, &root)?;
            init_pack(&root, &output)?;
            println!(
                "readiness pack initialized: manifest={}",
                output.join(MANIFEST_NAME).display()
            );
            Ok(())
        }
        "stage" => {
            let options = parse_stage_options(args, &root)?;
            let staged = stage_file(&root, &options.pack, &options.source, &options.path)?;
            println!("{}", inline_reference(&staged));
            println!("size_bytes = {}", staged.size_bytes);
            Ok(())
        }
        "--help" | "-h" => {
            print_usage();
            Ok(())
        }
        other => bail!("unknown readiness-pack command: {other}"),
    }
}

#[derive(Debug)]
struct StageOptions {
    pack: PathBuf,
    source: PathBuf,
    path: String,
}

fn print_usage() {
    println!("readiness-pack init --output DIR");
    println!("readiness-pack stage --pack DIR --source FILE --path FORWARD/SLASH/PATH");
}

fn parse_init_options(args: &mut impl Iterator<Item = String>, root: &Path) -> Result<PathBuf> {
    let mut output = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--output" => {
                set_once(
                    &mut output,
                    absolute_from(root, next_value(args, "--output")?),
                    "--output",
                )?;
            }
            other => bail!("unknown readiness-pack init argument: {other}"),
        }
    }
    output.context("readiness-pack init requires --output DIR")
}

fn parse_stage_options(
    args: &mut impl Iterator<Item = String>,
    root: &Path,
) -> Result<StageOptions> {
    let mut pack = None;
    let mut source = None;
    let mut path = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--pack" => set_once(
                &mut pack,
                absolute_from(root, next_value(args, "--pack")?),
                "--pack",
            )?,
            "--source" => set_once(
                &mut source,
                absolute_from(root, next_value(args, "--source")?),
                "--source",
            )?,
            "--path" => set_once(&mut path, next_value(args, "--path")?, "--path")?,
            other => bail!("unknown readiness-pack stage argument: {other}"),
        }
    }
    Ok(StageOptions {
        pack: pack.context("readiness-pack stage requires --pack DIR")?,
        source: source.context("readiness-pack stage requires --source FILE")?,
        path: path.context("readiness-pack stage requires --path RELATIVE/PATH")?,
    })
}

fn set_once<T>(slot: &mut Option<T>, value: T, option: &str) -> Result<()> {
    anyhow::ensure!(slot.is_none(), "{option} may be specified only once");
    *slot = Some(value);
    Ok(())
}

fn next_value(args: &mut impl Iterator<Item = String>, option: &str) -> Result<String> {
    args.next()
        .with_context(|| format!("{option} requires a value"))
}

fn absolute_from(root: &Path, path: String) -> PathBuf {
    let path = PathBuf::from(path);
    if path.is_absolute() {
        path
    } else {
        root.join(path)
    }
}

fn init_pack(root: &Path, output: &Path) -> Result<()> {
    release_readiness::validate_committed_manifest(root)?;
    ensure_missing(output, "readiness pack")?;
    let parent = output
        .parent()
        .context("readiness pack output must have a parent directory")?;
    fs::create_dir_all(parent)
        .with_context(|| format!("create readiness pack parent {}", parent.display()))?;
    let staging = staging_sibling(output, "rne-init")?;
    ensure_missing(&staging, "readiness pack staging path")?;
    fs::create_dir(&staging)
        .with_context(|| format!("create readiness pack staging {}", staging.display()))?;

    copy_regular_file(&root.join(BASELINE_MANIFEST), &staging.join(MANIFEST_NAME))?;
    let compatibility_destination = staging.join(COMPATIBILITY_DESTINATION);
    fs::create_dir_all(
        compatibility_destination
            .parent()
            .expect("compatibility destination has a parent"),
    )?;
    copy_regular_file(&root.join(COMPATIBILITY_SOURCE), &compatibility_destination)?;
    fs::rename(&staging, output).with_context(|| {
        format!(
            "publish readiness pack {} -> {}; staging was retained",
            staging.display(),
            output.display()
        )
    })?;
    Ok(())
}

fn stage_file(root: &Path, pack: &Path, source: &Path, path: &str) -> Result<StagedEvidence> {
    release_readiness::validate_manifest_path(root, &pack.join(MANIFEST_NAME))?;
    let canonical_pack = fs::canonicalize(pack)
        .with_context(|| format!("resolve readiness pack {}", pack.display()))?;
    anyhow::ensure!(canonical_pack.is_dir(), "readiness pack is not a directory");
    let relative = validate_relative_path(path)?;
    anyhow::ensure!(
        relative != Path::new(MANIFEST_NAME),
        "the readiness manifest cannot be replaced by readiness-pack stage"
    );
    let source_metadata = fs::symlink_metadata(source)
        .with_context(|| format!("inspect evidence source {}", source.display()))?;
    anyhow::ensure!(
        source_metadata.is_file() && !source_metadata.file_type().is_symlink(),
        "evidence source must be a regular non-symlink file: {}",
        source.display()
    );
    anyhow::ensure!(
        source_metadata.len() <= release_readiness::MAX_EVIDENCE_BYTES,
        "evidence source exceeds {} bytes: {}",
        release_readiness::MAX_EVIDENCE_BYTES,
        source.display()
    );
    let destination = prepare_destination(&canonical_pack, &relative)?;

    let destination_parent = destination
        .parent()
        .expect("validated staged evidence has a parent");
    let mut temporary = tempfile::Builder::new()
        .prefix(".rne-part-")
        .tempfile_in(destination_parent)
        .with_context(|| {
            format!(
                "create private staged evidence in {}",
                destination_parent.display()
            )
        })?;
    let staged = copy_and_hash(source, temporary.as_file_mut())?;
    temporary.as_file_mut().sync_all()?;
    temporary
        .persist_noclobber(&destination)
        .map_err(|error| error.error)
        .with_context(|| {
            format!(
                "publish staged evidence without replacing {}",
                destination.display()
            )
        })?;
    Ok(StagedEvidence {
        relative_path: path.to_string(),
        sha256: staged.0,
        size_bytes: staged.1,
    })
}

fn validate_relative_path(path: &str) -> Result<PathBuf> {
    anyhow::ensure!(
        !path.is_empty() && !path.contains('\\') && !path.chars().any(char::is_control),
        "staged evidence path must be a non-empty forward-slash relative path"
    );
    anyhow::ensure!(
        path.split('/')
            .all(|component| !component.is_empty() && component != "." && component != ".."),
        "staged evidence path must use canonical non-empty components: {path}"
    );
    let relative = PathBuf::from(path);
    anyhow::ensure!(
        !relative.is_absolute()
            && relative
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "staged evidence path escapes its pack: {path}"
    );
    Ok(relative)
}

fn prepare_destination(pack: &Path, relative: &Path) -> Result<PathBuf> {
    let mut current = pack.to_path_buf();
    let parent = relative
        .parent()
        .expect("validated relative evidence path has a parent");
    for component in parent.components() {
        let Component::Normal(component) = component else {
            unreachable!("relative path was validated")
        };
        current.push(component);
        match fs::symlink_metadata(&current) {
            Ok(metadata) => anyhow::ensure!(
                metadata.is_dir() && !metadata.file_type().is_symlink(),
                "staged evidence parent must be a regular directory: {}",
                current.display()
            ),
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                fs::create_dir(&current).with_context(|| {
                    format!("create staged evidence directory {}", current.display())
                })?;
            }
            Err(error) => {
                return Err(error).with_context(|| format!("inspect {}", current.display()))
            }
        }
    }
    let destination = pack.join(relative);
    ensure_missing(&destination, "staged evidence")?;
    Ok(destination)
}

fn ensure_missing(path: &Path, label: &str) -> Result<()> {
    match fs::symlink_metadata(path) {
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error).with_context(|| format!("inspect {label} {}", path.display())),
        Ok(_) => bail!("refusing to overwrite existing {label}: {}", path.display()),
    }
}

fn copy_regular_file(source: &Path, destination: &Path) -> Result<()> {
    let metadata = fs::symlink_metadata(source)
        .with_context(|| format!("inspect baseline file {}", source.display()))?;
    anyhow::ensure!(
        metadata.is_file() && !metadata.file_type().is_symlink(),
        "baseline file must be a regular non-symlink file: {}",
        source.display()
    );
    fs::copy(source, destination).with_context(|| {
        format!(
            "copy readiness baseline {} -> {}",
            source.display(),
            destination.display()
        )
    })?;
    Ok(())
}

fn copy_and_hash(source: &Path, output: &mut File) -> Result<(String, u64)> {
    let mut input =
        File::open(source).with_context(|| format!("open evidence source {}", source.display()))?;
    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    let mut size_bytes = 0_u64;
    loop {
        let read = input.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        size_bytes = size_bytes
            .checked_add(u64::try_from(read)?)
            .context("staged evidence size overflow")?;
        anyhow::ensure!(
            size_bytes <= release_readiness::MAX_EVIDENCE_BYTES,
            "evidence source grew beyond {} bytes while staging",
            release_readiness::MAX_EVIDENCE_BYTES
        );
        output.write_all(&buffer[..read])?;
        hasher.update(&buffer[..read]);
    }
    output.flush()?;
    Ok((format!("sha256:{:x}", hasher.finalize()), size_bytes))
}

fn staging_sibling(path: &Path, marker: &str) -> Result<PathBuf> {
    let name = path
        .file_name()
        .context("readiness pack output must have a directory name")?;
    let mut staging_name = OsString::from(".");
    staging_name.push(name);
    staging_name.push(format!(".{marker}-{}", std::process::id()));
    Ok(path.with_file_name(staging_name))
}

fn inline_reference(staged: &StagedEvidence) -> String {
    format!(
        "{{ path = {}, sha256 = {} }}",
        toml_string(&staged.relative_path),
        toml_string(&staged.sha256)
    )
}

fn toml_string(value: &str) -> String {
    toml::Value::String(value.to_string()).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initializes_an_external_pack_from_the_honest_baseline() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();

        assert_eq!(
            fs::read(pack.join(MANIFEST_NAME)).unwrap(),
            fs::read(root.join(BASELINE_MANIFEST)).unwrap()
        );
        assert_eq!(
            fs::read(pack.join(COMPATIBILITY_DESTINATION)).unwrap(),
            fs::read(root.join(COMPATIBILITY_SOURCE)).unwrap()
        );
        assert!(init_pack(&root, &pack).is_err());
    }

    #[test]
    fn stages_bytes_once_and_returns_a_canonical_reference() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();
        let source = temp.path().join("controller report.json");
        fs::write(&source, b"external evidence\n").unwrap();

        let staged = stage_file(&root, &pack, &source, "plugins/controller/report.json").unwrap();
        assert_eq!(
            staged,
            StagedEvidence {
                relative_path: "plugins/controller/report.json".to_string(),
                sha256: "sha256:e60b658f8e64bc576589f0b066c7b019a42b624650a328fcf598ef6cf02b4dbe"
                    .to_string(),
                size_bytes: 18,
            }
        );
        assert_eq!(
            fs::read(pack.join("plugins/controller/report.json")).unwrap(),
            b"external evidence\n"
        );
        assert_eq!(
            inline_reference(&staged),
            "{ path = \"plugins/controller/report.json\", sha256 = \"sha256:e60b658f8e64bc576589f0b066c7b019a42b624650a328fcf598ef6cf02b4dbe\" }"
        );
        assert!(stage_file(&root, &pack, &source, "plugins/controller/report.json").is_err());
    }

    #[test]
    fn rejects_unsafe_destinations_and_oversized_sources() {
        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();
        let source = temp.path().join("evidence.bin");
        fs::write(&source, b"evidence").unwrap();

        for path in [
            "",
            "../escape",
            "nested\\escape",
            "/absolute",
            "nested//file",
            "nested/./file",
            "trailing/",
            "line\nbreak",
        ] {
            assert!(stage_file(&root, &pack, &source, path).is_err());
        }
        assert!(stage_file(&root, &pack, &source, MANIFEST_NAME).is_err());

        let oversized = temp.path().join("oversized.bin");
        File::create(&oversized)
            .unwrap()
            .set_len(release_readiness::MAX_EVIDENCE_BYTES + 1)
            .unwrap();
        assert!(stage_file(&root, &pack, &oversized, "large/file.bin").is_err());
        assert!(!pack.join("large/file.bin").exists());
    }

    #[cfg(unix)]
    #[test]
    fn rejects_symlinked_sources_and_destination_parents() {
        use std::os::unix::fs::symlink;

        let root = workspace_root().unwrap();
        let temp = tempfile::tempdir().unwrap();
        let pack = temp.path().join("readiness");
        init_pack(&root, &pack).unwrap();
        let source = temp.path().join("evidence.bin");
        fs::write(&source, b"evidence").unwrap();
        let source_link = temp.path().join("evidence-link.bin");
        symlink(&source, &source_link).unwrap();
        assert!(stage_file(&root, &pack, &source_link, "safe/file.bin").is_err());

        let outside = temp.path().join("outside");
        fs::create_dir(&outside).unwrap();
        symlink(&outside, pack.join("linked")).unwrap();
        assert!(stage_file(&root, &pack, &source, "linked/file.bin").is_err());
        assert!(!outside.join("file.bin").exists());
    }

    #[test]
    fn option_parsers_require_each_input_exactly_once() {
        let root = Path::new("workspace");
        assert!(parse_init_options(&mut Vec::<String>::new().into_iter(), root).is_err());
        assert!(parse_stage_options(
            &mut ["--pack", "pack", "--source", "file"]
                .map(str::to_string)
                .into_iter(),
            root
        )
        .is_err());
        assert!(parse_init_options(
            &mut ["--output", "one", "--output", "two"]
                .map(str::to_string)
                .into_iter(),
            root
        )
        .is_err());
    }
}
