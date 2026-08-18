//! Dependency-free deterministic process mock for the installed accelerator kit.

use rne_accelerator_contract::AcceleratorProtocolTranscript;
use serde_json::Value;
use std::io::{BufRead, Write};
use std::path::PathBuf;

const USAGE: &str =
    "rne-accelerator-protocol-mock --transcript protocol-transcript-v1.json | --stall";
const MAX_LINE_BYTES: usize = 16 * 1024 * 1024;

fn main() {
    if let Err(error) = run() {
        eprintln!("accelerator protocol mock failed: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let option = parse_args(std::env::args().skip(1))?;
    if option == MockOption::Stall {
        let mut line = String::new();
        std::io::stdin().lock().read_line(&mut line)?;
        std::thread::sleep(std::time::Duration::from_secs(60));
        return Ok(());
    }
    let MockOption::Transcript { path, extra_output } = option else {
        unreachable!("stall returned above")
    };
    let bytes = std::fs::read(&path)?;
    let transcript = AcceleratorProtocolTranscript::from_json_slice(&bytes)?;
    let stdin = std::io::stdin();
    let mut input = stdin.lock();
    let stdout = std::io::stdout();
    let mut output = stdout.lock();
    for (index, frame) in transcript.frames.iter().enumerate() {
        let Some(line) = read_bounded_line(&mut input)? else {
            return Err(format!("request {index} was not received").into());
        };
        let request: Value = serde_json::from_slice(&line)?;
        if request != frame.request {
            return Err(format!("request {index} differs from transcript").into());
        }
        serde_json::to_writer(&mut output, &frame.response)?;
        output.write_all(b"\n")?;
        output.flush()?;
    }
    if extra_output {
        output.write_all(b"{}\n")?;
        output.flush()?;
    }
    Ok(())
}

fn read_bounded_line(
    reader: &mut impl BufRead,
) -> Result<Option<Vec<u8>>, Box<dyn std::error::Error>> {
    let mut line = Vec::new();
    loop {
        let available = reader.fill_buf()?;
        if available.is_empty() {
            return if line.is_empty() {
                Ok(None)
            } else {
                Err("request ended without a newline".into())
            };
        }
        let take = available
            .iter()
            .position(|byte| *byte == b'\n')
            .map_or(available.len(), |index| index + 1);
        if line.len().saturating_add(take) > MAX_LINE_BYTES {
            return Err(format!("request exceeds {MAX_LINE_BYTES} bytes").into());
        }
        line.extend_from_slice(&available[..take]);
        reader.consume(take);
        if line.last() == Some(&b'\n') {
            line.pop();
            if line.last() == Some(&b'\r') {
                line.pop();
            }
            return Ok(Some(line));
        }
    }
}

#[derive(Debug, Eq, PartialEq)]
enum MockOption {
    Transcript { path: PathBuf, extra_output: bool },
    Stall,
}

fn parse_args(
    mut args: impl Iterator<Item = String>,
) -> Result<MockOption, Box<dyn std::error::Error>> {
    match (
        args.next().as_deref(),
        args.next(),
        args.next(),
        args.next(),
    ) {
        (Some("--transcript"), Some(path), None, None) if !path.is_empty() => {
            Ok(MockOption::Transcript {
                path: PathBuf::from(path),
                extra_output: false,
            })
        }
        (Some("--transcript"), Some(path), Some(flag), None)
            if !path.is_empty() && flag == "--extra-output" =>
        {
            Ok(MockOption::Transcript {
                path: PathBuf::from(path),
                extra_output: true,
            })
        }
        (Some("--stall"), None, None, None) => Ok(MockOption::Stall),
        (Some("--help" | "-h"), None, None, None) => {
            println!("{USAGE}");
            std::process::exit(0);
        }
        _ => Err(format!("usage: {USAGE}").into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parser_requires_exactly_one_bounded_mode() {
        assert_eq!(
            parse_args(
                ["--transcript", "fixture.json"]
                    .into_iter()
                    .map(str::to_string)
            )
            .unwrap(),
            MockOption::Transcript {
                path: PathBuf::from("fixture.json"),
                extra_output: false,
            }
        );
        assert_eq!(
            parse_args(["--stall"].into_iter().map(str::to_string)).unwrap(),
            MockOption::Stall
        );
        assert!(parse_args(std::iter::empty()).is_err());
    }
}
