//! Bounded loopback-only HTTP host for the embedded workbench UI.

use crate::host::{Command, Host};
use anyhow::{bail, ensure, Context, Result};
use std::{
    io::{Read, Write},
    net::{Ipv4Addr, TcpListener, TcpStream},
    time::Duration,
};

const MAX_HEADER_BYTES: usize = 8192;
const MAX_BODY_BYTES: usize = 4 * 1024 * 1024;

#[derive(Debug)]
struct Request {
    method: String,
    path: String,
    body: Vec<u8>,
}

fn read_request(stream: &mut impl Read, host: &str) -> Result<Request> {
    let mut bytes = Vec::new();
    let header_end = loop {
        let mut byte = [0];
        stream
            .read_exact(&mut byte)
            .context("incomplete HTTP header")?;
        bytes.push(byte[0]);
        ensure!(bytes.len() <= MAX_HEADER_BYTES, "HTTP header too large");
        if bytes.ends_with(b"\r\n\r\n") {
            break bytes.len();
        }
    };
    let header = std::str::from_utf8(&bytes[..header_end]).context("invalid HTTP header")?;
    let mut lines = header.split("\r\n");
    let first = lines
        .next()
        .context("missing request line")?
        .split_whitespace()
        .collect::<Vec<_>>();
    ensure!(
        first.len() == 3 && first[2] == "HTTP/1.1",
        "HTTP/1.1 required"
    );
    let method = first[0].to_string();
    let path = first[1].to_string();
    ensure!(
        matches!(method.as_str(), "GET" | "POST"),
        "unsupported method"
    );
    let mut length = None;
    let mut host_seen = false;
    let mut json_body = false;
    for line in lines.filter(|line| !line.is_empty()) {
        let (name, value) = line.split_once(':').context("malformed header")?;
        let value = value.trim();
        match name.to_ascii_lowercase().as_str() {
            "host" => {
                ensure!(!host_seen && value == host, "invalid Host");
                host_seen = true;
            }
            "origin" => ensure!(
                value == format!("http://{host}"),
                "cross-origin request rejected"
            ),
            "content-type" => json_body = value.split(';').next() == Some("application/json"),
            "content-length" => {
                ensure!(length.is_none(), "duplicate Content-Length");
                length = Some(value.parse::<usize>().context("invalid Content-Length")?);
            }
            "transfer-encoding" => bail!("streamed request bodies are not supported"),
            _ => {}
        }
    }
    ensure!(host_seen, "Host required");
    let length = length.unwrap_or(0);
    ensure!(length <= MAX_BODY_BYTES, "request exceeds 4 MiB");
    if method == "POST" {
        ensure!(json_body && length > 0, "application/json body required");
    }
    if method == "GET" {
        ensure!(length == 0, "GET body is not supported");
    }
    let mut body = vec![0; length];
    stream
        .read_exact(&mut body)
        .context("incomplete request body")?;
    Ok(Request { method, path, body })
}

fn write_response(
    stream: &mut impl Write,
    status: &str,
    content_type: &str,
    body: &[u8],
) -> Result<()> {
    write!(stream, "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\nConnection: close\r\n\r\n", body.len())?;
    stream.write_all(body)?;
    Ok(())
}

fn handle(stream: &mut TcpStream, host_name: &str, host: &mut Host) -> Result<()> {
    let request = read_request(stream, host_name)?;
    match (request.method.as_str(), request.path.as_str()) {
        ("GET", "/") => write_response(
            stream,
            "200 OK",
            "text/html; charset=utf-8",
            include_bytes!("ui/index.html"),
        ),
        ("GET", "/workbench.js") => write_response(
            stream,
            "200 OK",
            "text/javascript; charset=utf-8",
            include_bytes!("ui/workbench.js"),
        ),
        ("GET", "/workbench.css") => write_response(
            stream,
            "200 OK",
            "text/css; charset=utf-8",
            include_bytes!("ui/workbench.css"),
        ),
        ("GET", "/api/state") => write_response(
            stream,
            "200 OK",
            "application/json",
            &serde_json::to_vec(&host.snapshot()?)?,
        ),
        ("POST", "/api/command") => {
            let command: Command =
                serde_json::from_slice(&request.body).context("invalid workbench command")?;
            host.command(command)?;
            write_response(
                stream,
                "200 OK",
                "application/json",
                &serde_json::to_vec(&host.snapshot()?)?,
            )
        }
        _ => write_response(
            stream,
            "404 Not Found",
            "application/json",
            b"{\"error\":\"not found\"}",
        ),
    }
}

pub(crate) fn serve(mut host: Host, port: u16) -> Result<()> {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, port))?;
    let host_name = listener.local_addr()?.to_string();
    println!("RoboSim workbench: http://{host_name}");
    std::io::stdout().flush()?;
    for stream in listener.incoming() {
        let mut stream = stream?;
        stream.set_read_timeout(Some(Duration::from_secs(3)))?;
        stream.set_write_timeout(Some(Duration::from_secs(5)))?;
        if let Err(error) = handle(&mut stream, &host_name, &mut host) {
            let error = serde_json::to_vec(&serde_json::json!({"error": format!("{error:#}")}))?;
            let _ = write_response(&mut stream, "400 Bad Request", "application/json", &error);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn requests_reject_cross_origin_and_oversized_bodies_before_reading_them() {
        for headers in [
            "Origin: http://unrelated.invalid\r\nContent-Length: 2",
            "Content-Length: 4194305",
            "Content-Length: 2\r\nContent-Length: 2",
            "Transfer-Encoding: chunked",
        ] {
            let request = format!("POST /api/command HTTP/1.1\r\nHost: 127.0.0.1:8000\r\nContent-Type: application/json\r\n{headers}\r\n\r\n{{}}");
            assert!(read_request(&mut request.as_bytes(), "127.0.0.1:8000").is_err());
        }
        let request = b"GET /api/state HTTP/1.1\r\nHost: 127.0.0.1:8000\r\n\r\n";
        assert_eq!(
            read_request(&mut request.as_slice(), "127.0.0.1:8000")
                .unwrap()
                .path,
            "/api/state"
        );
    }
}
