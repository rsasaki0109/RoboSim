# Robot Native Engine — common developer commands

default:
    @just ci

ci:
    cargo run -p xtask -- ci

fmt:
    cargo fmt --all

clippy:
    cargo clippy --workspace --all-targets -- -D warnings

test:
    cargo test --workspace

check:
    cargo check --workspace
