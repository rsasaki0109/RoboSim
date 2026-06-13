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

ci-ros2:
    cargo run -p xtask -- ci-ros2

ci-ros2-bridge:
    cargo run -p xtask -- ci-ros2-bridge

check:
    cargo check --workspace
