//! Prints the built-in LeKiwi reference profile or its TaskSpec.

use rne_hardware_lekiwi::{lekiwi_base_task_spec, lekiwi_reference_profile_v1};

fn main() {
    let task_only = match std::env::args().skip(1).collect::<Vec<_>>().as_slice() {
        [] => false,
        [argument] if argument == "--task-only" => true,
        _ => {
            eprintln!("usage: rne-lekiwi-profile [--task-only]");
            std::process::exit(2);
        }
    };
    let result = if task_only {
        serde_json::to_string_pretty(&lekiwi_base_task_spec())
    } else {
        serde_json::to_string_pretty(&lekiwi_reference_profile_v1())
    };
    match result {
        Ok(json) => println!("{json}"),
        Err(error) => {
            eprintln!("failed to serialize LeKiwi profile: {error}");
            std::process::exit(1);
        }
    }
}
