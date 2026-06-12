mod bridge;
mod convert;

fn main() {
    if let Err(error) = bridge::run() {
        eprintln!("rne_ros2_node error: {error:#}");
        std::process::exit(1);
    }
}
