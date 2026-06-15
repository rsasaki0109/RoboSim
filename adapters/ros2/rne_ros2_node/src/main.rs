mod bridge;
mod cmd_vel;
mod convert;
mod sim_control;

fn main() {
    if let Err(error) = bridge::run() {
        eprintln!("rne_ros2_node error: {error:#}");
        std::process::exit(1);
    }
}
