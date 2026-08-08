//! Runner control commands for interactive pause, resume, stepping, and reset.
//!
//! The runner consults a [`RunnerControl`] transport at every fixed-step
//! boundary. A transport-neutral [`RunControl`] state machine turns the
//! commands into an [`EpisodeOutcome`], so the fixed-step loop can be driven
//! interactively (or scripted) without coupling the loop to any transport.

/// A command a client can send to a running experiment at a step boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlCommand {
    /// Suspend the fixed-step loop at the next step boundary.
    Pause,
    /// Resume advancing freely.
    Resume,
    /// Advance exactly `frames` steps, then pause again.
    Step {
        /// Number of steps to advance before pausing again.
        frames: u64,
    },
    /// Rebuild the world from the episode's initial conditions and restart from
    /// step 0.
    Reset,
    /// Stop the run gracefully; the current episode is still reported and can
    /// be recorded.
    Quit,
}

/// How the fixed-step loop should proceed at a step boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EpisodeOutcome {
    /// Continue with one more step.
    Advance,
    /// Rebuild the world from the initial conditions and restart the episode.
    Reset,
    /// Stop the run; report the current episode.
    Quit,
}

/// Paused or advancing state of the runner.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RunnerControlState {
    /// The loop advances one step per boundary.
    #[default]
    Running,
    /// The loop blocks until a command resumes it.
    Paused,
}

/// A transport-neutral source of runner control commands.
///
/// A `try_poll` returning `None` means no command is queued. `wait_command`
/// must return the next command, or [`ControlCommand::Quit`] if the transport
/// is exhausted (for example, stdin closed).
pub trait RunnerControl {
    /// Non-blocking: returns the next queued command, if any.
    fn try_poll(&mut self) -> Option<ControlCommand>;

    /// Blocking: waits until the next command is available.
    fn wait_command(&mut self) -> ControlCommand;
}

/// Drives the fixed-step loop from a control transport.
///
/// `checkpoint` is called before each step. While paused it blocks on the
/// transport; otherwise it processes at most one queued command per boundary
/// and reports whether the loop should advance, reset, or quit. Processing at
/// most one command per boundary keeps a burst of scripted commands ordered:
/// each takes effect at the next boundary, like a human typing them one at a
/// time.
pub struct RunControl<'a> {
    transport: &'a mut dyn RunnerControl,
    state: RunnerControlState,
    step_remaining: u64,
}

impl<'a> RunControl<'a> {
    /// Wraps a control transport, initially running freely.
    pub fn new(transport: &'a mut dyn RunnerControl) -> Self {
        Self {
            transport,
            state: RunnerControlState::Running,
            step_remaining: 0,
        }
    }

    /// Wraps a control transport, initially paused until the first command
    /// arrives. Interactive transports use this so the runner never advances
    /// before the client is connected.
    pub fn paused(transport: &'a mut dyn RunnerControl) -> Self {
        Self {
            transport,
            state: RunnerControlState::Paused,
            step_remaining: 0,
        }
    }

    /// The current paused or advancing state.
    pub fn state(&self) -> RunnerControlState {
        self.state
    }

    /// Applies one command, returning a terminal outcome when the run should
    /// reset or quit.
    fn apply(&mut self, command: ControlCommand) -> Option<EpisodeOutcome> {
        match command {
            ControlCommand::Pause => {
                self.state = RunnerControlState::Paused;
                self.step_remaining = 0;
            }
            ControlCommand::Resume => {
                self.state = RunnerControlState::Running;
                self.step_remaining = 0;
            }
            ControlCommand::Step { frames } => {
                self.state = RunnerControlState::Paused;
                self.step_remaining = frames;
            }
            ControlCommand::Reset => return Some(EpisodeOutcome::Reset),
            ControlCommand::Quit => return Some(EpisodeOutcome::Quit),
        }
        None
    }

    /// Consults the control transport at a step boundary and returns how the
    /// loop should proceed. Blocks while paused until a command resumes it.
    ///
    /// An active `step N` budget is never interrupted: queued commands are only
    /// consumed at free boundaries, and at most one queued command is consumed
    /// per boundary, so a burst of scripted commands stays strictly ordered.
    pub fn checkpoint(&mut self) -> EpisodeOutcome {
        let mut consumed = false;
        loop {
            if self.state == RunnerControlState::Paused && self.step_remaining > 0 {
                self.step_remaining -= 1;
                return EpisodeOutcome::Advance;
            }
            if !consumed {
                if let Some(command) = self.transport.try_poll() {
                    consumed = true;
                    if let Some(outcome) = self.apply(command) {
                        return outcome;
                    }
                    continue;
                }
            }
            if self.state == RunnerControlState::Paused && self.step_remaining == 0 {
                let command = self.transport.wait_command();
                if let Some(outcome) = self.apply(command) {
                    return outcome;
                }
                continue;
            }
            return EpisodeOutcome::Advance;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ControlCommand, EpisodeOutcome, RunControl, RunnerControl, RunnerControlState};
    use std::cell::Cell;
    use std::collections::VecDeque;
    use std::rc::Rc;

    /// A scripted transport driven by a queue, counting blocking waits, for
    /// deterministic tests.
    struct ScriptedControl {
        commands: VecDeque<ControlCommand>,
        waits: Rc<Cell<u64>>,
    }

    impl ScriptedControl {
        fn new(commands: Vec<ControlCommand>, waits: Rc<Cell<u64>>) -> Self {
            Self {
                commands: commands.into(),
                waits,
            }
        }
    }

    impl RunnerControl for ScriptedControl {
        fn try_poll(&mut self) -> Option<ControlCommand> {
            self.commands.pop_front()
        }

        fn wait_command(&mut self) -> ControlCommand {
            self.waits.set(self.waits.get() + 1);
            self.commands.pop_front().unwrap_or(ControlCommand::Quit)
        }
    }

    #[test]
    fn runs_freely_without_commands() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(vec![], waits.clone());
        let mut control = RunControl::new(&mut scripted);
        for _ in 0..3 {
            assert_eq!(control.checkpoint(), EpisodeOutcome::Advance);
        }
        assert_eq!(control.state(), RunnerControlState::Running);
        assert_eq!(waits.get(), 0, "free-running must not block");
    }

    #[test]
    fn paused_start_waits_for_the_first_command() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(vec![], waits.clone());
        let mut control = RunControl::paused(&mut scripted);
        assert_eq!(control.state(), RunnerControlState::Paused);
        assert_eq!(
            control.checkpoint(),
            EpisodeOutcome::Quit,
            "paused start with no commands must block, not advance"
        );
        assert_eq!(waits.get(), 1, "paused start must block before any step");
    }

    #[test]
    fn paused_start_consumes_a_queued_step_without_extra_wait() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(
            vec![ControlCommand::Step { frames: 2 }, ControlCommand::Resume],
            waits.clone(),
        );
        let mut control = RunControl::paused(&mut scripted);
        assert_eq!(control.checkpoint(), EpisodeOutcome::Advance);
        assert_eq!(waits.get(), 0, "a queued command is consumed by poll");
        assert_eq!(control.state(), RunnerControlState::Paused);
    }

    #[test]
    fn pause_blocks_then_resume_advances() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(
            vec![ControlCommand::Pause, ControlCommand::Resume],
            waits.clone(),
        );
        let mut control = RunControl::new(&mut scripted);
        assert_eq!(control.checkpoint(), EpisodeOutcome::Advance);
        assert_eq!(waits.get(), 1, "pause must block on the transport");
        assert_eq!(control.state(), RunnerControlState::Running);
    }

    #[test]
    fn lone_pause_blocks_then_quits_when_transport_exhausted() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(vec![ControlCommand::Pause], waits.clone());
        let mut control = RunControl::new(&mut scripted);
        assert_eq!(control.checkpoint(), EpisodeOutcome::Quit);
        assert_eq!(waits.get(), 1, "pause must block until a command arrives");
        assert_eq!(control.state(), RunnerControlState::Paused);
    }

    #[test]
    fn step_advances_exactly_the_requested_frames_then_blocks() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted =
            ScriptedControl::new(vec![ControlCommand::Step { frames: 3 }], waits.clone());
        let mut control = RunControl::new(&mut scripted);
        for _ in 0..3 {
            assert_eq!(control.checkpoint(), EpisodeOutcome::Advance);
        }
        assert_eq!(control.checkpoint(), EpisodeOutcome::Quit);
        assert_eq!(waits.get(), 1, "checkpoint must block after the budget");
        assert_eq!(control.state(), RunnerControlState::Paused);
    }

    #[test]
    fn reset_is_reported_at_the_next_boundary() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(vec![ControlCommand::Reset], waits);
        let mut control = RunControl::new(&mut scripted);
        assert_eq!(control.checkpoint(), EpisodeOutcome::Reset);
    }

    #[test]
    fn quit_is_reported_at_the_next_boundary() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(vec![ControlCommand::Quit], waits);
        let mut control = RunControl::new(&mut scripted);
        assert_eq!(control.checkpoint(), EpisodeOutcome::Quit);
    }

    #[test]
    fn pause_then_quit_reports_quit() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(
            vec![ControlCommand::Pause, ControlCommand::Quit],
            waits.clone(),
        );
        let mut control = RunControl::new(&mut scripted);
        assert_eq!(control.checkpoint(), EpisodeOutcome::Quit);
        assert_eq!(waits.get(), 1, "quit must be consumed while paused");
    }

    #[test]
    fn step_commands_are_applied_in_order_across_boundaries() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(
            vec![
                ControlCommand::Step { frames: 2 },
                ControlCommand::Step { frames: 3 },
            ],
            waits.clone(),
        );
        let mut control = RunControl::new(&mut scripted);
        for _ in 0..5 {
            assert_eq!(control.checkpoint(), EpisodeOutcome::Advance);
        }
        assert_eq!(control.checkpoint(), EpisodeOutcome::Quit);
        assert_eq!(waits.get(), 1, "the budget finishes before commands apply");
        assert_eq!(control.state(), RunnerControlState::Paused);
    }

    #[test]
    fn step_zero_consumes_resume_in_the_same_boundary() {
        let waits = Rc::new(Cell::new(0));
        let mut scripted = ScriptedControl::new(
            vec![ControlCommand::Step { frames: 0 }, ControlCommand::Resume],
            waits.clone(),
        );
        let mut control = RunControl::new(&mut scripted);
        assert_eq!(control.checkpoint(), EpisodeOutcome::Advance);
        assert_eq!(waits.get(), 1, "step 0 must block until resumed");
        assert_eq!(control.state(), RunnerControlState::Running);
    }
}
