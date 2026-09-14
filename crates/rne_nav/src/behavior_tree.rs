//! A minimal deterministic behavior tree.
//!
//! Nav2 uses a BehaviorTree.CPP tree to sequence planning, control, and
//! recovery. This module provides the same control-flow primitives without a
//! runtime dependency: [`Sequence`] and [`Selector`] composites, plus
//! [`Condition`] and [`Action`] leaves built from closures. Ticking is
//! index-ordered and side effects are confined to [`BtContext`], so a tree
//! replays deterministically.

use crate::control::VelocityCommand2d;
use crate::pose2d::Pose2d;

/// Result of ticking a behavior-tree node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BtStatus {
    /// The node completed successfully.
    Success,
    /// The node failed.
    Failure,
    /// The node is still executing and should be ticked again.
    Running,
}

/// Mutable state threaded through a tree tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BtContext {
    /// Number of completed ticks.
    pub ticks: u64,
    /// Current robot pose.
    pub pose: Pose2d,
    /// Command produced by the tree.
    pub command: VelocityCommand2d,
}

/// A behavior-tree node.
pub trait BtNode {
    /// Advances the node by one tick.
    fn tick(&mut self, context: &mut BtContext) -> BtStatus;

    /// Clears any stored progress so the node can run again.
    fn reset(&mut self) {}
}

/// Ticks children in order, succeeding only when all succeed.
pub struct Sequence {
    children: Vec<Box<dyn BtNode>>,
    current: usize,
}

impl Sequence {
    /// Creates a sequence from child nodes.
    pub fn new(children: Vec<Box<dyn BtNode>>) -> Self {
        Self {
            children,
            current: 0,
        }
    }
}

impl BtNode for Sequence {
    fn tick(&mut self, context: &mut BtContext) -> BtStatus {
        for index in self.current..self.children.len() {
            match self.children[index].tick(context) {
                BtStatus::Success => continue,
                BtStatus::Running => {
                    self.current = index;
                    return BtStatus::Running;
                }
                BtStatus::Failure => {
                    self.reset();
                    return BtStatus::Failure;
                }
            }
        }
        self.reset();
        BtStatus::Success
    }

    fn reset(&mut self) {
        self.current = 0;
        for child in &mut self.children {
            child.reset();
        }
    }
}

/// Ticks children in order, succeeding on the first success.
pub struct Selector {
    children: Vec<Box<dyn BtNode>>,
    current: usize,
}

impl Selector {
    /// Creates a selector from child nodes.
    pub fn new(children: Vec<Box<dyn BtNode>>) -> Self {
        Self {
            children,
            current: 0,
        }
    }
}

impl BtNode for Selector {
    fn tick(&mut self, context: &mut BtContext) -> BtStatus {
        for index in self.current..self.children.len() {
            match self.children[index].tick(context) {
                BtStatus::Failure => continue,
                BtStatus::Running => {
                    self.current = index;
                    return BtStatus::Running;
                }
                BtStatus::Success => {
                    self.reset();
                    return BtStatus::Success;
                }
            }
        }
        self.reset();
        BtStatus::Failure
    }

    fn reset(&mut self) {
        self.current = 0;
        for child in &mut self.children {
            child.reset();
        }
    }
}

/// A leaf that succeeds when a predicate holds and fails otherwise.
pub struct Condition<F: FnMut(&BtContext) -> bool> {
    predicate: F,
}

impl<F: FnMut(&BtContext) -> bool> Condition<F> {
    /// Creates a condition leaf.
    pub fn new(predicate: F) -> Self {
        Self { predicate }
    }
}

impl<F: FnMut(&BtContext) -> bool> BtNode for Condition<F> {
    fn tick(&mut self, context: &mut BtContext) -> BtStatus {
        if (self.predicate)(context) {
            BtStatus::Success
        } else {
            BtStatus::Failure
        }
    }
}

/// A leaf that runs a user action.
pub struct Action<F: FnMut(&mut BtContext) -> BtStatus> {
    action: F,
}

impl<F: FnMut(&mut BtContext) -> BtStatus> Action<F> {
    /// Creates an action leaf.
    pub fn new(action: F) -> Self {
        Self { action }
    }
}

impl<F: FnMut(&mut BtContext) -> BtStatus> BtNode for Action<F> {
    fn tick(&mut self, context: &mut BtContext) -> BtStatus {
        (self.action)(context)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Counter {
        remaining: u32,
        ticks: u32,
    }

    impl BtNode for Counter {
        fn tick(&mut self, _context: &mut BtContext) -> BtStatus {
            self.ticks += 1;
            if self.remaining == 0 {
                BtStatus::Success
            } else {
                self.remaining -= 1;
                BtStatus::Running
            }
        }

        fn reset(&mut self) {
            self.remaining = 0;
        }
    }

    fn context() -> BtContext {
        BtContext::default()
    }

    #[test]
    fn sequence_requires_every_child_to_succeed() {
        let mut sequence = Sequence::new(vec![
            Box::new(Condition::new(|_| true)),
            Box::new(Condition::new(|_| true)),
        ]);
        assert_eq!(sequence.tick(&mut context()), BtStatus::Success);

        let mut failing = Sequence::new(vec![
            Box::new(Condition::new(|_| true)),
            Box::new(Condition::new(|_| false)),
        ]);
        assert_eq!(failing.tick(&mut context()), BtStatus::Failure);
    }

    #[test]
    fn selector_falls_through_to_the_next_child() {
        let mut selector = Selector::new(vec![
            Box::new(Condition::new(|_| false)),
            Box::new(Condition::new(|_| true)),
        ]);
        assert_eq!(selector.tick(&mut context()), BtStatus::Success);

        let mut none = Selector::new(vec![
            Box::new(Condition::new(|_| false)),
            Box::new(Condition::new(|_| false)),
        ]);
        assert_eq!(none.tick(&mut context()), BtStatus::Failure);
    }

    #[test]
    fn running_child_resumes_at_the_same_index() {
        let mut sequence = Sequence::new(vec![
            Box::new(Counter {
                remaining: 2,
                ticks: 0,
            }),
            Box::new(Condition::new(|_| true)),
        ]);
        let mut context = context();
        assert_eq!(sequence.tick(&mut context), BtStatus::Running);
        assert_eq!(sequence.tick(&mut context), BtStatus::Running);
        assert_eq!(sequence.tick(&mut context), BtStatus::Success);
    }

    #[test]
    fn action_leaf_can_write_the_context() {
        let mut action = Action::new(|context: &mut BtContext| {
            context.command = VelocityCommand2d::new(0.5, 0.1);
            context.ticks += 1;
            BtStatus::Success
        });
        let mut context = BtContext::default();
        assert_eq!(action.tick(&mut context), BtStatus::Success);
        assert_eq!(context.command, VelocityCommand2d::new(0.5, 0.1));
        assert_eq!(context.ticks, 1);
    }
}
