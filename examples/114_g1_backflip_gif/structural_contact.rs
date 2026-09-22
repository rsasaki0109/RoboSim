//! Single-robot structural filtering derived solely from the authored joint graph.
use rne_physics::CollisionGroups;

pub(super) fn masks(link_count: usize, edges: &[(usize, usize, bool)]) -> Vec<CollisionGroups> {
    let mut group: Vec<_> = (0..link_count).collect();
    for &(a, b, fixed) in edges {
        assert!(a < link_count && b < link_count);
        if fixed {
            let low = group[a].min(group[b]);
            let high = group[a].max(group[b]);
            for label in &mut group {
                if *label == high {
                    *label = low;
                }
            }
        }
    }
    let mut unique = group.clone();
    unique.sort_unstable();
    unique.dedup();
    // One spare bit ensures default environmental colliders remain accepted.
    assert!(
        unique.len() <= 31,
        "structural probe supports at most 31 rigid clusters"
    );
    let bits: Vec<_> = group
        .iter()
        .map(|g| 1_u32 << unique.binary_search(g).unwrap())
        .collect();
    let mut excluded = bits.clone();
    for &(a, b, _) in edges {
        for i in 0..link_count {
            if group[i] == group[a] {
                excluded[i] |= bits[b];
            }
            if group[i] == group[b] {
                excluded[i] |= bits[a];
            }
        }
    }
    bits.into_iter()
        .zip(excluded)
        .map(|(memberships, excluded)| CollisionGroups {
            memberships,
            filter: !excluded,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn interacts(a: CollisionGroups, b: CollisionGroups) -> bool {
        a.memberships & b.filter != 0 && b.memberships & a.filter != 0
    }
    #[test]
    fn structural_groups_preserve_nonadjacent_self_and_environment_contacts() {
        let edges = [
            (0, 1, true),
            (0, 2, false),
            (2, 3, false),
            (2, 4, true),
            (0, 5, false),
        ];
        let groups = masks(6, &edges);
        for (a, b) in [(0, 1), (0, 2), (1, 4), (2, 3), (3, 4), (2, 4)] {
            assert!(!interacts(groups[a], groups[b]));
        }
        for (a, b) in [(0, 3), (1, 3), (2, 5), (3, 5), (4, 5)] {
            assert!(interacts(groups[a], groups[b]));
        }
        for group in &groups {
            assert!(interacts(*group, CollisionGroups::default()));
        }
        let reverse: Vec<_> = edges.into_iter().rev().collect();
        assert_eq!(groups, masks(6, &reverse));
    }
    #[test]
    #[should_panic(expected = "at most 31")]
    fn refuses_insufficient_group_bits() {
        masks(32, &[]);
    }
}
