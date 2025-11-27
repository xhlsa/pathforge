use std::collections::HashMap;
use std::hash::Hash;
use std::time::{Duration, Instant};

use crate::algorithms::astar::{astar, AStarConfig};
use crate::traits::{Graph, Heuristic, PathResult, PathStatus};

#[derive(Clone)]
struct CachedPath<N> {
    result: PathResult<N>,
    created: Instant,
    hits: u32,
}

pub struct PathCache<N: Hash + Eq> {
    cache: HashMap<(N, N), CachedPath<N>>,
    max_entries: usize,
    max_age: Duration,
}

impl<N: Hash + Eq + Clone> PathCache<N> {
    /// Create a path cache with a maximum size and entry age.
    /// Paths older than `max_age` are treated as misses.
    pub fn new(max_entries: usize, max_age: Duration) -> Self {
        Self {
            cache: HashMap::new(),
            max_entries,
            max_age,
        }
    }

    /// Returns a cached PathResult clone if present and fresh.
    pub fn get(&mut self, start: &N, goal: &N) -> Option<PathResult<N>> {
        let key = (start.clone(), goal.clone());
        if let Some(entry) = self.cache.get_mut(&key) {
            if entry.created.elapsed() < self.max_age {
                entry.hits += 1;
                return Some(entry.result.clone());
            }
        }
        None
    }

    /// Insert a PathResult (typically only on success) into the cache.
    pub fn insert(&mut self, start: N, goal: N, result: PathResult<N>) {
        if result.status != PathStatus::Found {
            return;
        }
        if self.cache.len() >= self.max_entries {
            self.evict_one();
        }
        self.cache.insert(
            (start, goal),
            CachedPath {
                result,
                created: Instant::now(),
                hits: 0,
            },
        );
    }

    /// Evict a single entry using a simple heuristic: oldest or lowest-hit.
    fn evict_one(&mut self) {
        if let Some((evict_key, _)) = self
            .cache
            .iter()
            .min_by_key(|(_, v)| (v.hits, v.created))
            .map(|(k, v)| (k.clone(), v.created))
        {
            self.cache.remove(&evict_key);
        } else {
            self.cache.clear();
        }
    }

    /// Invalidate cached paths that touch nodes matching `predicate`.
    pub fn invalidate_region<F>(&mut self, predicate: F)
    where
        F: Fn(&N) -> bool,
    {
        self.cache.retain(|(start, goal), entry| {
            if predicate(start) || predicate(goal) {
                return false;
            }
            entry.result.path.iter().all(|n| !predicate(n))
        });
    }

    pub fn clear(&mut self) {
        self.cache.clear();
    }

    pub fn len(&self) -> usize {
        self.cache.len()
    }
}

/// Run A* with a simple start/goal cache. Only `Found` results are cached.
pub fn astar_with_cache<G, H>(
    graph: &G,
    heuristic: &H,
    start: G::Node,
    goal: G::Node,
    config: AStarConfig,
    cache: &mut PathCache<G::Node>,
) -> PathResult<G::Node>
where
    G: Graph,
    H: Heuristic<G::Node>,
{
    if let Some(hit) = cache.get(&start, &goal) {
        return hit;
    }

    let result = astar(graph, heuristic, start.clone(), goal.clone(), config);
    if result.status == PathStatus::Found {
        cache.insert(start, goal, result.clone());
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graphs::grid2d::{DiagonalMode, Grid2D, GridPos};
    use crate::heuristics::Diagonal;

    #[test]
    fn caches_and_invalidates() {
        let mut cache = PathCache::new(4, Duration::from_secs(60));
        let grid = Grid2D::new(5, 5, DiagonalMode::Always);
        let start = GridPos { x: 0, y: 0 };
        let goal = GridPos { x: 4, y: 4 };

        // miss initially
        assert!(cache.get(&start, &goal).is_none());

        let res1 = astar_with_cache(
            &grid,
            &Diagonal::default(),
            start,
            goal,
            AStarConfig::default(),
            &mut cache,
        );
        assert_eq!(res1.status, PathStatus::Found);
        assert_eq!(cache.len(), 1);

        // hit after insert
        let res2 = cache.get(&GridPos { x: 0, y: 0 }, &GridPos { x: 4, y: 4 });
        assert!(res2.is_some());

        // invalidate anything touching x=2 should drop the cached path
        cache.invalidate_region(|p: &GridPos| p.x == 2);
        assert!(cache.get(&GridPos { x: 0, y: 0 }, &GridPos { x: 4, y: 4 }).is_none());
    }
}
