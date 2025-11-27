use std::collections::HashMap;
use std::hash::Hash;
use std::time::{Duration, Instant};

struct CachedPath<N> {
    path: Vec<N>,
    #[allow(dead_code)]
    cost: f32,
    created: Instant,
    hits: u32,
}

pub struct PathCache<N: Hash + Eq> {
    cache: HashMap<(N, N), CachedPath<N>>,
    max_entries: usize,
    max_age: Duration,
}

impl<N: Hash + Eq + Clone> PathCache<N> {
    pub fn new(max_entries: usize, max_age: Duration) -> Self {
        Self {
            cache: HashMap::new(),
            max_entries,
            max_age,
        }
    }

    pub fn get(&mut self, start: &N, goal: &N) -> Option<&[N]> {
        let key = (start.clone(), goal.clone());
        if let Some(entry) = self.cache.get_mut(&key) {
            if entry.created.elapsed() < self.max_age {
                entry.hits += 1;
                return Some(&entry.path);
            }
        }
        None
    }

    pub fn insert(&mut self, start: N, goal: N, path: Vec<N>, cost: f32) {
        if self.cache.len() >= self.max_entries {
            // Simple eviction strategy: clear all for now to avoid O(N) scan
            // In production, use an LRU cache
            self.cache.clear(); 
        }
        
        self.cache.insert((start, goal), CachedPath {
            path,
            cost,
            created: Instant::now(),
            hits: 0,
        });
    }

    pub fn invalidate_region<F>(&mut self, predicate: F)
    where F: Fn(&N) -> bool {
        // Remove paths that touch the invalidated region
        self.cache.retain(|(start, goal), entry| {
             if predicate(start) || predicate(goal) { return false; }
             
             for node in &entry.path {
                 if predicate(node) { return false; }
             }
             true
        });
    }
    
    pub fn clear(&mut self) {
        self.cache.clear();
    }
}
