//! In-memory CSR routing graph: construction from OSM ways, largest-SCC
//! filtering, and the versioned on-disk format.
//!
//! Conventions, asserted at every boundary:
//! - coordinates are `(lon, lat)`, matching GeoJSON
//! - distances are metres (f64), accumulated in f64 and only stored as f32
//! - edge weights are milliseconds (u32), so the priority queue stays integral
//! - node ids are dense u32 indices, NOT OSM ids (`osm_id` is a debug side table)

pub mod construct;
pub mod contract;
pub mod grid;
pub mod io;
#[cfg(test)]
mod tests;
pub mod topology;

pub use construct::{build, BuildStats};
pub use contract::{contract, contractible_at, twins_agree, ContractStats};
pub use io::{file_hash, FORMAT_VERSION};
pub use topology::{chain_at, scc, shape_at, Chain, Shape};

pub const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// Great-circle distance in metres between two `(lon, lat)` points.
pub fn haversine(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (lon1, lat1) = (a.0.to_radians(), a.1.to_radians());
    let (lon2, lat2) = (b.0.to_radians(), b.1.to_radians());
    let (dlon, dlat) = (lon2 - lon1, lat2 - lat1);
    let h = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.sqrt().asin()
}

/// Sentinel in `name_id` for an edge whose way had no `name` tag.
pub const NO_NAME: u32 = u32::MAX;
/// Sentinel in `twin` for a one-way edge with no opposite direction.
pub const NO_TWIN: u32 = u32::MAX;

/// Set on the reverse direction of a two-way road: it shares its twin's
/// geometry span and walks it backwards. Sits above the class bits that
/// `osm-parse` owns.
pub const FLAG_GEOM_REVERSED: u8 = 0x40;

/// `node_flags` bits.
pub const NODE_SIGNALS: u8 = 0x01;
pub const NODE_BARRIER: u8 = 0x02;

pub struct Graph {
    // --- per node ---
    pub lon: Vec<f64>,
    pub lat: Vec<f64>,
    /// Internal id -> OSM node id. Debugging only; nothing routes on this.
    pub osm_id: Vec<i64>,
    /// Routing-relevant node tags, so the contraction pass knows not to splice
    /// them away. See `NODE_SIGNALS` / `NODE_BARRIER`.
    pub node_flags: Vec<u8>,

    // --- forward CSR, indexed by node then by edge ---
    /// Length `n_nodes + 1`.
    pub offsets: Vec<u32>,
    pub head: Vec<u32>,
    /// Travel time in milliseconds.
    pub weight: Vec<u32>,
    /// Length in metres, summed along the polyline.
    pub length: Vec<f32>,
    /// Start of this edge's polyline in `geom`. Two directions of one road
    /// share a span, so this is a start/len pair rather than a prefix array.
    pub geom_start: Vec<u32>,
    pub geom_len: Vec<u32>,
    pub flags: Vec<u8>,
    pub name_id: Vec<u32>,

    // --- reverse CSR: at node v, the edges arriving at v ---
    /// Length `n_nodes + 1`.
    pub r_offsets: Vec<u32>,
    /// Tail node of the incoming edge.
    pub r_head: Vec<u32>,
    /// Forward edge id, so weight and geometry are one indirection away
    /// instead of duplicated.
    pub r_edge: Vec<u32>,

    /// The opposite direction of the same road segment, or `NO_TWIN` for a
    /// one-way. Lets road length be counted once per segment, and is what the
    /// geometry arena will share a span on.
    pub twin: Vec<u32>,

    /// Flat `(lon, lat)` polyline points, both endpoints included. Stored once
    /// per road, forward-oriented.
    pub geom: Vec<[f32; 2]>,
    pub names: Vec<String>,

    /// Fastest edge in the graph, metres per millisecond, fixed at build time
    /// and carried in the file header. A* divides by this, so it must be the
    /// value the weights were actually built with rather than something
    /// recomputed on a load path that could later drift.
    pub max_speed_m_per_ms: f64,
}

impl Graph {
    pub fn n_nodes(&self) -> usize {
        self.lon.len()
    }
    pub fn n_edges(&self) -> usize {
        self.head.len()
    }
    /// Edge ids leaving `v`.
    pub fn out_edges(&self, v: u32) -> std::ops::Range<usize> {
        self.offsets[v as usize] as usize..self.offsets[v as usize + 1] as usize
    }
    /// Reverse-CSR slots arriving at `v`.
    pub fn in_edges(&self, v: u32) -> std::ops::Range<usize> {
        self.r_offsets[v as usize] as usize..self.r_offsets[v as usize + 1] as usize
    }
    /// The edge's polyline in travel order.
    pub fn geometry(&self, edge: usize) -> Polyline<'_> {
        let start = self.geom_start[edge] as usize;
        Polyline {
            span: &self.geom[start..start + self.geom_len[edge] as usize],
            reversed: self.flags[edge] & FLAG_GEOM_REVERSED != 0,
            next: 0,
        }
    }
    pub fn name(&self, edge: usize) -> Option<&str> {
        match self.name_id[edge] {
            NO_NAME => None,
            i => Some(&self.names[i as usize]),
        }
    }
    pub fn coord(&self, v: u32) -> (f64, f64) {
        (self.lon[v as usize], self.lat[v as usize])
    }
    /// The `highway` class this edge came from. For inspection and debugging;
    /// nothing routes on it.
    pub fn edge_class(&self, edge: usize) -> &'static str {
        osm_parse::CLASSES[(self.flags[edge] & osm_parse::FLAG_CLASS_MASK) as usize].0
    }
    /// The node an edge leaves. Recovered from the CSR offsets instead of being
    /// stored: the last offset that is still <= the edge id is the node that
    /// owns it, which stays correct even if some node ever has out-degree 0.
    pub fn edge_source(&self, edge: usize) -> u32 {
        (self.offsets.partition_point(|o| *o as usize <= edge) - 1) as u32
    }
    /// A stable id for the undirected road segment behind a directed edge:
    /// the lower of the edge and its twin.
    pub fn pair_of(&self, edge: usize) -> u32 {
        match self.twin[edge] {
            NO_TWIN => edge as u32,
            t => (edge as u32).min(t),
        }
    }
    /// Polyline segments (point pairs) across distinct roads. Splicing joins
    /// polylines end to end without adding or removing a segment, so this is
    /// exactly invariant under contraction - unlike the raw point count, which
    /// drops by one per joint because the joint was stored twice.
    pub fn geom_segments(&self) -> usize {
        (0..self.n_edges())
            .filter(|e| self.flags[*e] & FLAG_GEOM_REVERSED == 0)
            .map(|e| self.geom_len[e] as usize - 1)
            .sum()
    }
    /// Road length in metres, counting a two-way segment once.
    pub fn road_length_m(&self) -> f64 {
        (0..self.n_edges())
            .filter(|e| self.twin[*e] == NO_TWIN || (*e as u32) < self.twin[*e])
            .map(|e| self.length[e] as f64)
            .sum()
    }
}

/// An edge's points in travel order. The reverse direction of a two-way road
/// shares its twin's span and yields it backwards, so this cannot be a slice.
pub struct Polyline<'a> {
    span: &'a [[f32; 2]],
    reversed: bool,
    next: usize,
}

impl Iterator for Polyline<'_> {
    type Item = [f32; 2];
    fn next(&mut self) -> Option<[f32; 2]> {
        let p = self.span.get(if self.reversed {
            self.span.len().checked_sub(self.next + 1)?
        } else {
            self.next
        })?;
        self.next += 1;
        Some(*p)
    }
    fn size_hint(&self) -> (usize, Option<usize>) {
        let n = self.span.len() - self.next;
        (n, Some(n))
    }
}

impl ExactSizeIterator for Polyline<'_> {}
