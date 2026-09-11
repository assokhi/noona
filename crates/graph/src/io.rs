//! Versioned on-disk format: a header plus flat arrays, so loading is
//! `read_exact` into pre-sized vectors rather than deserialising a nested
//! struct graph.

use crate::Graph;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;

const MAGIC: &[u8; 8] = b"CHDGRAPH";
pub const FORMAT_VERSION: u32 = 3;

macro_rules! flat_io {
    ($w:ident, $r:ident, $t:ty, $size:expr) => {
        fn $w<W: Write>(out: &mut W, v: &[$t]) -> io::Result<()> {
            let mut buf = Vec::with_capacity(v.len() * $size);
            for x in v {
                buf.extend_from_slice(&x.to_le_bytes());
            }
            out.write_all(&buf)
        }
        fn $r<R: Read>(inp: &mut R, n: usize) -> io::Result<Vec<$t>> {
            let mut buf = vec![0u8; n * $size];
            inp.read_exact(&mut buf)?;
            Ok(buf
                .chunks_exact($size)
                .map(|c| <$t>::from_le_bytes(c.try_into().unwrap()))
                .collect())
        }
    };
}
flat_io!(w_u32, r_u32, u32, 4);
flat_io!(w_f32, r_f32, f32, 4);
flat_io!(w_f64, r_f64, f64, 8);
flat_io!(w_i64, r_i64, i64, 8);

impl Graph {
    /// `source_hash` identifies the extract this graph was built from.
    pub fn write_to<W: Write>(&self, out: &mut W, source_hash: u64) -> io::Result<()> {
        out.write_all(MAGIC)?;
        w_u32(out, &[FORMAT_VERSION])?;
        out.write_all(&source_hash.to_le_bytes())?;
        w_f64(out, &[self.max_speed_m_per_ms])?;
        w_u32(
            out,
            &[
                self.n_nodes() as u32,
                self.n_edges() as u32,
                self.geom.len() as u32,
                self.names.len() as u32,
            ],
        )?;
        w_f64(out, &self.lon)?;
        w_f64(out, &self.lat)?;
        w_i64(out, &self.osm_id)?;
        out.write_all(&self.node_flags)?;
        w_u32(out, &self.offsets)?;
        w_u32(out, &self.head)?;
        w_u32(out, &self.weight)?;
        w_f32(out, &self.length)?;
        w_u32(out, &self.geom_start)?;
        w_u32(out, &self.geom_len)?;
        out.write_all(&self.flags)?;
        w_u32(out, &self.name_id)?;
        w_u32(out, &self.r_offsets)?;
        w_u32(out, &self.r_head)?;
        w_u32(out, &self.r_edge)?;
        w_u32(out, &self.twin)?;
        let flat: Vec<f32> = self.geom.iter().flat_map(|p| [p[0], p[1]]).collect();
        w_f32(out, &flat)?;
        for s in &self.names {
            w_u32(out, &[s.len() as u32])?;
            out.write_all(s.as_bytes())?;
        }
        Ok(())
    }

    pub fn read_from<R: Read>(inp: &mut R) -> io::Result<(Graph, u64)> {
        let mut magic = [0u8; 8];
        inp.read_exact(&mut magic)?;
        if &magic != MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "not a graph file",
            ));
        }
        let version = r_u32(inp, 1)?[0];
        if version != FORMAT_VERSION {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("graph format v{version}, expected v{FORMAT_VERSION}"),
            ));
        }
        let mut hash = [0u8; 8];
        inp.read_exact(&mut hash)?;
        let source_hash = u64::from_le_bytes(hash);
        let max_speed_m_per_ms = r_f64(inp, 1)?[0];
        let counts = r_u32(inp, 4)?;
        let (n, m, ng, nn) = (
            counts[0] as usize,
            counts[1] as usize,
            counts[2] as usize,
            counts[3] as usize,
        );

        let lon = r_f64(inp, n)?;
        let lat = r_f64(inp, n)?;
        let osm_id = r_i64(inp, n)?;
        let mut node_flags = vec![0u8; n];
        inp.read_exact(&mut node_flags)?;
        let offsets = r_u32(inp, n + 1)?;
        let head = r_u32(inp, m)?;
        let weight = r_u32(inp, m)?;
        let length = r_f32(inp, m)?;
        let geom_start = r_u32(inp, m)?;
        let geom_len = r_u32(inp, m)?;
        let mut flags = vec![0u8; m];
        inp.read_exact(&mut flags)?;
        let name_id = r_u32(inp, m)?;
        let r_offsets = r_u32(inp, n + 1)?;
        let r_head = r_u32(inp, m)?;
        let r_edge = r_u32(inp, m)?;
        let twin = r_u32(inp, m)?;
        let flat = r_f32(inp, ng * 2)?;
        let geom = flat.as_chunks::<2>().0.to_vec();
        let mut names = Vec::with_capacity(nn);
        for _ in 0..nn {
            let len = r_u32(inp, 1)?[0] as usize;
            let mut b = vec![0u8; len];
            inp.read_exact(&mut b)?;
            names.push(
                String::from_utf8(b).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?,
            );
        }

        // The header value is what A* divides by. If it no longer bounds every
        // edge the heuristic is inadmissible and routes go quietly non-optimal.
        if let Some(e) = (0..m).find(|e| length[*e] as f64 / weight[*e] as f64 > max_speed_m_per_ms)
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "edge {e} runs at {:.3} m/ms, above the header maximum {max_speed_m_per_ms:.3}",
                    length[e] as f64 / weight[e] as f64
                ),
            ));
        }

        Ok((
            Graph {
                lon,
                lat,
                osm_id,
                node_flags,
                twin,
                offsets,
                head,
                weight,
                length,
                geom_start,
                geom_len,
                flags,
                name_id,
                r_offsets,
                r_head,
                r_edge,
                geom,
                names,
                max_speed_m_per_ms,
            },
            source_hash,
        ))
    }

    pub fn save(&self, path: &Path, source_hash: u64) -> io::Result<()> {
        let mut w = BufWriter::new(File::create(path)?);
        self.write_to(&mut w, source_hash)?;
        w.flush()
    }

    pub fn load(path: &Path) -> io::Result<(Graph, u64)> {
        Graph::read_from(&mut BufReader::new(File::open(path)?))
    }

    pub fn to_bytes(&self, source_hash: u64) -> Vec<u8> {
        let mut v = Vec::new();
        self.write_to(&mut v, source_hash).expect("in-memory write");
        v
    }
}

/// FNV-1a over a file, so the graph header can name the extract it came from.
pub fn file_hash(path: &Path) -> io::Result<u64> {
    let mut f = BufReader::new(File::open(path)?);
    let mut buf = vec![0u8; 1 << 20];
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    loop {
        let n = f.read(&mut buf)?;
        if n == 0 {
            return Ok(h);
        }
        for b in &buf[..n] {
            h = (h ^ *b as u64).wrapping_mul(0x1000_0000_01b3);
        }
    }
}
