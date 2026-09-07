/**
 * Constant-velocity Kalman filter over [lon, lat, vlon, vlat].
 *
 * Raw browser fixes in a dense sector grid jump between parallel V4 and V5
 * roads and look broken. This smooths them without pretending to know the road
 * network - that is Phase 6's map matching, and the visible difference between
 * the raw dot and the filtered one is the argument for building it.
 *
 * Working in degrees would make the process noise anisotropic, because a degree
 * of longitude is shorter than a degree of latitude. State is therefore metres
 * relative to a fixed origin, converted at the boundary.
 */

import type { LonLat } from "./api";

/** Metres per degree at a given latitude. Good to well under a metre locally. */
function scale(lat: number): { x: number; y: number } {
  const rad = (lat * Math.PI) / 180;
  return { x: 111320 * Math.cos(rad), y: 110574 };
}

export interface Fix {
  at: LonLat;
  /** One-sigma horizontal accuracy in metres, from `coords.accuracy`. */
  accuracyM: number;
  /** Milliseconds. */
  timestamp: number;
}

export class ConstantVelocity {
  private origin: LonLat | null = null;
  private s = { x: 0, y: 0, vx: 0, vy: 0 };
  /** Covariance, kept diagonal-ish: position and velocity blocks. */
  private p = { xx: 1e6, xv: 0, vv: 1e6 };
  private lastMs = 0;

  /**
   * How much unmodelled acceleration to expect, m/s^2. A car turning and
   * braking in traffic is a couple of m/s^2; too low and the filter lags
   * through corners, too high and it stops filtering.
   */
  constructor(private readonly accelNoise = 2.0) {}

  reset(): void {
    this.origin = null;
    this.p = { xx: 1e6, xv: 0, vv: 1e6 };
  }

  /** Feeds one fix and returns the filtered position. */
  update(fix: Fix): LonLat {
    if (!this.origin) {
      this.origin = fix.at;
      this.s = { x: 0, y: 0, vx: 0, vy: 0 };
      this.lastMs = fix.timestamp;
      const r = Math.max(fix.accuracyM, 1) ** 2;
      this.p = { xx: r, xv: 0, vv: 100 };
      return fix.at;
    }

    const k = scale(this.origin[1]);
    const zx = (fix.at[0] - this.origin[0]) * k.x;
    const zy = (fix.at[1] - this.origin[1]) * k.y;

    // Clamp dt: a backgrounded tab produces enormous gaps, and predicting
    // forward across one just launches the state into space.
    const dt = Math.min(Math.max((fix.timestamp - this.lastMs) / 1000, 0), 5);
    this.lastMs = fix.timestamp;

    // Predict.
    this.s.x += this.s.vx * dt;
    this.s.y += this.s.vy * dt;
    const q = this.accelNoise ** 2;
    const { xx, xv, vv } = this.p;
    this.p = {
      xx: xx + 2 * xv * dt + vv * dt * dt + (q * dt ** 4) / 4,
      xv: xv + vv * dt + (q * dt ** 3) / 2,
      vv: vv + q * dt * dt,
    };

    // Update. Both axes share the covariance because accuracy is a single
    // radius; that is what the browser gives us and inventing per-axis error
    // would be fiction.
    const r = Math.max(fix.accuracyM, 1) ** 2;
    const s = this.p.xx + r;
    const kx = this.p.xx / s;
    const kv = this.p.xv / s;

    this.s.x += kx * (zx - this.s.x);
    this.s.y += kx * (zy - this.s.y);
    this.s.vx += kv * (zx - this.s.x);
    this.s.vy += kv * (zy - this.s.y);

    this.p = {
      xx: this.p.xx * (1 - kx),
      xv: this.p.xv * (1 - kx),
      vv: this.p.vv - kv * this.p.xv,
    };

    return [this.origin[0] + this.s.x / k.x, this.origin[1] + this.s.y / k.y];
  }

  /** Current speed estimate in m/s, for display. */
  speed(): number {
    return Math.hypot(this.s.vx, this.s.vy);
  }
}
