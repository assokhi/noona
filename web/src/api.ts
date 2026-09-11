/**
 * The routing API, and the one place lat/lon order is converted.
 *
 * The engine is lon,lat everywhere, matching GeoJSON. The browser Geolocation
 * API hands back lat,lon. Chandigarh sits at 76.7, 30.7 - both numbers are
 * plausible as either, so a swap does not throw, it silently puts the route in
 * the Arabian Sea. Everything crosses through `fromGeolocation` and every
 * coordinate is range-asserted on the way in.
 */

export type LonLat = [number, number];

export const BBOX = { west: 76.65, south: 30.65, east: 76.87, north: 30.8 };

/** Range-checks a lon,lat pair and returns it. Throws rather than guessing. */
export function lonLat(lon: number, lat: number): LonLat {
  if (!Number.isFinite(lon) || !Number.isFinite(lat)) {
    throw new Error(`coordinate is not finite: ${lon},${lat}`);
  }
  if (lon < -180 || lon > 180 || lat < -90 || lat > 90) {
    throw new Error(`coordinate out of range: ${lon},${lat} - order swapped?`);
  }
  // Everything this app handles is near Chandigarh. A latitude that arrived in
  // the longitude slot is inside the global range but nowhere near here, and
  // this is the assertion that catches it.
  if (lat > 60 && lon < 60) {
    throw new Error(`suspicious coordinate ${lon},${lat} - looks like lat,lon`);
  }
  return [lon, lat];
}

/** The single conversion point for browser positions, which are lat,lon. */
export function fromGeolocation(c: GeolocationCoordinates): LonLat {
  return lonLat(c.longitude, c.latitude);
}

export type Alg = "dijkstra" | "astar" | "bidir" | "alt" | "ch";

export interface SnapDebug {
  edge_id: number;
  distance_m: number;
  snapped: LonLat;
  way_name: string | null;
}

export interface RouteDebug {
  alg: Alg;
  nodes_settled: number;
  edges_relaxed: number;
  search_ms: number;
  snap_ms: number;
  pool_wait_ms: number;
  same_edge: boolean;
  from_snap: SnapDebug;
  to_snap: SnapDebug;
}

export interface RouteResponse {
  distance_m: number;
  duration_s: number;
  geometry: { type: "LineString"; coordinates: LonLat[] };
  debug: RouteDebug;
}

export class ApiError extends Error {
  constructor(
    readonly code: string,
    message: string,
    readonly status: number,
  ) {
    super(message);
  }
}

async function get<T>(path: string): Promise<T> {
  const res = await fetch(path);
  if (!res.ok) {
    let code = "HTTP_ERROR";
    let message = `${res.status} ${res.statusText}`;
    try {
      const body = await res.json();
      code = body?.error?.code ?? code;
      message = body?.error?.message ?? message;
    } catch {
      // A non-JSON error body is still an error; keep the status text.
    }
    throw new ApiError(code, message, res.status);
  }
  return (await res.json()) as T;
}

export function route(from: LonLat, to: LonLat, alg: Alg): Promise<RouteResponse> {
  const q = new URLSearchParams({
    from: `${from[0]},${from[1]}`,
    to: `${to[0]},${to[1]}`,
    alg,
    geometry: "geojson",
  });
  return get<RouteResponse>(`/v1/route?${q}`);
}

export interface NearestResponse {
  edge_id: number;
  snapped: LonLat;
  distance_m: number;
  way_name: string | null;
}

export function nearest(at: LonLat): Promise<NearestResponse> {
  const q = new URLSearchParams({ lon: String(at[0]), lat: String(at[1]) });
  return get<NearestResponse>(`/v1/nearest?${q}`);
}
