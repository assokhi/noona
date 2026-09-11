import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import * as pmtiles from "pmtiles";

import { ApiError, BBOX, fromGeolocation, isochrone, lonLat, route } from "./api";
import type { Alg, LonLat, RouteResponse } from "./api";
import { ConstantVelocity } from "./kalman";

// Must be registered before the map is constructed, or the style's source
// cannot resolve pmtiles:// URLs.
maplibregl.addProtocol("pmtiles", new pmtiles.Protocol().tile);

const map = new maplibregl.Map({
  container: "map",
  style: {
    version: 8,
    glyphs: "https://fonts.openmaptiles.org/{fontstack}/{range}.pbf",
    sources: {
      base: {
        type: "vector",
        url: `pmtiles://${location.origin}/chandigarh.pmtiles`,
        attribution: "&copy; OpenStreetMap contributors, &copy; Protomaps",
      },
    },
    layers: baseLayers(),
  },
  center: [(BBOX.west + BBOX.east) / 2, (BBOX.south + BBOX.north) / 2],
  zoom: 12.5,
  maxBounds: [
    [BBOX.west - 0.15, BBOX.south - 0.15],
    [BBOX.east + 0.15, BBOX.north + 0.15],
  ],
});

/**
 * A deliberately plain basemap. The route has to be the loudest thing on the
 * screen; a full-colour basemap competes with it.
 */
function baseLayers(): maplibregl.LayerSpecification[] {
  const l = (
    id: string,
    kind: string,
    paint: Record<string, unknown>,
    filter?: unknown,
  ): maplibregl.LayerSpecification =>
    ({
      id,
      type: kind,
      source: "base",
      "source-layer": id.split(":")[0],
      paint,
      ...(filter ? { filter } : {}),
    }) as maplibregl.LayerSpecification;
  return [
    { id: "bg", type: "background", paint: { "background-color": "#12151a" } },
    l("earth", "fill", { "fill-color": "#191d24" }),
    l("landuse", "fill", { "fill-color": "#1c222a" }),
    l("water", "fill", { "fill-color": "#0e2436" }),
    l("roads:casing", "line", {
      "line-color": "#2b323c",
      "line-width": ["interpolate", ["linear"], ["zoom"], 10, 1, 16, 8],
    }),
    l("roads", "line", {
      "line-color": "#39424f",
      "line-width": ["interpolate", ["linear"], ["zoom"], 10, 0.4, 16, 4],
    }),
    l("buildings", "fill", { "fill-color": "#20262f" }),
  ];
}

// --- state -----------------------------------------------------------------

let origin: LonLat | null = null;
let destination: LonLat | null = null;
let alg: Alg = "bidir";

const $ = (id: string) => document.getElementById(id)!;
const status = (msg: string, bad = false) => {
  const el = $("status");
  el.textContent = msg;
  el.className = bad ? "bad" : "";
};

// --- map layers ------------------------------------------------------------

function emptyFC(): GeoJSON.FeatureCollection {
  return { type: "FeatureCollection", features: [] };
}

map.on("load", () => {
  map.addSource("route", { type: "geojson", data: emptyFC() });
  map.addSource("snaps", { type: "geojson", data: emptyFC() });
  map.addSource("pins", { type: "geojson", data: emptyFC() });
  map.addSource("me", { type: "geojson", data: emptyFC() });
  map.addSource("me-raw", { type: "geojson", data: emptyFC() });
  map.addSource("iso", { type: "geojson", data: emptyFC() });

  // Under everything else: an isochrone is context, not the subject.
  map.addLayer({
    id: "iso-fill",
    type: "fill",
    source: "iso",
    paint: {
      "fill-color": [
        "interpolate", ["linear"], ["get", "minutes"],
        5, "#3ddc97", 10, "#ffd166", 20, "#ff6b6b",
      ],
      "fill-opacity": 0.16,
    },
  });
  map.addLayer({
    id: "iso-line",
    type: "line",
    source: "iso",
    paint: {
      "line-color": [
        "interpolate", ["linear"], ["get", "minutes"],
        5, "#3ddc97", 10, "#ffd166", 20, "#ff6b6b",
      ],
      "line-width": 1.5,
      "line-opacity": 0.8,
    },
  });

  // Two layers, wide dark casing under a narrower bright line. A single line
  // disappears against the basemap's own road casings.
  map.addLayer({
    id: "route-casing",
    type: "line",
    source: "route",
    paint: { "line-color": "#07131f", "line-width": 8, "line-opacity": 0.9 },
    layout: { "line-cap": "round", "line-join": "round" },
  });
  map.addLayer({
    id: "route-line",
    type: "line",
    source: "route",
    paint: { "line-color": "#3ddc97", "line-width": 5 },
    layout: { "line-cap": "round", "line-join": "round" },
  });

  // Silent snapping is how you fail to notice a click that landed 300 m off on
  // the wrong carriageway, so the leap from click to road is always drawn.
  map.addLayer({
    id: "snap-lines",
    type: "line",
    source: "snaps",
    paint: { "line-color": "#ffb347", "line-width": 1.5, "line-dasharray": [2, 2] },
  });
  map.addLayer({
    id: "snap-dots",
    type: "circle",
    source: "snaps",
    filter: ["==", ["geometry-type"], "Point"],
    paint: { "circle-radius": 3.5, "circle-color": "#ffb347" },
  });
  map.addLayer({
    id: "pins",
    type: "circle",
    source: "pins",
    paint: {
      "circle-radius": 7,
      "circle-color": ["case", ["==", ["get", "role"], "origin"], "#3ddc97", "#ff6b6b"],
      "circle-stroke-width": 2,
      "circle-stroke-color": "#07131f",
    },
  });

  // Accuracy circle first so the dot sits on top of it.
  map.addLayer({
    id: "me-accuracy",
    type: "circle",
    source: "me",
    paint: {
      "circle-radius": ["get", "radiusPx"],
      "circle-color": "#4aa8ff",
      "circle-opacity": 0.15,
      "circle-stroke-color": "#4aa8ff",
      "circle-stroke-opacity": 0.4,
      "circle-stroke-width": 1,
    },
  });
  map.addLayer({
    id: "me-raw",
    type: "circle",
    source: "me-raw",
    paint: { "circle-radius": 4, "circle-color": "#ffffff", "circle-opacity": 0.35 },
    layout: { visibility: "none" },
  });
  map.addLayer({
    id: "me-dot",
    type: "circle",
    source: "me",
    paint: {
      "circle-radius": 6,
      "circle-color": "#4aa8ff",
      "circle-stroke-width": 2,
      "circle-stroke-color": "#ffffff",
    },
  });
  map.addLayer({
    id: "me-heading",
    type: "symbol",
    source: "me",
    filter: ["has", "heading"],
    layout: {
      "icon-image": "heading-cone",
      "icon-rotate": ["get", "heading"],
      "icon-rotation-alignment": "map",
      "icon-allow-overlap": true,
    },
  });
  map.addImage("heading-cone", headingCone(), { pixelRatio: 2 });
  status("click the map to set an origin");
});

/** A small triangular cone, drawn once and rotated by `coords.heading`. */
function headingCone(): ImageData {
  const size = 48;
  const c = document.createElement("canvas");
  c.width = c.height = size;
  const ctx = c.getContext("2d")!;
  const grad = ctx.createLinearGradient(size / 2, size / 2, size / 2, 0);
  grad.addColorStop(0, "rgba(74,168,255,0.65)");
  grad.addColorStop(1, "rgba(74,168,255,0)");
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.moveTo(size / 2, size / 2);
  ctx.lineTo(size / 2 - 11, 4);
  ctx.lineTo(size / 2 + 11, 4);
  ctx.closePath();
  ctx.fill();
  return ctx.getImageData(0, 0, size, size);
}

// --- routing ---------------------------------------------------------------

map.on("click", (e) => {
  const at = lonLat(e.lngLat.lng, e.lngLat.lat);
  if (!origin || destination) {
    origin = at;
    destination = null;
    setSource("route", emptyFC());
    setSource("snaps", emptyFC());
    renderDebug(null);
    status("origin set, click a destination");
  } else {
    destination = at;
    status("routing...");
  }
  drawPins();
  if (origin && destination) void request();
});

function drawPins() {
  const features: GeoJSON.Feature[] = [];
  if (origin) features.push(point(origin, { role: "origin" }));
  if (destination) features.push(point(destination, { role: "destination" }));
  setSource("pins", { type: "FeatureCollection", features });
}

async function request() {
  if (!origin || !destination) return;
  const started = performance.now();
  try {
    const r = await route(origin, destination, alg);
    const wall = performance.now() - started;
    setSource("route", {
      type: "FeatureCollection",
      features: [{ type: "Feature", properties: {}, geometry: r.geometry }],
    });
    setSource("snaps", {
      type: "FeatureCollection",
      features: [
        snapLeap(origin, r.debug.from_snap.snapped),
        snapLeap(destination, r.debug.to_snap.snapped),
        point(r.debug.from_snap.snapped, {}),
        point(r.debug.to_snap.snapped, {}),
      ],
    });
    renderDebug(r, wall);
    status(
      `${(r.distance_m / 1000).toFixed(2)} km, ${fmtDuration(r.duration_s)}`,
    );
  } catch (err) {
    renderDebug(null);
    setSource("route", emptyFC());
    if (err instanceof ApiError) {
      status(`${err.code}: ${err.message}`, true);
    } else {
      status(String(err), true);
    }
  }
}

function snapLeap(from: LonLat, to: LonLat): GeoJSON.Feature {
  return {
    type: "Feature",
    properties: {},
    geometry: { type: "LineString", coordinates: [from, to] },
  };
}
function point(at: LonLat, properties: Record<string, unknown>): GeoJSON.Feature {
  return { type: "Feature", properties, geometry: { type: "Point", coordinates: at } };
}
function setSource(id: string, data: GeoJSON.FeatureCollection) {
  (map.getSource(id) as maplibregl.GeoJSONSource | undefined)?.setData(data);
}
function fmtDuration(s: number): string {
  const m = Math.round(s / 60);
  return m >= 60 ? `${Math.floor(m / 60)}h ${m % 60}m` : `${m} min`;
}

/** The debug block, rendered every response. This is the benchmark table made
 * operable: switch algorithm and watch nodes settled move. */
function renderDebug(r: RouteResponse | null, wallMs?: number) {
  const el = $("debug");
  if (!r) {
    el.innerHTML = '<div class="dim">no route yet</div>';
    return;
  }
  const d = r.debug;
  const row = (k: string, v: string) => `<div><span>${k}</span><b>${v}</b></div>`;
  el.innerHTML =
    row("algorithm", d.alg) +
    row("nodes settled", d.nodes_settled.toLocaleString()) +
    row("edges relaxed", d.edges_relaxed.toLocaleString()) +
    row("search", `${d.search_ms.toFixed(3)} ms`) +
    row("snap", `${d.snap_ms.toFixed(3)} ms`) +
    row("pool wait", `${d.pool_wait_ms.toFixed(3)} ms`) +
    (wallMs !== undefined ? row("round trip", `${wallMs.toFixed(1)} ms`) : "") +
    row("origin snap", `${d.from_snap.distance_m.toFixed(1)} m`) +
    row("dest snap", `${d.to_snap.distance_m.toFixed(1)} m`) +
    row("on one edge", String(d.same_edge)) +
    row("origin road", d.from_snap.way_name ?? "unnamed") +
    row("dest road", d.to_snap.way_name ?? "unnamed");
}

for (const el of document.querySelectorAll<HTMLButtonElement>("[data-alg]")) {
  el.addEventListener("click", () => {
    alg = el.dataset.alg as Alg;
    for (const b of document.querySelectorAll("[data-alg]")) b.classList.remove("on");
    el.classList.add("on");
    // Re-issue the same query so the two are directly comparable.
    if (origin && destination) void request();
  });
}
$("iso").addEventListener("click", async () => {
  if (!origin) {
    status("set an origin first, then ask for an isochrone", true);
    return;
  }
  status("computing isochrone...");
  try {
    const r = await isochrone(origin, [5, 10, 15]);
    setSource("iso", r as GeoJSON.FeatureCollection);
    const bands = r.features.map((f) => `${f.properties.minutes} min`).join(", ");
    status(`isochrone: ${bands} from ${r.debug.nodes_reached} nodes reached`);
  } catch (err) {
    status(err instanceof ApiError ? `${err.code}: ${err.message}` : String(err), true);
  }
});

$("clear").addEventListener("click", () => {
  setSource("iso", emptyFC());
  origin = destination = null;
  for (const s of ["route", "snaps", "pins"]) setSource(s, emptyFC());
  renderDebug(null);
  status("click the map to set an origin");
});

// --- live location ---------------------------------------------------------

const filter = new ConstantVelocity();
let watchId: number | null = null;
let showRaw = false;
let lastFix: {
  filtered: LonLat;
  raw: LonLat;
  accuracy: number;
  heading: number | null;
} | null = null;

function drawMe() {
  if (!lastFix) return;
  const props: Record<string, unknown> = {
    radiusPx: metresToPixels(lastFix.accuracy, lastFix.filtered[1]),
  };
  if (lastFix.heading !== null) props.heading = lastFix.heading;
  setSource("me", {
    type: "FeatureCollection",
    features: [point(lastFix.filtered, props)],
  });
  setSource("me-raw", { type: "FeatureCollection", features: [point(lastFix.raw, {})] });
}

$("locate").addEventListener("click", () => {
  if (watchId !== null) {
    navigator.geolocation.clearWatch(watchId);
    watchId = null;
    filter.reset();
    lastFix = null;
    setSource("me", emptyFC());
    setSource("me-raw", emptyFC());
    $("locate").textContent = "track me";
    return;
  }
  if (!navigator.geolocation) {
    status("this browser has no Geolocation API", true);
    return;
  }
  if (!window.isSecureContext) {
    // Worth saying plainly: a LAN IP silently fails this check, and testing on
    // a phone against a dev box needs a tunnel or a local certificate.
    status("geolocation needs https or localhost - a LAN IP will not work", true);
    return;
  }
  $("locate").textContent = "stop tracking";
  watchId = navigator.geolocation.watchPosition(
    (pos) => {
      const raw = fromGeolocation(pos.coords);
      const filtered = filter.update({
        at: raw,
        accuracyM: pos.coords.accuracy,
        timestamp: pos.timestamp,
      });
      lastFix = {
        filtered,
        raw,
        accuracy: pos.coords.accuracy,
        heading:
          pos.coords.heading !== null && !Number.isNaN(pos.coords.heading)
            ? pos.coords.heading
            : null,
      };
      drawMe();
      $("speed").textContent = `${(filter.speed() * 3.6).toFixed(0)} km/h · ±${pos.coords.accuracy.toFixed(0)} m`;
    },
    (err) => status(`geolocation: ${err.message}`, true),
    { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 },
  );
});

$("raw").addEventListener("click", () => {
  showRaw = !showRaw;
  map.setLayoutProperty("me-raw", "visibility", showRaw ? "visible" : "none");
  $("raw").classList.toggle("on", showRaw);
});

/** Accuracy is metres; circle-radius is pixels, and the conversion depends on
 * both zoom and latitude. */
function metresToPixels(m: number, lat: number): number {
  const metresPerPixel =
    (156543.03392 * Math.cos((lat * Math.PI) / 180)) / Math.pow(2, map.getZoom());
  return Math.min(m / metresPerPixel, 200);
}
// The accuracy circle is drawn in pixels but means metres, so it has to be
// recomputed whenever the scale changes or it lies at every zoom but one.
map.on("zoom", drawMe);
