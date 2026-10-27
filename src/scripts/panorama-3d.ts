// The view from the hilltop in 3D: you stand 1.7 m above the summit and look around.
//
// The terrain is the same Copernicus 30 m model the drawing comes from, resampled by
// scripts/terrain.mjs onto a polar grid centred on the summit — one column every 0.15° of
// bearing, rings that grow 2% each step out to 160 km — and packed into a PNG. A polar grid
// is the natural mesh for standing at one point: every bearing gets the same angular detail,
// so Aconcagua at 65 km is drawn as finely as the hill at your feet. Each far cell keeps the
// highest ground inside it, which is what makes a skyline.
//
// It is rendered with three.js: the sun where it really is for the date and time you pick
// (NOAA solar position), cast shadows from a sun-aligned depth map, snow above a seasonal
// snowline, haze that thins with altitude, and Preetham's analytic sky. Labels are HTML,
// projected each frame and kept from overlapping.

import * as THREE from "three";
import { Sky } from "three/examples/jsm/objects/Sky.js";

export interface PanoramaView3D {
  /** Turn to a summit (by id) and zoom in on it. */
  lookAt(id: string): void;
  /** Turn to a bearing, keeping the zoom. */
  turnTo(azimuth: number): void;
  heading(): number;
  pause(): void;
  resume(): void;
  destroy(): void;
}

interface PeakData {
  id: string;
  name: string;
  short: string;
  ele: number;
  model: number;
  azimuth: number;
  distance: number;
  angle: number;
  status: "visible" | "borderline" | "hidden";
  named: boolean;
  url: string | null;
}

interface ViewData {
  viewpoint: { lat: number; lon: number; elevation: number; eye: number };
  mesh: { cols: number; rings: number; azStep: number; r0: number; ratio: number; effectiveRadius: number; file: string };
  peaks: PeakData[];
  labels: Record<string, string>;
}

const RAD = Math.PI / 180;
const SEASONS = {
  // Mid-season dates; the snowline is a typical one for the central Andes in that season,
  // not a measurement — late storms can drop it to the valley, a dry summer lift it above 4,500 m.
  summer: { month: 0, day: 15, utcOffset: -3, snowline: 4400 },
  autumn: { month: 3, day: 15, utcOffset: -4, snowline: 3500 },
  winter: { month: 6, day: 15, utcOffset: -4, snowline: 2300 },
  spring: { month: 9, day: 15, utcOffset: -3, snowline: 2900 },
} as const;
type Season = keyof typeof SEASONS;

/** Sun position (NOAA algorithm): azimuth clockwise from north and elevation, in degrees. */
function sunPosition(date: Date, lat: number, lon: number) {
  const jd = date.getTime() / 86_400_000 + 2_440_587.5;
  const T = (jd - 2_451_545) / 36_525;
  const L0 = (280.46646 + T * (36000.76983 + T * 0.0003032)) % 360;
  const M = 357.52911 + T * (35999.05029 - 0.0001537 * T);
  const e = 0.016708634 - T * (0.000042037 + 0.0000001267 * T);
  const C =
    Math.sin(M * RAD) * (1.914602 - T * (0.004817 + 0.000014 * T)) +
    Math.sin(2 * M * RAD) * (0.019993 - 0.000101 * T) +
    Math.sin(3 * M * RAD) * 0.000289;
  const omega = 125.04 - 1934.136 * T;
  const lambda = L0 + C - 0.00569 - 0.00478 * Math.sin(omega * RAD);
  const eps0 = 23 + (26 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60) / 60;
  const eps = eps0 + 0.00256 * Math.cos(omega * RAD);
  const decl = Math.asin(Math.sin(eps * RAD) * Math.sin(lambda * RAD));
  const y = Math.tan((eps * RAD) / 2) ** 2;
  const eqTime =
    (4 / RAD) *
    (y * Math.sin(2 * L0 * RAD) -
      2 * e * Math.sin(M * RAD) +
      4 * e * y * Math.sin(M * RAD) * Math.cos(2 * L0 * RAD) -
      0.5 * y * y * Math.sin(4 * L0 * RAD) -
      1.25 * e * e * Math.sin(2 * M * RAD));
  const minutes = date.getUTCHours() * 60 + date.getUTCMinutes() + date.getUTCSeconds() / 60;
  const solarTime = (((minutes + eqTime + 4 * lon) % 1440) + 1440) % 1440;
  const ha = (solarTime / 4 - 180) * RAD;
  const φ = lat * RAD;
  const cosZenith = Math.min(1, Math.max(-1, Math.sin(φ) * Math.sin(decl) + Math.cos(φ) * Math.cos(decl) * Math.cos(ha)));
  const elevation = 90 - Math.acos(cosZenith) / RAD;
  const azimuth = (Math.atan2(Math.sin(ha), Math.cos(ha) * Math.sin(φ) - Math.tan(decl) * Math.cos(φ)) / RAD + 180 + 360) % 360;
  return { azimuth, elevation };
}

/** Direction (unit vector) in the local frame: x east, y up, −z north. */
function direction(azimuth: number, elevation: number) {
  const ce = Math.cos(elevation * RAD);
  return new THREE.Vector3(ce * Math.sin(azimuth * RAD), Math.sin(elevation * RAD), -ce * Math.cos(azimuth * RAD));
}

const EARTH = 6_371_008.8;
const CIRCUMFERENCE = 40_075_016.686;
/** Web Mercator, 0–1 across the world: x eastward from 180°W, y southward from the top. */
const worldX = (lon: number) => (lon + 180) / 360;
const worldY = (lat: number) => (1 - Math.log(Math.tan(Math.PI / 4 + (lat * RAD) / 2)) / Math.PI) / 2;

/**
 * The satellite mosaic: Esri World Imagery at five zoom levels, each a canvas of whole tiles
 * centred on the hill — 1.2 m per pixel for the hilltop itself, down to 128 m per pixel out
 * at 80 km, past which nothing can be seen from the summit anyway. Far levels load first,
 * because the mountains are what you came for.
 */
function createImagery(viewpoint: { lat: number; lon: number }, anisotropy: number, onTile: () => void) {
  const LEVELS = [
    { zoom: 17, radius: 450 },
    { zoom: 15, radius: 3_000 },
    { zoom: 13, radius: 10_000 },
    { zoom: 11, radius: 35_000 },
    { zoom: 10, radius: 80_000 },
  ];
  const x0 = worldX(viewpoint.lon);
  const y0 = worldY(viewpoint.lat);
  const levels = LEVELS.map(({ zoom, radius }) => {
    const n = 2 ** zoom;
    const dLat = radius / 111_195;
    const dLon = radius / (111_195 * Math.cos(viewpoint.lat * RAD));
    const tx0 = Math.floor(worldX(viewpoint.lon - dLon) * n);
    const tx1 = Math.floor(worldX(viewpoint.lon + dLon) * n);
    const ty0 = Math.floor(worldY(viewpoint.lat + dLat) * n);
    const ty1 = Math.floor(worldY(viewpoint.lat - dLat) * n);
    const canvas = document.createElement("canvas");
    canvas.width = (tx1 - tx0 + 1) * 256;
    canvas.height = (ty1 - ty0 + 1) * 256;
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.flipY = false;
    texture.anisotropy = Math.min(8, anisotropy);
    texture.minFilter = THREE.LinearMipmapLinearFilter;
    const rect = new THREE.Vector4(
      (tx0 / n - x0) * CIRCUMFERENCE,
      (ty0 / n - y0) * CIRCUMFERENCE,
      ((tx1 - tx0 + 1) / n) * CIRCUMFERENCE,
      ((ty1 - ty0 + 1) / n) * CIRCUMFERENCE,
    );
    return { zoom, tx0, tx1, ty0, ty1, canvas, context: canvas.getContext("2d")!, texture, rect, pending: false };
  });

  let started = false;
  let disposed = false;
  let flush = 0;
  const schedule = (level: (typeof levels)[number]) => {
    level.pending = true;
    if (flush) return;
    flush = window.setTimeout(() => {
      flush = 0;
      for (const l of levels) if (l.pending) (l.texture.needsUpdate = true), (l.pending = false);
      onTile();
    }, 250);
  };

  function start() {
    if (started) return;
    started = true;
    const queue = levels
      .slice()
      .reverse()
      .flatMap((level) => {
        const tiles = [];
        for (let ty = level.ty0; ty <= level.ty1; ty++) for (let tx = level.tx0; tx <= level.tx1; tx++) tiles.push({ level, tx, ty });
        return tiles;
      });
    let next = 0;
    const worker = async () => {
      while (next < queue.length && !disposed) {
        const { level, tx, ty } = queue[next++];
        const host = (tx + ty) % 2 ? "server" : "services";
        const image = new Image();
        image.crossOrigin = "anonymous";
        image.src = `https://${host}.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${level.zoom}/${ty}/${tx}`;
        try {
          await image.decode();
          if (disposed) return;
          level.context.drawImage(image, (tx - level.tx0) * 256, (ty - level.ty0) * 256);
          schedule(level);
        } catch {
          // A missing tile just leaves that patch to the coarser level below it.
        }
      }
    };
    for (let i = 0; i < 8; i++) worker();
  }

  return {
    levels,
    start,
    dispose() {
      disposed = true;
      clearTimeout(flush);
      for (const level of levels) level.texture.dispose();
    },
  };
}

const smooth = (a: number, b: number, x: number) => {
  const t = Math.min(1, Math.max(0, (x - a) / (b - a)));
  return t * t * (3 - 2 * t);
};

// ------------------------------------------------------------------------------ shaders

const TERRAIN_VERTEX = /* glsl */ `
  attribute float elevation;
  attribute vec2 mercator;
  varying vec3 vWorld;
  varying vec3 vNormal;
  varying float vElevation;
  varying vec2 vMercator;
  #include <common>
  #include <logdepthbuf_pars_vertex>
  void main() {
    vWorld = position;
    vNormal = normal;
    vElevation = elevation;
    vMercator = mercator;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
    #include <logdepthbuf_vertex>
  }
`;

const TERRAIN_FRAGMENT = /* glsl */ `
  uniform vec3 sunDirection;
  uniform vec3 sunColor;
  uniform vec3 skyAmbient;
  uniform vec3 groundAmbient;
  uniform vec3 hazeColor;
  uniform vec3 hazeSunColor;
  uniform float eyeElevation;
  uniform float snowline;
  uniform sampler2D shadowMap;
  uniform mat4 shadowMatrix;
  uniform float shadowTexel;
  uniform float shadowsOn;
  uniform float imageryOn;
  uniform sampler2D imagery0;
  uniform sampler2D imagery1;
  uniform sampler2D imagery2;
  uniform sampler2D imagery3;
  uniform sampler2D imagery4;
  uniform vec4 imageryRect0;
  uniform vec4 imageryRect1;
  uniform vec4 imageryRect2;
  uniform vec4 imageryRect3;
  uniform vec4 imageryRect4;
  varying vec3 vWorld;
  varying vec3 vNormal;
  varying float vElevation;
  varying vec2 vMercator;
  #include <common>
  #include <logdepthbuf_pars_fragment>

  // One level of the satellite mosaic: colour, and how much of it to use here — none outside
  // its square, fading in near its edge, and none where a tile hasn't arrived (alpha 0).
  vec4 imageryLevel(sampler2D tex, vec4 rect) {
    vec2 uv = (vMercator - rect.xy) / rect.zw;
    vec4 c = texture2D(tex, clamp(uv, 0.0, 1.0));
    vec2 edge = min(uv, 1.0 - uv);
    float inside = smoothstep(0.0, 0.05, min(edge.x, edge.y));
    return vec4(c.rgb, c.a * inside);
  }

  float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
  float noise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1, 0)), u.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), u.x), u.y);
  }
  float fbm(vec2 p) { return 0.5 * noise(p) + 0.25 * noise(p * 2.03) + 0.125 * noise(p * 4.01); }

  float sunShadow(vec3 world, vec3 n) {
    if (shadowsOn < 0.5) return 1.0;
    // Normal offset: push the lookup out of the surface so slopes don't shadow themselves.
    vec4 p = shadowMatrix * vec4(world + n * shadowTexel * 2.5, 1.0);
    vec3 c = p.xyz / p.w * 0.5 + 0.5;
    if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0 || c.z > 1.0) return 1.0;
    float lit = 0.0;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        float depth = texture2D(shadowMap, c.xy + vec2(float(i), float(j)) / 4096.0).r;
        lit += c.z - 0.0004 > depth ? 0.0 : 1.0;
      }
    }
    return lit / 9.0;
  }

  void main() {
    #include <logdepthbuf_fragment>
    vec3 n = normalize(vNormal);
    float h = vElevation;
    float steep = 1.0 - n.y;
    float variation = fbm(vWorld.xz / 1800.0);
    float fine = fbm(vWorld.xz / 260.0);

    // Ground colour by altitude, linear RGB: irrigated valley floor, dry matorral hills,
    // the reddish volcanic rock of the high Andes, grey rock near the tops.
    vec3 valley = mix(vec3(0.075, 0.105, 0.038), vec3(0.16, 0.14, 0.075), variation);
    vec3 scrub = mix(vec3(0.20, 0.16, 0.095), vec3(0.15, 0.13, 0.08), fine);
    vec3 rock = mix(vec3(0.21, 0.15, 0.11), vec3(0.17, 0.155, 0.15), variation);
    vec3 high = mix(vec3(0.26, 0.24, 0.23), vec3(0.20, 0.17, 0.15), fine);
    vec3 color = valley;
    color = mix(color, scrub, smoothstep(820.0, 1150.0, h + variation * 200.0));
    color = mix(color, rock, smoothstep(2000.0, 2700.0, h + variation * 400.0));
    color = mix(color, high, smoothstep(3600.0, 4300.0, h + fine * 300.0));
    // Cliffs are rock at any height.
    color = mix(color, rock * 0.9, smoothstep(0.45, 0.7, steep));

    // Snow: above the seasonal line, patchy around it, sliding off the steepest faces.
    float snow = smoothstep(snowline - 250.0, snowline + 350.0, h + (variation - 0.5) * 700.0);
    snow *= 1.0 - smoothstep(0.5, 0.75, steep + (fine - 0.5) * 0.2);
    color = mix(color, vec3(0.86, 0.88, 0.92), snow);

    float lambert = max(dot(n, sunDirection), 0.0);
    float shadow = sunShadow(vWorld, n);
    vec3 ambient = mix(groundAmbient, skyAmbient, n.y * 0.5 + 0.5);
    vec3 lit = color * (sunColor * lambert * shadow + ambient);

    if (imageryOn > 0.5) {
      // The photograph already carries the relief shading of the morning it was taken, so the
      // sun only adds half its slope term — but all of its shadows and its colour.
      vec3 photo = color;
      vec4 level = imageryLevel(imagery4, imageryRect4);
      photo = mix(photo, level.rgb, level.a);
      level = imageryLevel(imagery3, imageryRect3);
      photo = mix(photo, level.rgb, level.a);
      level = imageryLevel(imagery2, imageryRect2);
      photo = mix(photo, level.rgb, level.a);
      level = imageryLevel(imagery1, imageryRect1);
      photo = mix(photo, level.rgb, level.a);
      level = imageryLevel(imagery0, imageryRect0);
      photo = mix(photo, level.rgb, level.a);
      vec3 photoLit = photo * (sunColor * shadow * (0.45 + 0.55 * lambert) + ambient * 0.8);
      // Seasonal snow has no shading baked in, so it takes the full slope term: that is what
      // keeps a snowed-in range from turning into a flat white band.
      vec3 snowLit = vec3(0.8, 0.82, 0.86) * (sunColor * lambert * shadow + ambient);
      lit = mix(photoLit, snowLit, snow * 0.7);
    }

    // Aerial perspective: haze thickens with distance and thins with altitude.
    float distance = length(vWorld);
    float meanAltitude = 0.5 * (h + eyeElevation);
    float density = exp(-max(meanAltitude - 700.0, 0.0) / 2600.0);
    float haze = 1.0 - exp(-distance / 75000.0 * density * 1.6);
    vec3 view = normalize(vWorld);
    float toSun = pow(max(dot(view, sunDirection), 0.0), 6.0);
    vec3 air = mix(hazeColor, hazeSunColor, toSun);
    gl_FragColor = vec4(mix(lit, air, clamp(haze, 0.0, 0.92)), 1.0);
    #include <tonemapping_fragment>
    #include <colorspace_fragment>
  }
`;

// ------------------------------------------------------------------------------ the view

export async function createPanoramaView3D(container: HTMLElement, data: ViewData, startAzimuth: number): Promise<PanoramaView3D> {
  const L = data.labels;
  const status = container.querySelector<HTMLElement>("[data-mp-3d-status]");
  const fail = (message: string) => {
    if (status) {
      status.hidden = false;
      status.textContent = message;
    }
  };

  let renderer: THREE.WebGLRenderer;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true, powerPreference: "high-performance" });
  } catch {
    fail(L.failed);
    return inert(startAzimuth);
  }
  const small = Math.min(window.innerWidth, window.innerHeight) < 700;
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, small ? 1.5 : 2));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.55;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.prepend(renderer.domElement);

  // ---- terrain
  const { mesh: grid, viewpoint } = data;
  const eye = viewpoint.eye;
  const R = grid.effectiveRadius;
  let heights: Float32Array;
  try {
    const blob = await (await fetch(grid.file)).blob();
    const bitmap = await createImageBitmap(blob, { colorSpaceConversion: "none", premultiplyAlpha: "none" });
    const canvas = document.createElement("canvas");
    canvas.width = bitmap.width;
    canvas.height = bitmap.height;
    const context = canvas.getContext("2d", { willReadFrequently: true })!;
    context.drawImage(bitmap, 0, 0);
    const rgba = context.getImageData(0, 0, bitmap.width, bitmap.height).data;
    // Centimetres, 24-bit big-endian across R, G and B (see scripts/terrain.mjs).
    heights = new Float32Array(bitmap.width * bitmap.height);
    for (let i = 0; i < heights.length; i++) heights[i] = (rgba[i * 4] * 65536 + rgba[i * 4 + 1] * 256 + rgba[i * 4 + 2]) / 100;
  } catch {
    renderer.dispose();
    renderer.domElement.remove();
    fail(L.failed);
    return inert(startAzimuth);
  }

  // On small screens every other bearing is plenty, and halves the memory.
  const stride = small ? 2 : 1;
  const cols = grid.cols / stride;
  const rings = grid.rings;
  const count = cols * rings + 1;
  const positions = new Float32Array(count * 3);
  const elevations = new Float32Array(count);
  for (let r = 0; r < rings; r++) {
    const d = grid.r0 * grid.ratio ** r;
    const drop = (d * d) / (2 * R);
    for (let c = 0; c < cols; c++) {
      const θ = c * stride * grid.azStep * RAD;
      const h = heights[r * grid.cols + c * stride];
      const i = r * cols + c;
      positions[i * 3] = d * Math.sin(θ);
      positions[i * 3 + 1] = h - eye - drop;
      positions[i * 3 + 2] = -d * Math.cos(θ);
      elevations[i] = h;
    }
  }
  const centre = count - 1;
  positions[centre * 3 + 1] = viewpoint.elevation - eye;
  elevations[centre] = viewpoint.elevation;

  // Web Mercator position of every vertex, relative to the summit and in equatorial metres,
  // so the satellite mosaic can be looked up per fragment without losing float precision.
  const mercator = new Float32Array(count * 2);
  {
    const φ1 = viewpoint.lat * RAD;
    const λ1 = viewpoint.lon * RAD;
    const x0 = worldX(viewpoint.lon);
    const y0 = worldY(viewpoint.lat);
    for (let r = 0; r < rings; r++) {
      const δ = (grid.r0 * grid.ratio ** r) / EARTH;
      for (let c = 0; c < cols; c++) {
        const θ = c * stride * grid.azStep * RAD;
        const φ2 = Math.asin(Math.sin(φ1) * Math.cos(δ) + Math.cos(φ1) * Math.sin(δ) * Math.cos(θ));
        const λ2 = λ1 + Math.atan2(Math.sin(θ) * Math.sin(δ) * Math.cos(φ1), Math.cos(δ) - Math.sin(φ1) * Math.sin(φ2));
        const i = r * cols + c;
        mercator[i * 2] = (worldX(λ2 / RAD) - x0) * CIRCUMFERENCE;
        mercator[i * 2 + 1] = (worldY(φ2 / RAD) - y0) * CIRCUMFERENCE;
      }
    }
  }
  const imagery = createImagery(viewpoint, renderer.capabilities.getMaxAnisotropy(), () => {
    dirty = true;
  });

  const index = new Uint32Array((rings - 1) * cols * 6 + cols * 3);
  let k = 0;
  for (let r = 0; r < rings - 1; r++) {
    for (let c = 0; c < cols; c++) {
      const a = r * cols + c;
      const b = r * cols + ((c + 1) % cols);
      const c1 = (r + 1) * cols + c;
      const d1 = (r + 1) * cols + ((c + 1) % cols);
      index[k++] = a; index[k++] = b; index[k++] = c1;
      index[k++] = b; index[k++] = d1; index[k++] = c1;
    }
  }
  for (let c = 0; c < cols; c++) {
    index[k++] = centre; index[k++] = (c + 1) % cols; index[k++] = c;
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("elevation", new THREE.BufferAttribute(elevations, 1));
  geometry.setAttribute("mercator", new THREE.BufferAttribute(mercator, 2));
  geometry.setIndex(new THREE.BufferAttribute(index, 1));
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();

  // ---- sun shadows: a depth map rendered from the sun, over the 110 km around the hill.
  const SHADOW_SIZE = 4096;
  const SHADOW_REACH = 110_000;
  const shadowTarget = new THREE.WebGLRenderTarget(SHADOW_SIZE, SHADOW_SIZE, {
    depthTexture: new THREE.DepthTexture(SHADOW_SIZE, SHADOW_SIZE),
    depthBuffer: true,
  });
  shadowTarget.depthTexture!.minFilter = THREE.NearestFilter;
  shadowTarget.depthTexture!.magFilter = THREE.NearestFilter;
  const shadowCamera = new THREE.OrthographicCamera(-SHADOW_REACH, SHADOW_REACH, SHADOW_REACH, -SHADOW_REACH, 1, 500_000);
  const shadowScene = new THREE.Scene();
  const shadowMesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ colorWrite: false }));
  shadowScene.add(shadowMesh);

  const uniforms = {
    sunDirection: { value: new THREE.Vector3(0, 1, 0) },
    sunColor: { value: new THREE.Color() },
    skyAmbient: { value: new THREE.Color() },
    groundAmbient: { value: new THREE.Color() },
    hazeColor: { value: new THREE.Color() },
    hazeSunColor: { value: new THREE.Color() },
    eyeElevation: { value: viewpoint.elevation + 1.7 },
    snowline: { value: 3000 },
    shadowMap: { value: shadowTarget.depthTexture },
    shadowMatrix: { value: new THREE.Matrix4() },
    shadowTexel: { value: (2 * SHADOW_REACH) / SHADOW_SIZE },
    shadowsOn: { value: 1 },
    imageryOn: { value: 0 },
    ...Object.fromEntries(imagery.levels.flatMap((level, i) => [[`imagery${i}`, { value: level.texture }], [`imageryRect${i}`, { value: level.rect }]])),
  };
  const material = new THREE.ShaderMaterial({ vertexShader: TERRAIN_VERTEX, fragmentShader: TERRAIN_FRAGMENT, uniforms });
  const terrain = new THREE.Mesh(geometry, material);
  terrain.frustumCulled = false;

  const scene = new THREE.Scene();
  scene.add(terrain);
  const sky = new Sky();
  sky.scale.setScalar(450_000);
  sky.renderOrder = -1;
  const skyUniforms = sky.material.uniforms;
  skyUniforms.turbidity.value = 3.2;
  skyUniforms.rayleigh.value = 1.1;
  skyUniforms.mieCoefficient.value = 0.004;
  skyUniforms.mieDirectionalG.value = 0.82;
  scene.add(sky);

  const camera = new THREE.PerspectiveCamera(55, 1, 0.5, 600_000);
  camera.rotation.order = "YXZ";

  // ---- labels and controls
  const labelLayer = document.createElement("div");
  labelLayer.className = "mp-3d__labels";
  const compassPill = document.createElement("div");
  compassPill.className = "mp-3d__compass";
  const attribution = document.createElement("div");
  attribution.className = "mp-3d__attribution";
  attribution.textContent = "Copernicus GLO-30 · OpenStreetMap";
  const IMAGERY_CREDIT = "Imagery © Esri, Maxar, Earthstar Geographics · Copernicus GLO-30 · OpenStreetMap";
  const controls = document.createElement("div");
  controls.className = "mp-3d__controls";
  const today = new Date();
  const month = today.getMonth();
  const currentSeason: Season = month <= 1 || month === 11 ? "summer" : month <= 4 ? "autumn" : month <= 7 ? "winter" : "spring";
  controls.innerHTML = `
    <label>${L.surface}
      <select data-surface>
        <option value="satellite" selected>${L.satellite}</option>
        <option value="relief">${L.relief}</option>
      </select>
    </label>
    <label>${L.season}
      <select data-season>
        ${(Object.keys(SEASONS) as Season[]).map((s) => `<option value="${s}"${s === currentSeason ? " selected" : ""}>${L[s]}</option>`).join("")}
      </select>
    </label>
    <label>${L.time}
      <input type="range" min="300" max="1320" step="5" data-time aria-label="${L.time}">
      <span class="mp-3d__clock" data-clock></span>
    </label>
    <button type="button" data-sunrise>${L.sunrise}</button>
    <button type="button" data-sunset>${L.sunset}</button>
    <label><input type="checkbox" data-labels checked> ${L.labels}</label>
    <button type="button" data-fullscreen>${L.fullscreen}</button>
  `;
  container.append(labelLayer, compassPill, attribution, controls);
  status?.setAttribute("hidden", "");

  const seasonSelect = controls.querySelector<HTMLSelectElement>("[data-season]")!;
  const timeInput = controls.querySelector<HTMLInputElement>("[data-time]")!;
  const clock = controls.querySelector<HTMLElement>("[data-clock]")!;
  const labelsToggle = controls.querySelector<HTMLInputElement>("[data-labels]")!;
  const surfaceSelect = controls.querySelector<HTMLSelectElement>("[data-surface]")!;
  const setSurface = (surface: string) => {
    const satellite = surface === "satellite";
    uniforms.imageryOn.value = satellite ? 1 : 0;
    attribution.textContent = satellite ? IMAGERY_CREDIT : "Copernicus GLO-30 · OpenStreetMap";
    if (satellite) imagery.start();
    dirty = true;
  };
  surfaceSelect.addEventListener("change", () => setSurface(surfaceSelect.value));

  const peaks = data.peaks.map((peak) => {
    const d = peak.distance * 1000;
    const θ = peak.azimuth * RAD;
    const y = peak.model - eye - (d * d) / (2 * R);
    const element = document.createElement("div");
    element.className = `mp-3d__label${peak.status === "hidden" ? " mp-3d__label--hidden" : ""}${peak.url ? " mp-3d__label--climbed" : ""}`;
    const name = peak.url ? `<a href="${peak.url}">${escape(peak.short)} ↗\uFE0E</a>` : escape(peak.short);
    element.innerHTML = `${name}<small>${peak.ele.toLocaleString(document.documentElement.lang || "es")} m · ${Math.round(peak.distance)} km</small>`;
    element.hidden = true;
    labelLayer.append(element);
    const priority = (peak.named ? 100_000 : 0) + (peak.url ? 50_000 : 0) + (peak.status === "hidden" ? -80_000 : 0) + peak.ele;
    return { ...peak, position: new THREE.Vector3(d * Math.sin(θ), y, -d * Math.cos(θ)), element, priority, width: 0 };
  });
  peaks.sort((a, b) => b.priority - a.priority);

  // ---- state
  let azimuth = startAzimuth;
  let pitch = 2.5;
  let fov = 55;
  let season: Season = currentSeason;
  let minutes = 0;
  let dirty = true;
  let paused = false;
  let shadowDirty = true;
  let frame = 0;

  const dateFor = (m: number) => {
    const { month, day, utcOffset } = SEASONS[season];
    return new Date(Date.UTC(today.getFullYear(), month, day, 0, 0) + (m - utcOffset * 60) * 60_000);
  };
  const sunAt = (m: number) => sunPosition(dateFor(m), viewpoint.lat, viewpoint.lon);
  /** Minute of the local day when the sun crosses the horizon, rising or setting. */
  const crossing = (rising: boolean) => {
    let previous = sunAt(rising ? 240 : 1080).elevation;
    for (let m = rising ? 241 : 1081; m < (rising ? 600 : 1380); m++) {
      const current = sunAt(m).elevation;
      if (rising ? previous < -0.83 && current >= -0.83 : previous > -0.83 && current <= -0.83) return m;
      previous = current;
    }
    return rising ? 420 : 1200;
  };
  const formatClock = (m: number) => `${String(Math.floor(m / 60)).padStart(2, "0")}:${String(m % 60).padStart(2, "0")}`;

  function updateSun() {
    const sun = sunAt(minutes);
    const dir = direction(sun.azimuth, sun.elevation);
    uniforms.sunDirection.value.copy(dir);
    skyUniforms.sunPosition.value.copy(dir);
    uniforms.snowline.value = SEASONS[season].snowline;

    // Light colour through the atmosphere: white at noon, gold, then red at the horizon.
    const el = sun.elevation;
    const day = smooth(-4, 6, el);
    const golden = 1 - smooth(2, 18, el);
    const sunColor = new THREE.Color().setRGB(1, 0.93 - 0.42 * golden, 0.84 - 0.66 * golden);
    uniforms.sunColor.value.copy(sunColor).multiplyScalar(2.9 * smooth(-1.5, 3, el));
    uniforms.skyAmbient.value.setRGB(0.3, 0.42, 0.62).multiplyScalar(0.06 + 0.62 * day);
    uniforms.groundAmbient.value.setRGB(0.25, 0.2, 0.15).multiplyScalar(0.03 + 0.3 * day);
    const haze = new THREE.Color().setRGB(0.52, 0.62, 0.78).lerp(new THREE.Color().setRGB(0.75, 0.55, 0.45), golden * 0.7);
    uniforms.hazeColor.value.copy(haze).multiplyScalar(0.03 + 1.05 * day);
    uniforms.hazeSunColor.value.copy(sunColor).multiplyScalar(0.05 + 1.4 * day);
    renderer.toneMappingExposure = 0.5 + 0.35 * (1 - day);
    uniforms.shadowsOn.value = el > -1 ? 1 : 0;
    shadowDirty = true;
    clock.textContent = formatClock(minutes);
    dirty = true;
  }

  function renderShadows() {
    const dir = uniforms.sunDirection.value;
    if (dir.y < -0.02) return;
    shadowCamera.position.copy(dir).multiplyScalar(250_000);
    shadowCamera.up.set(0, 1, 0);
    if (Math.abs(dir.y) > 0.98) shadowCamera.up.set(0, 0, -1);
    shadowCamera.lookAt(0, 0, 0);
    shadowCamera.updateMatrixWorld();
    shadowCamera.updateProjectionMatrix();
    renderer.setRenderTarget(shadowTarget);
    renderer.clear();
    renderer.render(shadowScene, shadowCamera);
    renderer.setRenderTarget(null);
    uniforms.shadowMatrix.value.multiplyMatrices(shadowCamera.projectionMatrix, shadowCamera.matrixWorldInverse);
  }

  function resize() {
    const width = container.clientWidth;
    const height = container.clientHeight;
    if (!width || !height) return;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    dirty = true;
  }

  const projected = new THREE.Vector3();
  function placeLabels(width: number, height: number) {
    const show = labelsToggle.checked;
    const boxes: { x0: number; x1: number; y0: number; y1: number }[] = [];
    for (const peak of peaks) {
      const element = peak.element;
      if (!show) {
        element.hidden = true;
        continue;
      }
      projected.copy(peak.position).project(camera);
      const x = (projected.x * 0.5 + 0.5) * width;
      const y = (-projected.y * 0.5 + 0.5) * height;
      if (projected.z > 1 || x < -40 || x > width + 40 || y < 0 || y > height) {
        element.hidden = true;
        continue;
      }
      if (element.hidden) element.hidden = false;
      if (!peak.width) peak.width = element.offsetWidth || 80;
      const half = peak.width / 2 + 3;
      let placedAt = -1;
      for (let row = 0; row < 6; row++) {
        const leader = 14 + row * 30;
        const top = y - leader - 30;
        if (top < 4) break;
        const box = { x0: x - half, x1: x + half, y0: top, y1: top + 28 };
        if (!boxes.some((b) => b.x0 < box.x1 && box.x0 < b.x1 && b.y0 < box.y1 && box.y0 < b.y1)) {
          boxes.push(box);
          placedAt = leader;
          break;
        }
      }
      if (placedAt < 0) {
        element.hidden = true;
        continue;
      }
      element.style.setProperty("--leader", `${placedAt}px`);
      element.style.transform = `translate(${x.toFixed(1)}px, ${y.toFixed(1)}px) translate(-50%, -100%)`;
    }
  }

  const COMPASS = document.documentElement.lang?.startsWith("en")
    ? ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    : ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"];

  function render() {
    frame = requestAnimationFrame(render);
    if (paused || !dirty) return;
    dirty = false;
    if (shadowDirty) {
      renderShadows();
      shadowDirty = false;
    }
    azimuth = ((azimuth % 360) + 360) % 360;
    camera.rotation.y = -azimuth * RAD;
    camera.rotation.x = pitch * RAD;
    camera.fov = fov;
    camera.updateProjectionMatrix();
    camera.updateMatrixWorld();
    renderer.render(scene, camera);
    placeLabels(container.clientWidth, container.clientHeight);
    compassPill.textContent = `${COMPASS[Math.round(azimuth / 22.5) % 16]} ${Math.round(azimuth)}° · ${Math.round(fov)}°`;
  }

  // ---- input: drag to look, wheel or pinch to zoom, arrows to turn
  const canvas = renderer.domElement;
  canvas.tabIndex = 0;
  const pointers = new Map<number, { x: number; y: number }>();
  let pinch = 0;
  canvas.addEventListener("pointerdown", (event) => {
    canvas.setPointerCapture(event.pointerId);
    pointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
    canvas.classList.add("is-dragging");
  });
  canvas.addEventListener("pointermove", (event) => {
    const last = pointers.get(event.pointerId);
    if (!last) return;
    if (pointers.size === 2) {
      const [a, b] = [...pointers.values()];
      const other = a === last ? b : a;
      const before = Math.hypot(last.x - other.x, last.y - other.y);
      const after = Math.hypot(event.clientX - other.x, event.clientY - other.y);
      if (pinch && before > 0) fov = Math.min(80, Math.max(4, (fov * before) / after));
      pinch = 1;
    } else {
      const degreesPerPixel = fov / container.clientHeight;
      azimuth -= (event.clientX - last.x) * degreesPerPixel;
      pitch = Math.min(40, Math.max(-35, pitch + (event.clientY - last.y) * degreesPerPixel));
    }
    pointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
    dirty = true;
  });
  const release = (event: PointerEvent) => {
    pointers.delete(event.pointerId);
    if (pointers.size < 2) pinch = 0;
    if (!pointers.size) canvas.classList.remove("is-dragging");
  };
  canvas.addEventListener("pointerup", release);
  canvas.addEventListener("pointercancel", release);
  canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      fov = Math.min(80, Math.max(4, fov * Math.exp(event.deltaY * 0.0012)));
      dirty = true;
    },
    { passive: false },
  );
  canvas.addEventListener("keydown", (event) => {
    const step = fov / 8;
    if (event.key === "ArrowLeft") azimuth -= step;
    else if (event.key === "ArrowRight") azimuth += step;
    else if (event.key === "ArrowUp") pitch = Math.min(40, pitch + step / 2);
    else if (event.key === "ArrowDown") pitch = Math.max(-35, pitch - step / 2);
    else if (event.key === "+" || event.key === "=") fov = Math.max(4, fov / 1.2);
    else if (event.key === "-") fov = Math.min(80, fov * 1.2);
    else return;
    event.preventDefault();
    dirty = true;
  });

  seasonSelect.addEventListener("change", () => {
    season = seasonSelect.value as Season;
    updateSun();
  });
  timeInput.addEventListener("input", () => {
    minutes = Number(timeInput.value);
    updateSun();
  });
  const setTime = (m: number) => {
    minutes = Math.round(m / 5) * 5;
    timeInput.value = String(minutes);
    updateSun();
  };
  controls.querySelector("[data-sunrise]")!.addEventListener("click", () => setTime(crossing(true) + 10));
  controls.querySelector("[data-sunset]")!.addEventListener("click", () => setTime(crossing(false) - 10));
  labelsToggle.addEventListener("change", () => (dirty = true));
  controls.querySelector("[data-fullscreen]")!.addEventListener("click", () => {
    if (document.fullscreenElement) document.exitFullscreen();
    else container.requestFullscreen?.();
  });

  const observer = new ResizeObserver(resize);
  observer.observe(container);

  // Open at the golden hour: the low western sun on the Andes is the view from this hill.
  setTime(crossing(false) - 40);
  setSurface("satellite");
  resize();
  render();

  // Smooth turns for the buttons and the table.
  let animation = 0;
  const animateTo = (target: { azimuth: number; pitch?: number; fov?: number }) => {
    cancelAnimationFrame(animation);
    const from = { azimuth, pitch, fov };
    let delta = (((target.azimuth - azimuth) % 360) + 540) % 360 - 180;
    const start = performance.now();
    const step = (now: number) => {
      const t = Math.min(1, (now - start) / 700);
      const e = t < 0.5 ? 2 * t * t : 1 - (-2 * t + 2) ** 2 / 2;
      azimuth = from.azimuth + delta * e;
      if (target.pitch !== undefined) pitch = from.pitch + (target.pitch - from.pitch) * e;
      if (target.fov !== undefined) fov = from.fov + (target.fov - from.fov) * e;
      dirty = true;
      if (t < 1) animation = requestAnimationFrame(step);
    };
    animation = requestAnimationFrame(step);
  };

  return {
    lookAt(id) {
      const peak = peaks.find((p) => p.id === id);
      if (peak) animateTo({ azimuth: peak.azimuth, pitch: peak.angle + 1, fov: Math.min(fov, 22) });
    },
    turnTo(target) {
      animateTo({ azimuth: target });
    },
    heading: () => azimuth,
    pause() {
      paused = true;
    },
    resume() {
      paused = false;
      resize();
      dirty = true;
    },
    destroy() {
      cancelAnimationFrame(frame);
      cancelAnimationFrame(animation);
      observer.disconnect();
      geometry.dispose();
      material.dispose();
      shadowMesh.material.dispose();
      shadowTarget.dispose();
      imagery.dispose();
      sky.material.dispose();
      renderer.dispose();
      container.replaceChildren();
    },
  };
}

function escape(text: string) {
  return text.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

/** Stand-in when WebGL isn't there: keeps the tab working, shows the message. */
function inert(azimuth: number): PanoramaView3D {
  return {
    lookAt() {},
    turnTo(target) {
      azimuth = target;
    },
    heading: () => azimuth,
    pause() {},
    resume() {},
    destroy() {},
  };
}
