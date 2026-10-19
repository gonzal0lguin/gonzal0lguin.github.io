// Kinds of marked points (waypoints): their icon, color and the GPX <sym> other apps understand.
// Shared by the parser (reading <type>/<sym>), the map (icons) and the route tool (marking).

export const WAYPOINT_KINDS = {
  camp: {
    color: "#7c3aed",
    sym: "Campground",
    icon: '<path d="M2 20h20M4 20 12 5l8 15M12 5l2-2M12 5l-2-2M9.5 20l2.5-5 2.5 5" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"/>',
  },
  summit: {
    color: "#d97706",
    sym: "Summit",
    icon: '<path d="M2 20 9.5 7l4 6.5L16 9.5 22 20z" fill="currentColor"/>',
  },
  view: {
    color: "#0d9488",
    sym: "Scenic Area",
    icon: '<path d="M2 12s3.6-7 10-7 10 7 10 7-3.6 7-10 7S2 12 2 12z" fill="none" stroke="currentColor" stroke-width="2.2"/><circle cx="12" cy="12" r="3" fill="currentColor"/>',
  },
  water: {
    color: "#2563eb",
    sym: "Drinking Water",
    icon: '<path d="M12 3s6 6.7 6 11a6 6 0 0 1-12 0c0-4.3 6-11 6-11z" fill="currentColor"/>',
  },
  hut: {
    color: "#92400e",
    sym: "Lodge",
    icon: '<path d="M3 11 12 4l9 7M5 10v10h14V10M10 20v-6h4v6" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linejoin="round"/>',
  },
  crux: {
    color: "#ea580c",
    sym: "Trail Head",
    icon: '<path d="M7 21 12 3l5 18M8.5 15h7M10 9h4" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round"/>',
  },
  danger: {
    color: "#dc2626",
    sym: "Danger Area",
    icon: '<path d="M12 3 2 21h20L12 3z" fill="currentColor"/><path d="M12 10v5M12 18v.5" stroke="#fff" stroke-width="2.2" stroke-linecap="round"/>',
  },
  parking: {
    color: "#475569",
    sym: "Parking Area",
    icon: '<path d="M8 20V4h5a4.5 4.5 0 0 1 0 9H8" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linejoin="round"/>',
  },
  note: {
    color: "#0284c7",
    sym: "Flag, Blue",
    icon: '<path d="M6 21V4m0 0h11l-2 4 2 4H6" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linejoin="round"/>',
  },
} as const;

export type WaypointKind = keyof typeof WAYPOINT_KINDS;
export const WAYPOINT_KIND_IDS = Object.keys(WAYPOINT_KINDS) as WaypointKind[];

/** Our kind for a GPX waypoint, from its <type> (ours) or its <sym> (Garmin and most apps). */
export function waypointKind(type?: string | null, sym?: string | null): WaypointKind {
  const t = (type ?? "").trim().toLowerCase();
  if (t in WAYPOINT_KINDS) return t as WaypointKind;
  const s = (sym ?? type ?? "").toLowerCase();
  if (/camp|tent|bivouac|bivy/.test(s)) return "camp";
  if (/summit|peak|cumbre/.test(s)) return "summit";
  if (/scenic|view|overlook|mirador/.test(s)) return "view";
  if (/water|spring|agua/.test(s)) return "water";
  if (/lodge|hut|shelter|refug|house/.test(s)) return "hut";
  if (/danger|hazard|caution/.test(s)) return "danger";
  if (/parking|car/.test(s)) return "parking";
  if (/trail head|crux|pass/.test(s)) return "crux";
  return "note";
}
