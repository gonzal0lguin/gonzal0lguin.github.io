// The digital inventory: types for src/data/inventory.json (written by scripts/inventory.mjs)
// and every number the page shows, derived here so the component only lays things out.

import inventory from "../data/inventory.json";
import { SPECIES, type Habit, type Species } from "../data/species";

/** `unchecked` means the spreadsheet's status cell is empty: nobody has looked since planting. */
export type Status = "alive" | "unchecked" | "dead" | "lost" | "unknown";

export interface Plant {
  /** Code on the aluminium tag, e.g. "LT004". Not unique yet — see `issues`. */
  id: string;
  name: string;
  scientific: string;
  habit: Habit | null;
  /** ISO calendar day, or null when the spreadsheet has no date. */
  planted: string | null;
  origin: string | null;
  status: Status;
  lat: number | null;
  lon: number | null;
  /** What the coordinates cell said when it wasn't coordinates, e.g. "(mapa G)". */
  locationNote: string | null;
  height: number | null;
  notes: string | null;
}

export interface Issue {
  kind: "coordinates" | "status" | "id" | "date" | "habit" | "sequence";
  id: string;
  detail: string;
}

export const PLANTS: Plant[] = inventory.plants as Plant[];
export const ISSUES: Issue[] = inventory.issues as Issue[];
export const GENERATED: string = inventory.generated;

/** One species as the page needs it: the field guide entry plus this hill's numbers. */
export interface SpeciesRow {
  scientific: string;
  species: Species;
  plants: Plant[];
  total: number;
  alive: number;
  unchecked: number;
  lost: number;
  located: number;
}

/** One planting day: the trips are what the project actually consists of. */
export interface Wave {
  date: string;
  total: number;
  alive: number;
  unchecked: number;
  lost: number;
  /** Species codes planted that day, in descending count order. */
  species: { code: string; name: string; count: number }[];
}

const count = (plants: Plant[], status: Status) => plants.filter((plant) => plant.status === status).length;

/** Species present in the inventory, most planted first. Species with no field-guide entry are skipped. */
export function speciesRows(): SpeciesRow[] {
  const groups = new Map<string, Plant[]>();
  for (const plant of PLANTS) {
    const group = groups.get(plant.scientific);
    if (group) group.push(plant);
    else groups.set(plant.scientific, [plant]);
  }
  return [...groups]
    .filter(([scientific]) => scientific in SPECIES)
    .map(([scientific, plants]) => ({
      scientific,
      species: SPECIES[scientific],
      plants,
      total: plants.length,
      alive: count(plants, "alive"),
      unchecked: count(plants, "unchecked"),
      lost: count(plants, "lost") + count(plants, "dead"),
      located: plants.filter((plant) => plant.lat != null).length,
    }))
    .sort((a, b) => b.total - a.total || a.species.es.name.localeCompare(b.species.es.name, "es"));
}

/** Scientific names in the inventory with no entry in src/data/species.ts. */
export const unknownSpecies = (): string[] =>
  [...new Set(PLANTS.map((plant) => plant.scientific))].filter((scientific) => !(scientific in SPECIES));

export function waves(): Wave[] {
  const groups = new Map<string, Plant[]>();
  for (const plant of PLANTS) {
    if (!plant.planted) continue;
    const group = groups.get(plant.planted);
    if (group) group.push(plant);
    else groups.set(plant.planted, [plant]);
  }
  return [...groups]
    .sort(([a], [b]) => a.localeCompare(b))
    .map(([date, plants]) => {
      const perSpecies = new Map<string, { code: string; name: string; count: number }>();
      for (const plant of plants) {
        const species = SPECIES[plant.scientific];
        const key = species?.code ?? plant.scientific;
        const entry = perSpecies.get(key);
        if (entry) entry.count += 1;
        else perSpecies.set(key, { code: key, name: plant.name, count: 1 });
      }
      return {
        date,
        total: plants.length,
        alive: count(plants, "alive"),
        unchecked: count(plants, "unchecked"),
        lost: count(plants, "lost") + count(plants, "dead"),
        species: [...perSpecies.values()].sort((a, b) => b.count - a.count),
      };
    });
}

export interface Totals {
  plants: number;
  species: number;
  families: number;
  trees: number;
  shrubs: number;
  located: number;
  alive: number;
  unchecked: number;
  lost: number;
  waves: number;
  endemic: number;
  /** Area of the planted cluster's bounding box, in square metres. */
  extent: number;
  firstDate: string | null;
  lastDate: string | null;
}

export function totals(): Totals {
  const dates = PLANTS.map((plant) => plant.planted).filter((date): date is string => !!date).sort();
  const rows = speciesRows();
  const located = PLANTS.filter((plant) => plant.lat != null && plant.lon != null);
  const lats = located.map((plant) => plant.lat!);
  const lons = located.map((plant) => plant.lon!);
  // Rough metres per degree at this latitude — fine for a cluster 60 m across.
  const extent =
    located.length > 1
      ? (Math.max(...lats) - Math.min(...lats)) * 110_574 * ((Math.max(...lons) - Math.min(...lons)) * 93_500)
      : 0;

  return {
    plants: PLANTS.length,
    species: new Set(PLANTS.map((plant) => plant.scientific)).size,
    families: new Set(rows.map((row) => row.species.family)).size,
    trees: PLANTS.filter((plant) => plant.habit === "tree").length,
    shrubs: PLANTS.filter((plant) => plant.habit === "shrub").length,
    located: located.length,
    alive: count(PLANTS, "alive"),
    unchecked: count(PLANTS, "unchecked"),
    lost: count(PLANTS, "lost") + count(PLANTS, "dead"),
    waves: new Set(dates).size,
    endemic: rows.filter((row) => row.species.origin === "endemic").reduce((sum, row) => sum + row.total, 0),
    extent: Math.round(extent),
    firstDate: dates[0] ?? null,
    lastDate: dates.at(-1) ?? null,
  };
}

/**
 * How many of the planted species are in flower each month — the gap this planting is meant
 * to fill. The hill's own flowering peaks in September and October and then stops.
 */
export function floweringCalendar(): { month: number; species: string[] }[] {
  const rows = speciesRows();
  return Array.from({ length: 12 }, (_, i) => {
    const month = i + 1;
    return {
      month,
      species: rows.filter((row) => row.species.flowering.includes(month)).map((row) => row.species.code),
    };
  });
}
