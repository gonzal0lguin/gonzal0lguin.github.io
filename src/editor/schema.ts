// What the frontmatter form shows for each collection. Mirrors the zod schemas in
// src/content/config.ts: keep the two in step when a field is added there.

export type FieldType =
  | "text"
  | "textarea"
  | "date"
  | "number"
  | "tags"
  | "image"
  | "select"
  | "routes"
  | "coords"
  | "timezone"
  | "waypoints"
  | "checkbox"
  | "summits"
  | "photos";

export interface Field {
  key: string;
  label: string;
  type: FieldType;
  hint?: string;
  required?: boolean;
  options?: string[];
  placeholder?: string;
  step?: number;
  /** The pair of keys a `coords` field writes: latitude/longitude in a post, lat/lon in a summit. */
  keys?: [string, string];
}

const common: Field[] = [
  { key: "title", label: "Título", type: "text", required: true },
  { key: "description", label: "Descripción", type: "textarea", hint: "Se usa en los listados y en las vistas previas de los enlaces" },
  { key: "pubDate", label: "Fecha", type: "date", required: true },
];

const closing: Field[] = [
  { key: "heroImage", label: "Imagen de portada", type: "image", hint: "Ruta pública, p.ej. /assets/img/headers/about.JPG" },
  { key: "badge", label: "Insignia", type: "text", hint: "Etiqueta destacada junto al título" },
  { key: "tags", label: "Tags", type: "tags" },
];

export const FIELDS: Record<string, Field[]> = {
  ascents: [
    ...common,
    { key: "mountain", label: "Montaña", type: "text" },
    { key: "elevation", label: "Altitud (m)", type: "number" },
    { key: "summits", label: "Cumbres", type: "summits", hint: "Del registro de cumbres: dan la pestaña de panorama del post y lo ponen en la página de cada cumbre" },
    { key: "attempt", label: "Intento (no se hizo cumbre)", type: "checkbox", hint: "Las cumbres de arriba pasan a ser las intentadas; el punto más alto del track se marca como tal" },
    { key: "latitude", label: "Latitud / Longitud", type: "coords", keys: ["latitude", "longitude"], hint: "Ubicación del mapa de ascensos; sin cumbre del registro, es el pin de la entrada" },
    { key: "route", label: "Ruta GPX", type: "routes", hint: "Uno o varios .gpx en public/; se unen en orden cronológico" },
    { key: "waypoints", label: "Puntos marcados", type: "waypoints", hint: "Se editan en la pestaña Ruta" },
    { key: "timezone", label: "Zona horaria", type: "timezone", hint: "Para las horas del track (por defecto America/Santiago)" },
    { key: "category", label: "Categoría", type: "text" },
    ...closing,
  ],
  blog: [...common, { key: "updatedDate", label: "Actualizado", type: "date" }, { key: "category", label: "Categoría", type: "text" }, ...closing],
  projects: [
    ...common,
    { key: "updatedDate", label: "Actualizado", type: "date" },
    { key: "category", label: "Categoría", type: "select", options: ["", "academic-projects", "hobby-projects", "personal"], hint: "Define la URL /projects/<categoría>" },
    ...closing,
  ],
  pages: [
    { key: "title", label: "Título", type: "text", required: true },
    { key: "description", label: "Descripción", type: "textarea" },
    { key: "greeting", label: "Saludo", type: "text" },
    { key: "subtitle", label: "Subtítulo", type: "text" },
  ],
  // One summit of the registry (src/content/summits/*.yaml). Not a post: no translation, no
  // body, and `panorama` decides whether npm run panoramas computes the view from its top.
  summits: [
    { key: "name", label: "Nombre", type: "text", required: true, placeholder: "Cerro Provincia" },
    { key: "elevation", label: "Altitud oficial (m)", type: "number", required: true },
    { key: "lat", label: "Latitud / Longitud", type: "coords", keys: ["lat", "lon"], hint: "La cumbre misma: de aquí sale el panorama, el perfil y el pin del mapa" },
    { key: "region", label: "Zona", type: "text", hint: "Como se muestra en la lista y en la página, p. ej. Santiago · Precordillera" },
    { key: "osm", label: "Nodo de OpenStreetMap", type: "number", hint: "Para reconocer esta cumbre en los panoramas de las otras" },
    { key: "wikidata", label: "Wikidata", type: "text", placeholder: "Q5064972" },
    { key: "panorama", label: "Calcular el panorama", type: "checkbox", hint: "Con esto marcado, npm run panoramas calcula la vista desde la cumbre (no al norte ni al este de 0°)" },
    { key: "timezone", label: "Zona horaria", type: "timezone" },
    { key: "photos", label: "Fotos desde la cumbre", type: "photos", hint: "El azimut (grados desde el norte, hacia dónde mira la foto) las ubica en el panorama" },
  ],
};

/** The skeleton a new summit of the registry starts from; the rest is filled in the form. */
export const SUMMIT_TEMPLATE = (name: string) => `name: "${name.replace(/"/g, '\\"')}"\nelevation: 0\nlat: 0\nlon: 0\npanorama: false\n`;

/** The skeleton a new entry starts from. */
export const TEMPLATES: Record<string, string> = {
  ascents: `\n## Descripción\n\n\n## Aproximación\n\n\n## Cumbre\n\n\n## Comentarios finales\n\n`,
  blog: `\n`,
  projects: `\n`,
  pages: `\n`,
};

export const WAYPOINT_KIND_IDS = ["camp", "summit", "view", "water", "hut", "crux", "danger", "parking", "note"] as const;

export const TIMEZONES = [
  "",
  "America/Santiago",
  "America/Argentina/Mendoza",
  "America/Argentina/Buenos_Aires",
  "America/La_Paz",
  "America/Lima",
  "America/Mexico_City",
  "Europe/Madrid",
  "Europe/Berlin",
  "Europe/Zurich",
  "UTC",
];

/** Aspect-ratio presets offered for a photo group (`ratio=` of remark-media). */
export const RATIOS = ["", "auto", "4/3", "3/2", "16/9", "1/1", "2/3", "3/4"];

/** How a photo's place on the route map can be given (the "#" fragment). */
export const GEO_MODES = [
  { id: "", label: "Automático (EXIF)" },
  { id: "km", label: "Kilómetro de la ruta" },
  { id: "time", label: "Hora" },
  { id: "coords", label: "Coordenadas" },
  { id: "off", label: "Nunca en el mapa" },
];
