// What moves in the figures of the robot post: the crosshair over the line charts, the slider
// that lets the rays reach further and the layers over the photo. The drawings are all built at
// compile time; this only shows, hides and reads them.

interface ChartData {
  kind: "ray" | "curve" | "gps";
  /** Where the x axis runs in viewBox units, and the values at those two ends. */
  from: number;
  to: number;
  min: number;
  max: number;
  width: number;
  rows: number[][];
}

// es-CL and not the page's plain "es", which leaves four-digit numbers without a separator.
const number = (value: number, digits = 0) =>
  value.toLocaleString(document.documentElement.lang === "en" ? "en-GB" : "es-CL", { minimumFractionDigits: digits, maximumFractionDigits: digits });

function describe(data: ChartData, row: number[], labels: DOMStringMap) {
  if (data.kind === "ray") {
    const [km, h, angle, highest, seen] = row;
    return `${number(km, 1)} km · ${number(h)} m · ${labels.angle} ${number(angle, 2)}° · ${labels.highest} ${number(highest, 2)}° · ${seen ? labels.seen : labels.hidden}`;
  }
  if (data.kind === "curve") {
    const [km, plain, bent] = row;
    return `${number(km)} km · ${labels.plain} ${number(plain)} m · ${labels.bent} ${number(bent)} m`;
  }
  const [km, raw, filtered] = row;
  return `${number(km, 2)} km · ${labels.raw} ${number(raw)} m · ${labels.filtered} ${number(filtered)} m`;
}

function setUpChart(root: HTMLElement) {
  root.dataset.ready = "";
  const data: ChartData = JSON.parse(root.querySelector("[data-rf-data]")!.textContent!);
  const svg = root.querySelector("svg")!;
  const cross = root.querySelector<SVGLineElement>("[data-rf-cross]")!;
  const readout = root.querySelector<HTMLElement>("[data-rf-readout]")!;
  const resting = readout.textContent;
  const move = (event: PointerEvent) => {
    const box = svg.getBoundingClientRect();
    const unit = ((event.clientX - box.left) / box.width) * data.width;
    const value = data.min + ((Math.min(data.to, Math.max(data.from, unit)) - data.from) / (data.to - data.from)) * (data.max - data.min);
    let best = data.rows[0];
    for (const row of data.rows) if (Math.abs(row[0] - value) < Math.abs(best[0] - value)) best = row;
    const x = data.from + ((best[0] - data.min) / (data.max - data.min)) * (data.to - data.from);
    cross.setAttribute("x1", String(x));
    cross.setAttribute("x2", String(x));
    cross.style.display = "";
    readout.textContent = describe(data, best, root.dataset);
  };
  const leave = () => {
    cross.style.display = "none";
    readout.textContent = resting;
  };
  svg.addEventListener("pointermove", move);
  svg.addEventListener("pointerdown", move);
  svg.addEventListener("pointerleave", leave);
}

function setUpBands(root: HTMLElement) {
  root.dataset.ready = "";
  const slider = root.querySelector<HTMLInputElement>("input[type=range]")!;
  const bands = [...root.querySelectorAll<SVGElement>("[data-rf-band]")];
  const lines = [...root.querySelectorAll<SVGElement>("[data-rf-line]")];
  const late = [...root.querySelectorAll<SVGElement>("[data-rf-from]")];
  const output = root.querySelector<HTMLElement>("[data-rf-readout]")!;
  const reach: number[] = JSON.parse(root.dataset.reach!);
  const apply = () => {
    const step = Number(slider.value);
    for (const band of bands) band.style.display = Number(band.dataset.rfBand) <= step ? "" : "none";
    for (const line of lines) line.style.display = Number(line.dataset.rfLine) === step ? "" : "none";
    for (const item of late) item.style.display = step >= Number(item.dataset.rfFrom) ? "" : "none";
    const km = reach[step];
    output.textContent = root.dataset.template!.replace("{km}", number(km, km < 2 ? 1 : 0)).replace("{n}", number(Math.floor((km * 1000 - 60) / 30) + 1));
  };
  slider.addEventListener("input", apply);
  apply();
}

function setUpPhoto(root: HTMLElement) {
  root.dataset.ready = "";
  for (const chip of root.querySelectorAll<HTMLButtonElement>("[data-rf-layer]")) {
    const layer = root.querySelector<SVGElement>(`[data-rf-group="${chip.dataset.rfLayer}"]`)!;
    const apply = () => (layer.style.display = chip.getAttribute("aria-pressed") === "true" ? "" : "none");
    chip.addEventListener("click", () => {
      chip.setAttribute("aria-pressed", String(chip.getAttribute("aria-pressed") !== "true"));
      apply();
    });
    apply();
  }
}

export function initRobotFigures() {
  for (const root of document.querySelectorAll<HTMLElement>("[data-rf-chart]:not([data-ready])")) setUpChart(root);
  for (const root of document.querySelectorAll<HTMLElement>("[data-rf-bands]:not([data-ready])")) setUpBands(root);
  for (const root of document.querySelectorAll<HTMLElement>("[data-rf-photo]:not([data-ready])")) setUpPhoto(root);
}
