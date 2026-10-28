// One tooltip for the small charts of the mountaineering pages: any element with data-tip
// inside a [data-chart] shows it on hover or keyboard focus, next to the mark. Lines are split
// on "\n"; the first one is the title.

let tip: HTMLDivElement | null = null;

function show(target: HTMLElement, x: number, y: number) {
  tip ??= Object.assign(document.createElement("div"), { className: "chart-tip", role: "tooltip" });
  if (!tip.isConnected) document.body.append(tip);
  const [title, ...lines] = target.dataset.tip!.split("\n");
  tip.replaceChildren(Object.assign(document.createElement("strong"), { textContent: title }), ...lines.map((line) => Object.assign(document.createElement("span"), { textContent: line })));
  tip.hidden = false;
  const { width, height } = tip.getBoundingClientRect();
  const left = Math.min(window.innerWidth - width - 8, Math.max(8, x - width / 2));
  const top = y - height - 14 < 8 ? y + 18 : y - height - 14;
  tip.style.transform = `translate(${Math.round(left)}px, ${Math.round(top)}px)`;
}

const hide = () => tip && (tip.hidden = true);

export function initChartTooltips() {
  for (const chart of document.querySelectorAll<HTMLElement>("[data-chart]:not([data-chart-ready])")) {
    chart.dataset.chartReady = "";
    chart.addEventListener("pointermove", (event) => {
      const target = (event.target as Element).closest<HTMLElement>("[data-tip]");
      if (target && chart.contains(target)) show(target, event.clientX, event.clientY);
      else hide();
    });
    chart.addEventListener("pointerleave", hide);
    chart.addEventListener("focusin", (event) => {
      const target = (event.target as Element).closest<HTMLElement>("[data-tip]");
      if (!target) return;
      const box = target.getBoundingClientRect();
      show(target, box.left + box.width / 2, box.top);
    });
    chart.addEventListener("focusout", hide);
  }
}

document.addEventListener("astro:before-swap", () => {
  tip?.remove();
  tip = null;
});
