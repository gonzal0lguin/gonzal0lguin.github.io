// The home page's horizon: moving over it reads out the nearest named summit, like pointing a
// sensor along a bearing, and the buttons bring in the view from another top.

function wireStage(root: HTMLElement) {
  const frame = root.querySelector<HTMLElement>("[data-hh-frame]");
  const stage = root.querySelector<HTMLElement>("[data-hh-stage]");
  const cursor = root.querySelector<HTMLElement>("[data-hh-cursor]");
  const output = root.querySelector<HTMLElement>("[data-hh-readout]");
  const data = root.querySelector("[data-hh-peaks]");
  if (!frame || !stage || !cursor || !output || !data) return;
  const peaks: [number, string][] = JSON.parse(data.textContent!);
  const rest = frame.dataset.readout ?? "";
  output.textContent = rest;

  const move = (event: PointerEvent) => {
    const box = stage.getBoundingClientRect();
    const x = ((event.clientX - box.left) / box.width) * 100;
    let best: [number, string] | null = null;
    for (const peak of peaks) if (!best || Math.abs(peak[0] - x) < Math.abs(best[0] - x)) best = peak;
    cursor.hidden = false;
    cursor.style.left = `${best ? best[0] : x}%`;
    output.textContent = best?.[1] ?? rest;
  };
  const leave = () => {
    cursor.hidden = true;
    output.textContent = rest;
  };
  stage.addEventListener("pointermove", move);
  stage.addEventListener("pointerdown", move);
  stage.addEventListener("pointerleave", leave);
}

export function initHomeHorizon() {
  const root = document.querySelector<HTMLElement>("[data-hh]:not([data-ready])");
  if (!root) return;
  root.dataset.ready = "";
  wireStage(root);

  const buttons = [...root.querySelectorAll<HTMLButtonElement>("[data-hh-pick]")];
  const slot = root.querySelector<HTMLElement>("[data-hh-slot]")!;
  let request = 0;
  for (const button of buttons) {
    button.addEventListener("click", async () => {
      if (button.getAttribute("aria-pressed") === "true") return;
      const mine = ++request;
      for (const other of buttons) other.setAttribute("aria-pressed", String(other === button));
      slot.setAttribute("aria-busy", "true");
      try {
        const response = await fetch(button.dataset.hhPick!);
        if (!response.ok) throw new Error(String(response.status));
        const html = await response.text();
        if (mine !== request) return;
        slot.innerHTML = html;
        wireStage(root);
      } catch {
        // The view that's showing stays; only the pressed button has to go back.
        for (const other of buttons) other.setAttribute("aria-pressed", String(other.dataset.hhInitial === "true"));
      } finally {
        slot.removeAttribute("aria-busy");
      }
    });
  }
}
