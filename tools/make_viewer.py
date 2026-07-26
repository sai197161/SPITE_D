#!/usr/bin/env python3
"""Build a self-contained HTML viewer from demo_dynamic --dump-json output.

    ./build-core/demo_dynamic --roadmap DIR --dump-json data.json
    python3 tools/make_viewer.py data.json viewer.html

The result is one file with the data inlined: open it locally, or publish
it for a QR code. No network access required at view time.
"""

import json
import sys
from pathlib import Path

TEMPLATE = r"""<style>
  :root {
    --ground: #0b1014;
    --panel: #141b21;
    --panel-2: #1b242c;
    --line: #26333c;
    --ink: #dde5e9;
    --ink-dim: #93a4ae;
    --ink-faint: #6b7a85;
    --accent: #4dd0e1;
    --path: #f0c24b;
    --rgg-green: #3fb96a;
    --rgg-gray: #6b7a85;
    --rgg-red: #e0523f;
    --edge-valid: rgba(63, 185, 106, 0.30);
    --edge-gray: rgba(150, 166, 176, 0.55);
    --edge-red: rgba(224, 82, 63, 0.95);
    --env-fill: rgba(77, 208, 225, 0.10);
    --env-line: rgba(77, 208, 225, 0.55);
    --mono: ui-monospace, "SF Mono", SFMono-Regular, Menlo, Consolas, monospace;
    --sans: system-ui, -apple-system, "Segoe UI", sans-serif;
  }
  @media (prefers-color-scheme: light) {
    :root {
      --ground: #f2f5f6; --panel: #ffffff; --panel-2: #eef2f4; --line: #d4dde2;
      --ink: #16222b; --ink-dim: #4b5c67; --ink-faint: #7b8b95;
      --accent: #0d7f91; --path: #b07d09;
      --edge-valid: rgba(46, 138, 78, 0.28);
      --edge-gray: rgba(90, 108, 119, 0.50);
      --edge-red: rgba(198, 57, 39, 0.95);
      --env-fill: rgba(13, 127, 145, 0.10);
      --env-line: rgba(13, 127, 145, 0.55);
    }
  }
  :root[data-theme="dark"] {
    --ground: #0b1014; --panel: #141b21; --panel-2: #1b242c; --line: #26333c;
    --ink: #dde5e9; --ink-dim: #93a4ae; --ink-faint: #6b7a85;
    --accent: #4dd0e1; --path: #f0c24b;
    --edge-valid: rgba(63, 185, 106, 0.30);
    --edge-gray: rgba(150, 166, 176, 0.55);
    --edge-red: rgba(224, 82, 63, 0.95);
    --env-fill: rgba(77, 208, 225, 0.10);
    --env-line: rgba(77, 208, 225, 0.55);
  }
  :root[data-theme="light"] {
    --ground: #f2f5f6; --panel: #ffffff; --panel-2: #eef2f4; --line: #d4dde2;
    --ink: #16222b; --ink-dim: #4b5c67; --ink-faint: #7b8b95;
    --accent: #0d7f91; --path: #b07d09;
    --edge-valid: rgba(46, 138, 78, 0.28);
    --edge-gray: rgba(90, 108, 119, 0.50);
    --edge-red: rgba(198, 57, 39, 0.95);
    --env-fill: rgba(13, 127, 145, 0.10);
    --env-line: rgba(13, 127, 145, 0.55);
  }

  body {
    margin: 0; background: var(--ground); color: var(--ink);
    font-family: var(--sans); line-height: 1.5;
  }
  .wrap { max-width: 1180px; margin: 0 auto; padding: 28px 20px 40px; }

  header { display: flex; flex-wrap: wrap; gap: 16px 28px; align-items: baseline;
           border-bottom: 1px solid var(--line); padding-bottom: 16px; }
  h1 { font-size: 19px; font-weight: 600; margin: 0; letter-spacing: -0.01em; }
  .sub { color: var(--ink-dim); font-size: 13px; margin: 0; }
  .meta { margin-left: auto; font-family: var(--mono); font-size: 12px;
          color: var(--ink-faint); display: flex; gap: 18px; flex-wrap: wrap; }
  .meta b { color: var(--ink-dim); font-weight: 500; }

  .stage { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; margin-top: 20px; }
  @media (max-width: 860px) { .stage { grid-template-columns: 1fr; } }

  .panel { background: var(--panel); border: 1px solid var(--line);
           border-radius: 6px; overflow: hidden; transition: border-color .18s; }
  .panel.flash { border-color: var(--accent); }
  .phead { display: flex; align-items: center; gap: 10px; padding: 10px 14px;
           border-bottom: 1px solid var(--line); background: var(--panel-2); }
  .pname { font-size: 13px; font-weight: 600; letter-spacing: 0.02em; }
  .badge { font-family: var(--mono); font-size: 10.5px; letter-spacing: .06em;
           text-transform: uppercase; padding: 2px 7px; border-radius: 3px;
           border: 1px solid var(--line); color: var(--ink-faint); }
  .badge.on { color: var(--ground); background: var(--accent);
              border-color: var(--accent); }
  canvas { display: block; width: 100%; height: 330px; }

  .readout { display: grid; grid-template-columns: repeat(4, 1fr);
             border-top: 1px solid var(--line); font-family: var(--mono);
             font-variant-numeric: tabular-nums; }
  .cell { padding: 9px 12px; border-right: 1px solid var(--line); }
  .cell:last-child { border-right: 0; }
  .k { font-size: 10px; letter-spacing: .05em; text-transform: uppercase;
       color: var(--ink-faint); }
  .v { font-size: 17px; margin-top: 2px; }
  .v.hero { color: var(--accent); font-weight: 600; }
  .v.red { color: var(--rgg-red); }

  .controls { display: flex; align-items: center; gap: 14px; margin-top: 18px;
              padding: 12px 14px; background: var(--panel);
              border: 1px solid var(--line); border-radius: 6px; }
  button { font: inherit; font-size: 13px; color: var(--ink);
           background: var(--panel-2); border: 1px solid var(--line);
           padding: 6px 14px; border-radius: 4px; cursor: pointer; }
  button:hover { border-color: var(--accent); }
  button:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
  .track { position: relative; flex: 1; height: 26px; }
  input[type=range] { width: 100%; margin: 0; accent-color: var(--accent);
                      position: relative; z-index: 2; }
  .ticks { position: absolute; inset: 0 0 auto; height: 8px; top: -2px;
           pointer-events: none; }
  .tick { position: absolute; width: 2px; height: 8px; background: var(--accent);
          border-radius: 1px; }
  .clock { font-family: var(--mono); font-size: 13px; color: var(--ink-dim);
           font-variant-numeric: tabular-nums; min-width: 62px; }

  .legend { display: flex; flex-wrap: wrap; gap: 18px; margin-top: 14px;
            font-size: 12px; color: var(--ink-dim); }
  .swatch { display: inline-block; width: 22px; height: 3px; border-radius: 2px;
            margin-right: 7px; vertical-align: middle; }
  .note { margin-top: 20px; padding-top: 14px; border-top: 1px solid var(--line);
          font-size: 12.5px; color: var(--ink-faint); max-width: 74ch; }
  .note b { color: var(--ink-dim); font-weight: 500; }
</style>

<div class="wrap">
  <header>
    <div>
      <h1>Span-based validity certification</h1>
      <p class="sub">Identical obstacle, identical roadmap — frame-by-frame versus span-amortized.</p>
    </div>
    <div class="meta" id="meta"></div>
  </header>

  <div class="stage" id="stage"></div>

  <div class="controls">
    <button id="play">Pause</button>
    <div class="track">
      <div class="ticks" id="ticks"></div>
      <input type="range" id="scrub" min="0" value="0" step="1" aria-label="Timeline" />
    </div>
    <span class="clock" id="clock">t 0.0s</span>
  </div>

  <div class="legend">
    <span><i class="swatch" style="background:var(--edge-red)"></i>Red — certified invalid</span>
    <span><i class="swatch" style="background:var(--edge-gray)"></i>Gray — undecided</span>
    <span><i class="swatch" style="background:var(--edge-valid)"></i>Green — certified valid</span>
    <span><i class="swatch" style="background:var(--path)"></i>Active path</span>
    <span><i class="swatch" style="background:var(--env-line)"></i>Span envelope</span>
  </div>

  <p class="note">
    Both panels run the same code against the same roadmap and the same obstacle motion;
    only the update policy differs. <b>Geometry passes</b> counts how many times the
    expensive path — build obstacle approximations, query both AABB trees — actually ran.
    The span panel holds its frozen envelope until the obstacle leaves it or the horizon
    lapses, then rebuilds (panel border flashes). Drag a viewport to rotate.
  </p>
</div>

<script id="viewer-data" type="application/json">__DATA__</script>
<script>
(() => {
  const DATA = JSON.parse(document.getElementById('viewer-data').textContent);
  const V = DATA.roadmap.vertices, E = DATA.roadmap.edges;
  const nFrames = Math.min(...DATA.modes.map(m => m.frames.length));

  document.getElementById('meta').innerHTML = [
    ['roadmap', DATA.roadmap.name],
    ['vertices', DATA.roadmap.totalVertices.toLocaleString()],
    ['edges', DATA.roadmap.totalEdges.toLocaleString()],
    ['drawn', E.length.toLocaleString()],
    ['dof', DATA.roadmap.dof || 6]
  ].map(([k, v]) => `<span><b>${k}</b> ${v}</span>`).join('');

  // ---- world bounds -> fit projection
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const p of V) for (let a = 0; a < 3; a++) {
    if (p[a] < lo[a]) lo[a] = p[a];
    if (p[a] > hi[a]) hi[a] = p[a];
  }
  const mid = lo.map((l, i) => (l + hi[i]) / 2);
  const span = Math.max(...hi.map((h, i) => h - lo[i])) || 1;

  let az = -0.62, el = 0.42;   // radians

  function makeViewport(mode) {
    const wrap = document.createElement('div');
    wrap.className = 'panel';
    const isSpan = mode.name.startsWith('spans');
    wrap.innerHTML = `
      <div class="phead">
        <span class="pname">${isSpan ? 'Span-amortized' : 'Frame-by-frame'}</span>
        <span class="badge">${mode.name}</span>
        <span class="badge rebuild" style="margin-left:auto">idle</span>
      </div>
      <canvas></canvas>
      <div class="readout">
        <div class="cell"><div class="k">Geometry passes</div><div class="v hero" data-f="passes">0</div></div>
        <div class="cell"><div class="k">Update</div><div class="v" data-f="ms">—</div></div>
        <div class="cell"><div class="k">Red edges</div><div class="v red" data-f="red">0</div></div>
        <div class="cell"><div class="k">Replans</div><div class="v" data-f="replans">0</div></div>
      </div>`;
    const cv = wrap.querySelector('canvas');
    const ctx = cv.getContext('2d');
    let proj = [];

    function resize() {
      const r = cv.getBoundingClientRect(), dpr = devicePixelRatio || 1;
      cv.width = r.width * dpr; cv.height = r.height * dpr;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      project();
    }
    function project() {
      const r = cv.getBoundingClientRect();
      const s = Math.min(r.width, r.height) / (span * 1.5);
      const ca = Math.cos(az), sa = Math.sin(az);
      const ce = Math.cos(el), se = Math.sin(el);
      proj = V.map(p => {
        const x = p[0] - mid[0], y = p[1] - mid[1], z = p[2] - mid[2];
        const rx = x * ca - y * sa, ry = x * sa + y * ca;
        return [r.width / 2 + rx * s, r.height / 2 - (z * ce - ry * se) * s];
      });
    }
    function pt(p) {   // project a raw world point
      const r = cv.getBoundingClientRect();
      const s = Math.min(r.width, r.height) / (span * 1.5);
      const ca = Math.cos(az), sa = Math.sin(az);
      const ce = Math.cos(el), se = Math.sin(el);
      const x = p[0] - mid[0], y = p[1] - mid[1], z = p[2] - mid[2];
      const rx = x * ca - y * sa, ry = x * sa + y * ca;
      return [r.width / 2 + rx * s, r.height / 2 - (z * ce - ry * se) * s];
    }
    function boxRect(c, h) {
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
        const [px, py] = pt([c[0] + sx * h[0], c[1] + sy * h[1], c[2] + sz * h[2]]);
        x0 = Math.min(x0, px); y0 = Math.min(y0, py);
        x1 = Math.max(x1, px); y1 = Math.max(y1, py);
      }
      return [x0, y0, x1 - x0, y1 - y0];
    }

    function draw(i) {
      const f = mode.frames[i];
      const r = cv.getBoundingClientRect();
      ctx.clearRect(0, 0, r.width, r.height);

      const red = new Set(f.red), gray = new Set(f.gray);
      const css = getComputedStyle(document.documentElement);
      const strokeAll = (filter, color, width) => {
        ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
        for (let e = 0; e < E.length; e++) {
          if (!filter(e)) continue;
          const a = proj[E[e][0]], b = proj[E[e][1]];
          ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
        }
        ctx.stroke();
      };
      strokeAll(e => !red.has(e) && !gray.has(e), css.getPropertyValue('--edge-valid'), 0.5);
      strokeAll(e => gray.has(e), css.getPropertyValue('--edge-gray'), 0.7);
      strokeAll(e => red.has(e), css.getPropertyValue('--edge-red'), 1.3);

      // span envelope (frozen prediction)
      if (f.env) {
        ctx.fillStyle = css.getPropertyValue('--env-fill');
        ctx.strokeStyle = css.getPropertyValue('--env-line');
        ctx.lineWidth = 1;
        for (const s of f.env) {
          const [x, y, w, h] = boxRect(s.c, s.h);
          ctx.fillRect(x, y, w, h); ctx.strokeRect(x, y, w, h);
        }
      }

      // active path
      if (f.path && f.path.length > 1) {
        ctx.strokeStyle = css.getPropertyValue('--path');
        ctx.lineWidth = 2.4; ctx.lineJoin = 'round'; ctx.beginPath();
        f.path.forEach((v, k) => {
          const p = proj[v];
          k ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]);
        });
        ctx.stroke();
      }

      // obstacle
      const oh = DATA.obstacleHalf;
      const [ox, oy, ow, ohh] = boxRect(f.obstacle, [oh, oh, oh]);
      ctx.fillStyle = css.getPropertyValue('--rgg-red');
      ctx.globalAlpha = 0.9; ctx.fillRect(ox, oy, ow, ohh); ctx.globalAlpha = 1;

      // readout
      let passes = 0, replans = 0;
      for (let k = 0; k <= i; k++) {
        if (mode.frames[k].rebuilt) passes++;
        if (mode.frames[k].replanned) replans++;
      }
      const set = (name, val) => wrap.querySelector(`[data-f="${name}"]`).textContent = val;
      set('passes', passes);
      set('ms', f.ms < 0.01 ? '~0 ms' : f.ms.toFixed(1) + ' ms');
      set('red', f.red.length);
      set('replans', replans);
      const badge = wrap.querySelector('.rebuild');
      badge.textContent = f.rebuilt ? 'rebuild' : 'conforming';
      badge.classList.toggle('on', !!f.rebuilt);
      wrap.classList.toggle('flash', !!f.rebuilt);
    }

    // drag to rotate
    let dragging = false, lx = 0, ly = 0;
    cv.addEventListener('pointerdown', e => {
      dragging = true; lx = e.clientX; ly = e.clientY; cv.setPointerCapture(e.pointerId);
    });
    cv.addEventListener('pointermove', e => {
      if (!dragging) return;
      az += (e.clientX - lx) * 0.008;
      el = Math.max(-1.4, Math.min(1.4, el + (e.clientY - ly) * 0.006));
      lx = e.clientX; ly = e.clientY;
      viewports.forEach(v => { v.project(); v.draw(frame); });
    });
    cv.addEventListener('pointerup', () => dragging = false);

    return { el: wrap, resize, project, draw };
  }

  const stage = document.getElementById('stage');
  const viewports = DATA.modes.map(m => {
    const vp = makeViewport(m);
    stage.appendChild(vp.el);
    return vp;
  });

  // ---- timeline
  const scrub = document.getElementById('scrub');
  const clock = document.getElementById('clock');
  const playBtn = document.getElementById('play');
  scrub.max = nFrames - 1;

  const spanMode = DATA.modes.find(m => m.name.startsWith('spans')) || DATA.modes[0];
  document.getElementById('ticks').innerHTML = spanMode.frames
    .map((f, i) => f.rebuilt ? `<i class="tick" style="left:${(i / (nFrames - 1)) * 100}%"></i>` : '')
    .join('');

  let frame = 0, playing = !matchMedia('(prefers-reduced-motion: reduce)').matches;
  function show(i) {
    frame = i;
    scrub.value = i;
    clock.textContent = 't ' + DATA.modes[0].frames[i].t.toFixed(1) + 's';
    viewports.forEach(v => v.draw(i));
  }
  scrub.addEventListener('input', () => { playing = false; playBtn.textContent = 'Play'; show(+scrub.value); });
  playBtn.addEventListener('click', () => {
    playing = !playing;
    playBtn.textContent = playing ? 'Pause' : 'Play';
  });

  let last = 0;
  function tick(now) {
    if (playing && now - last > 110) {
      last = now;
      show((frame + 1) % nFrames);
    }
    requestAnimationFrame(tick);
  }

  addEventListener('resize', () => { viewports.forEach(v => v.resize()); show(frame); });
  viewports.forEach(v => v.resize());
  show(0);
  requestAnimationFrame(tick);
})();
</script>
"""


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    data = Path(sys.argv[1]).read_text()
    json.loads(data)  # validate
    out = Path(sys.argv[2])
    out.write_text(TEMPLATE.replace("__DATA__", data))
    print(f"wrote {out} ({out.stat().st_size / 1024:.0f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
