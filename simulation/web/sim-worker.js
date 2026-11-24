// Web Worker: carga Pyodide (Python en el navegador), los archivos .py del robot y del simulador
// (rover_py.json, generado por simulation/build_web.py) y corre simulation/web/live_web.py.
// Las órdenes de la página llegan por un SharedArrayBuffer (anillo de bytes con JSON por línea).
importScripts("https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.js");

let i32, u8, N;
const dec = new TextDecoder();

function readCmds() {
  const w = Atomics.load(i32, 0);
  let r = i32[1];
  if (w === r) return "[]";
  const n = w - r, buf = new Uint8Array(n);
  for (let k = 0; k < n; k++) buf[k] = u8[16 + ((r + k) % N)];
  Atomics.store(i32, 1, w);
  const lines = dec.decode(buf).split("\n").filter((s) => s.trim());
  return "[" + lines.join(",") + "]";
}

// llamada desde Python en cada paso de física: espera ms (o hasta que llegue una orden)
self.simSleep = (ms) => {
  const sig = Atomics.load(i32, 2);
  if (ms >= 0.5) Atomics.wait(i32, 2, sig, ms);
  return readCmds();
};

self.onmessage = async (e) => {
  if (e.data.type !== "init") return;
  const sab = e.data.sab;
  i32 = new Int32Array(sab, 0, 4);
  u8 = new Uint8Array(sab);
  N = sab.byteLength - 16;
  try {
    postMessage(JSON.stringify({ type: "progress", msg: "Cargando Python (Pyodide)…" }));
    const py = await loadPyodide();
    postMessage(JSON.stringify({ type: "progress", msg: "Cargando el programa del robot…" }));
    const files = await (await fetch("rover_py.json", { cache: "no-cache" })).json();
    for (const [path, src] of Object.entries(files)) {
      const full = "/rover/" + path;
      const dir = full.slice(0, full.lastIndexOf("/"));
      py.FS.mkdirTree(dir);
      py.FS.writeFile(full, src);
    }
    py.globals.set("PARAMS", e.data.params || "{}");
    postMessage(JSON.stringify({ type: "progress", msg: "Arrancando el simulador…" }));
    py.runPython(`
import sys
sys.path.insert(0, "/rover/simulation")
sys.path.insert(0, "/rover/simulation/web")
import live_web
live_web.main(PARAMS)
`);
  } catch (err) {
    postMessage(JSON.stringify({ type: "error", msg: String(err && err.message || err) }));
  }
};
