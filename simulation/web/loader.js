// Cargador de la versión web (se inserta en <head> de index.html por simulation/build_web.py).
// 1. Activa el aislamiento de origen cruzado (service worker coi-sw.js, una recarga la primera vez).
// 2. Arranca el worker de Python y espera los datos iniciales de la pista.
// 3. Ejecuta los scripts de la página (marcados type="text/sim-deferred") con esos datos.
// 4. Reemplaza las llamadas al servidor de live.py (/state, /btn, /pad, ...) por mensajes al worker.
(function () {
  const params = sessionStorage.getItem("simParams") || "level=1&seed=0&noise=0&mode=comp";
  const overlay = () => document.getElementById("simload");
  const say = (msg, err) => {
    const o = overlay();
    if (!o) return;
    o.querySelector("p").textContent = msg;
    if (err) o.classList.add("err");
  };

  if (!window.crossOriginIsolated) {
    if (!("serviceWorker" in navigator)) {
      addEventListener("DOMContentLoaded", () => say("Este navegador no puede correr el simulador (sin service workers). Usa Chrome, Edge o Firefox recientes.", true));
      return;
    }
    if (sessionStorage.getItem("coiTried")) {
      addEventListener("DOMContentLoaded", () => say("No se pudo activar el modo aislado del navegador. Recarga la página; si sigue igual, usa Chrome o Edge.", true));
      sessionStorage.removeItem("coiTried");
      return;
    }
    sessionStorage.setItem("coiTried", "1");
    navigator.serviceWorker.register("coi-sw.js").then(() => navigator.serviceWorker.ready)
      .then(() => location.reload());
    return;
  }
  sessionStorage.removeItem("coiTried");

  // ---------------------------------------------------------------- canal de órdenes
  const sab = new SharedArrayBuffer(16 + (1 << 16));
  const i32 = new Int32Array(sab, 0, 4), u8 = new Uint8Array(sab), N = sab.byteLength - 16;
  const enc = new TextEncoder();
  function send(obj) {
    const b = enc.encode(JSON.stringify(obj) + "\n");
    const w = Atomics.load(i32, 0);
    for (let k = 0; k < b.length; k++) u8[16 + ((w + k) % N)] = b[k];
    Atomics.store(i32, 0, w + b.length);
    Atomics.add(i32, 2, 1);
    Atomics.notify(i32, 2);
  }

  // ---------------------------------------------------------------- estado acumulado
  const store = { frames: [], events: [], status: { state: "", t: 0, speed: 1, paused: false, restarts: 0, falls: 0, screen: ["", "", "", ""] }, judge: [], program: {} };
  const json = (o) => Promise.resolve(new Response(JSON.stringify(o), { headers: { "Content-Type": "application/json" } }));
  const realFetch = window.fetch.bind(window);
  window.fetch = function (url, opts) {
    if (typeof url !== "string" || !url.startsWith("/")) return realFetch(url, opts);
    const u = new URL(url, location.href), q = u.searchParams, path = u.pathname.replace(/^.*\//, "");
    if (path === "state") {
      const f = +q.get("f") || 0, e = +q.get("e") || 0;
      return json({ frames: store.frames.slice(f), events: store.events.slice(e), status: store.status, judge: store.judge, program: store.program });
    }
    if (path === "btn") send({ c: "btn", k: q.get("k") });
    else if (path === "place") send({ c: "place" });
    else if (path === "pause") send({ c: "pause" });
    else if (path === "speed") send({ c: "speed", v: +q.get("v") || 1 });
    else if (path === "pad") {
      let st = {};
      try { st = JSON.parse((opts && opts.body) || "{}"); } catch (err) { st = {}; }
      send(Object.assign({ c: "pad" }, st));
    } else if (path === "new") {
      sessionStorage.setItem("simParams", u.search.slice(1));   // la página recarga y arranca así
      return json({ reload: true });
    } else if (path === "upload") {
      return json({ error: "En la versión web todavía no se pueden subir programas. Usa simulation/live.py en tu computadora." });
    }
    return json({ ok: true });
  };

  // ---------------------------------------------------------------- worker de Python
  const worker = new Worker("sim-worker.js");
  const qp = Object.fromEntries(new URLSearchParams(params));
  worker.postMessage({ type: "init", sab, params: JSON.stringify(qp) });
  worker.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.type === "progress") say(m.msg);
    else if (m.type === "error") say("Error al cargar el simulador: " + m.msg, true);
    else if (m.type === "init") start(m.data);
    else if (m.type === "snap") {
      const s = m.s;
      for (const f of s.frames) store.frames.push(f);
      for (const ev of s.events) store.events.push(ev);
      store.status = s.status; store.judge = s.judge; store.program = s.program;
    }
  };
  worker.onerror = (e) => say("Error en el simulador: " + (e.message || e), true);

  function start(data) {
    window.__SIM_DATA__ = data;
    const run = () => {
      for (const old of document.querySelectorAll('script[type="text/sim-deferred"]')) {
        const s = document.createElement("script");
        s.textContent = old.textContent;
        old.replaceWith(s);
      }
      const o = overlay();
      if (o) o.remove();
    };
    if (document.readyState === "loading") addEventListener("DOMContentLoaded", run); else run();
  }
})();
