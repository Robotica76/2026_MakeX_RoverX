// Service worker mínimo: agrega las cabeceras de "aislamiento de origen cruzado" (COOP/COEP)
// que GitHub Pages no permite configurar. Sin ellas el navegador no deja usar
// SharedArrayBuffer, con el que la página le pasa los botones y el control al simulador.
self.addEventListener("install", () => self.skipWaiting());
self.addEventListener("activate", (e) => e.waitUntil(self.clients.claim()));
self.addEventListener("fetch", (e) => {
  const r = e.request;
  if (r.cache === "only-if-cached" && r.mode !== "same-origin") return;
  e.respondWith(
    fetch(r).then((res) => {
      if (res.status === 0) return res;          // respuesta opaca (CDN): se usa tal cual
      const h = new Headers(res.headers);
      h.set("Cross-Origin-Embedder-Policy", "require-corp");
      h.set("Cross-Origin-Opener-Policy", "same-origin");
      h.set("Cross-Origin-Resource-Policy", "cross-origin");
      return new Response(res.body, { status: res.status, statusText: res.statusText, headers: h });
    })
  );
});
