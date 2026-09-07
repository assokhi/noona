import { defineConfig } from "vite";

export default defineConfig({
  server: {
    // The API runs separately; proxying keeps the browser on one origin so the
    // page stays a secure context for the Geolocation API.
    proxy: {
      "/v1": "http://127.0.0.1:8080",
      "/healthz": "http://127.0.0.1:8080",
    },
  },
  build: { target: "es2022" },
});
