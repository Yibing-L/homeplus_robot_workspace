import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Dev server proxies the console backend; `npm run build` writes the static bundle
// the backend serves (homeplus_console/static).
const backend = process.env.CONSOLE_BACKEND ?? "http://127.0.0.1:8080";

export default defineConfig({
  plugins: [react()],
  build: { outDir: "../homeplus_console/static", emptyOutDir: true },
  server: {
    proxy: {
      "/api": backend,
      "/img": backend,
      "/ws": { target: backend.replace("http", "ws"), ws: true },
    },
  },
});
