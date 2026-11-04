import type { Server as HttpServer } from 'node:http';
import { defineConfig, type Plugin, type PreviewServer, type ViteDevServer } from 'vite';
import { attachMockRobot } from './mock/robot.ts';
import { mockConfig } from './mock/sim.ts';

/**
 * Inline the bundled JS and CSS into index.html: the firmware serves a single
 * gzipped page from flash.
 */
function inlineIntoHtml(): Plugin {
  return {
    name: 'inline-into-html',
    apply: 'build',
    enforce: 'post',
    generateBundle(_, bundle) {
      const html = Object.values(bundle).find((f) => f.fileName.endsWith('.html'));
      if (!html || html.type !== 'asset') return;
      let source = String(html.source);
      for (const [name, file] of Object.entries(bundle)) {
        const ref = new RegExp(`<script type="module" crossorigin src="/${name}"></script>|<link rel="stylesheet" crossorigin href="/${name}">`);
        if (!ref.test(source)) continue;
        const code = file.type === 'chunk' ? file.code : String(file.source);
        const tag = file.type === 'chunk' ? `<script type="module">${code.replace(/<\/script/gi, '<\\/script')}</script>` : `<style>${code}</style>`;
        source = source.replace(ref, () => tag);
        delete bundle[name];
      }
      html.source = source;
    },
  };
}

/**
 * Dev and preview servers: serve /c.js like the firmware does. With ROBOT=<ip> the page
 * talks to a real robot, otherwise to the mock robot on this server.
 */
function robotDevServer(): Plugin {
  const setup = (server: ViteDevServer | PreviewServer) => {
    const robot = process.env.ROBOT;
    if (!robot && server.httpServer) {
      attachMockRobot(server.httpServer as HttpServer, (...a) => server.config.logger.info(a.join(' ')));
    }
    server.middlewares.use('/c.js', (req, res) => {
      const ws = robot ? `ws://${robot}/ws` : `ws://${req.headers.host}/ws`;
      res.setHeader('Content-Type', 'application/javascript');
      // TOF=<n> sets the number of ToF views (dog 1, hexapod 2)
      res.end(`const c=${JSON.stringify(mockConfig(Number(process.env.TOF ?? 2), ws))};`);
    });
  };
  return {
    name: 'robot-dev-server',
    configureServer: setup,
    configurePreviewServer: setup,
  };
}

/** Demo build: no /c.js, the page runs the mock robot itself (src/demo.ts). */
function dropConfigScript(): Plugin {
  return {
    name: 'drop-config-script',
    transformIndexHtml: (html) => html.replace(/\s*<!--[^>]*-->\s*<script src="\/c\.js"[^>]*><\/script>/, ''),
  };
}

// `vite build --mode demo` (npm run build:demo): a page with the mock robot built in, to try the UI
// without a robot or a dev server; dist-demo/index.html
export default defineConfig(({ mode }) => ({
  plugins: [inlineIntoHtml(), robotDevServer(), ...(mode === 'demo' ? [dropConfigScript()] : [])],
  define: { __DEMO__: JSON.stringify(mode === 'demo') },
  server: { host: true },
  build: {
    target: 'es2020',
    modulePreload: false,
    cssCodeSplit: false,
    assetsInlineLimit: Number.MAX_SAFE_INTEGER,
    reportCompressedSize: false,
    outDir: mode === 'demo' ? 'dist-demo' : 'dist',
  },
}));
