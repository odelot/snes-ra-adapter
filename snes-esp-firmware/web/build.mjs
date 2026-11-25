// Builds the page that goes to LittleFS:
//   web/index.html (readable source, edit this one)
//     -> minified (HTML + inline CSS via clean-css + inline JS via terser)
//     -> gzip -9 -> data/index.html.gz
// ESPAsyncWebServer serves /index.html.gz with "Content-Encoding: gzip" when
// /index.html does not exist, so data/ must NOT contain an index.html.
//
// Usage (in snes-esp-firmware/web):  npm install  (once)  then  npm run build
import { readFileSync, writeFileSync, existsSync, rmSync } from 'node:fs';
import { gzipSync, gunzipSync, constants } from 'node:zlib';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { minify } from 'html-minifier-terser';

const here = dirname(fileURLToPath(import.meta.url));
const src = join(here, 'index.html');
const outGz = join(here, '..', 'data', 'index.html.gz');
const stale = join(here, '..', 'data', 'index.html');

const html = readFileSync(src, 'utf8');
const min = await minify(html, {
  collapseWhitespace: true,
  conservativeCollapse: false,
  removeComments: true,
  removeRedundantAttributes: true,
  removeScriptTypeAttributes: true,
  removeStyleLinkTypeAttributes: true,
  useShortDoctype: true,
  minifyCSS: true,
  minifyJS: { compress: { passes: 2 }, mangle: true },   // top-level names are kept
});
const gz = gzipSync(Buffer.from(min, 'utf8'), { level: constants.Z_BEST_COMPRESSION });
if (gunzipSync(gz).toString('utf8') !== min) throw new Error('gzip round trip failed');

writeFileSync(outGz, gz);
if (existsSync(stale)) rmSync(stale);   // it would shadow the .gz

const kb = n => (n / 1024).toFixed(1) + ' KB';
console.log(`index.html ${kb(Buffer.byteLength(html))} -> minified ${kb(Buffer.byteLength(min))} -> gzip ${kb(gz.length)}  (${outGz})`);
