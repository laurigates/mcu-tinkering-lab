#!/usr/bin/env node
// Drive the JLCONE desktop app over the Chrome DevTools Protocol.
// JLCONE must be started with the debug port (see SKILL.md § 2):
//   open -a JLCONE --args --remote-debugging-port=9222
// Zero dependencies: Node 22+ has a global WebSocket.
//
//   cdp.mjs targets                         list page targets
//   cdp.mjs eval   <match> <js | @file.js>  evaluate in the page, print JSON
//   cdp.mjs nav    <match> <url>            navigate and wait for load
//   cdp.mjs upload <match> <css>[@N] <file>...  set files on an <input type=file>
//   cdp.mjs shot   <match> <out.png>        screenshot the page
//   cdp.mjs net    <match> <seconds> [re]   log API requests and responses
//
// <match> is a substring of the target URL (e.g. "jlcpcb.com/quote").
// The first page target whose URL contains it is used.

import { readFileSync, writeFileSync } from 'node:fs';
import { resolve } from 'node:path';

const PORT = process.env.CDP_PORT ?? '9222';
const die = (msg) => { console.error(msg); process.exit(1); };

async function targets() {
  const res = await fetch(`http://localhost:${PORT}/json`).catch(() =>
    die(`no CDP on :${PORT}; start JLCONE with --remote-debugging-port=${PORT}`));
  return (await res.json()).filter((t) => t.type === 'page' || t.type === 'webview');
}

async function connect(match) {
  const t = (await targets()).find((t) => t.url.includes(match));
  if (!t) die(`no target matching "${match}"; run: cdp.mjs targets`);
  const ws = new WebSocket(t.webSocketDebuggerUrl);
  await new Promise((ok, fail) => { ws.onopen = ok; ws.onerror = fail; });
  let id = 0;
  const pending = new Map();
  const listeners = [];
  ws.onmessage = ({ data }) => {
    const m = JSON.parse(data);
    if (m.id && pending.has(m.id)) {
      const { ok, fail } = pending.get(m.id);
      pending.delete(m.id);
      m.error ? fail(new Error(`${m.error.message} ${m.error.data ?? ''}`)) : ok(m.result);
    } else if (m.method) listeners.forEach((f) => f(m));
  };
  const send = (method, params = {}) => new Promise((ok, fail) => {
    pending.set(++id, { ok, fail });
    ws.send(JSON.stringify({ id, method, params }));
  });
  return { send, on: (f) => listeners.push(f), close: () => ws.close(), url: t.url };
}

async function evaluate(c, expression) {
  const r = await c.send('Runtime.evaluate', {
    expression, awaitPromise: true, returnByValue: true, userGesture: true,
  });
  if (r.exceptionDetails) die(r.exceptionDetails.exception?.description ?? r.exceptionDetails.text);
  return r.result.value;
}

const [cmd, match, ...rest] = process.argv.slice(2);

switch (cmd) {
  case 'targets':
    for (const t of await targets()) console.log(`${t.type}\t${t.url}`);
    break;
  case 'eval': {
    const c = await connect(match);
    const src = rest[0]?.startsWith('@') ? readFileSync(rest[0].slice(1), 'utf8') : rest.join(' ');
    console.log(JSON.stringify(await evaluate(c, src), null, 2));
    c.close();
    break;
  }
  case 'nav': {
    const c = await connect(match);
    await c.send('Page.enable');
    const loaded = new Promise((ok) => c.on((m) => m.method === 'Page.loadEventFired' && ok()));
    await c.send('Page.navigate', { url: rest[0] });
    await loaded;
    c.close();
    break;
  }
  case 'upload': {
    const [selector, ...files] = rest;
    if (!files.length) die('usage: cdp.mjs upload <match> <css> <file>...');
    const c = await connect(match);
    // "<css>@N" picks the Nth match (0-based), for pages with several inputs.
    const [, css, nth = '0'] = selector.match(/^(.*?)(?:@(\d+))?$/);
    const { root } = await c.send('DOM.getDocument', { depth: 0 });
    const { nodeIds } = await c.send('DOM.querySelectorAll', { nodeId: root.nodeId, selector: css });
    const nodeId = nodeIds[Number(nth)];
    if (!nodeId) die(`${nodeIds.length} element(s) match ${css}; no index ${nth}`);
    await c.send('DOM.setFileInputFiles', { nodeId, files: files.map((f) => resolve(f)) });
    console.log(`set ${files.length} file(s) on ${selector}`);
    c.close();
    break;
  }
  case 'shot': {
    const c = await connect(match);
    const { data } = await c.send('Page.captureScreenshot', { format: 'png' });
    writeFileSync(rest[0], Buffer.from(data, 'base64'));
    console.log(rest[0]);
    c.close();
    break;
  }
  case 'net': {
    const [seconds = '30', pattern = '/api/'] = rest;
    const re = new RegExp(pattern);
    const c = await connect(match);
    await c.send('Network.enable');
    const reqs = new Map();
    c.on(async (m) => {
      if (m.method === 'Network.requestWillBeSent' && re.test(m.params.request.url)) {
        const { method, url, postData } = m.params.request;
        reqs.set(m.params.requestId, url);
        console.log(`>> ${method} ${url}${postData ? `\n   ${postData.slice(0, 400)}` : ''}`);
      }
      if (m.method === 'Network.loadingFinished' && reqs.has(m.params.requestId)) {
        const body = await c.send('Network.getResponseBody', { requestId: m.params.requestId })
          .catch(() => ({ body: '(unavailable)' }));
        console.log(`<< ${reqs.get(m.params.requestId)}\n   ${body.body.slice(0, 400)}`);
      }
    });
    await new Promise((ok) => setTimeout(ok, Number(seconds) * 1000));
    c.close();
    break;
  }
  default:
    die('usage: cdp.mjs targets | eval | nav | upload | shot | net  (see header)');
}
