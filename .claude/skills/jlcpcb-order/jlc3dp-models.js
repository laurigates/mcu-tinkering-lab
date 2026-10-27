// List the models on a JLC3DP quote page with their size, volume and price.
//   node cdp.mjs eval jlc3dp @jlc3dp-models.js
// Reads each model's own row: in the page's flattened text a row's
// "Detail" sizes follow the next row's file name, so a regex over
// document.body.innerText pairs files with the wrong volumes.
(() => {
  const names = [...document.querySelectorAll('*')].filter(
    (e) => e.children.length === 0 && /\.(stl|step|stp|obj|3mf)$/i.test(e.innerText?.trim() ?? ''));
  const rows = names.map((n) => {
    let row = n;
    while (row.parentElement && !/Volume:/.test(row.innerText)) row = row.parentElement;
    const text = row.innerText.replace(/\s+/g, ' ');
    const [, size, volume] = text.match(/([\d.]+×[\d.]+×[\d.]+) cm, Volume: ([\d.]+) cm³/) ?? [];
    return {
      file: n.innerText.trim(),
      sizeCm: size ?? null,
      volumeCm3: volume ? Number(volume) : null,
      price: text.match(/[€$][\d.]+/)?.[0] ?? null,
    };
  });
  const total = document.body.innerText.match(/Total Price\s*([€$][\d.]+)/)?.[1] ?? null;
  return { models: rows, totalPrice: total };
})()
