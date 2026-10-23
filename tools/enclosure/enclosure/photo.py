"""Turn product-render plates into product photos with Fibo 2.0 on a ComfyUI box.

    python -m enclosure.photo <project> [--server URL] [--style studio]
        [--only ID ...] [--seed N] [--steps N]

For each view in output/product/product.json: uploads <id>-plate.png,
queues a Fibo 2.0 edit-turbo graph with the view's instruction for the style,
waits, and saves output/product/photos/<id>-<style>-s<seed>.png.

The graph is the lab's reference workflow
(comfyui-nodes lab/scripts/build_moss_fibo2_workflows.py, the `fibo2` skill
there): TextEncodeFibo2 takes the image and emits positive, negative and the
latent; euler/simple, 8 steps, cfg 1. Width and height 0 let Fibo pick its
~1 MP size from the plate's 4:3. **Fibo 2.0 is CC BY-NC 4.0: the photos are
for non-commercial use only.** Stdlib only: no build123d needed to run it.
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.parse
import urllib.request
import uuid
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
UNET = "Fibo/fibo2_edit_turbo_int8_convrot.safetensors"
TE = "qwen/qwen3vl_4b_int8_convrot.safetensors"
VAE = "Flux2/flux2-vae.safetensors"


def graph(image: str, prompt: str, seed: int, steps: int, prefix: str) -> dict:
    return {
        "1": {"class_type": "UNETLoader",
              "inputs": {"unet_name": UNET, "weight_dtype": "default"}},
        "2": {"class_type": "CLIPLoader",
              "inputs": {"clip_name": TE, "type": "fibo2", "device": "default"}},
        "3": {"class_type": "VAELoader", "inputs": {"vae_name": VAE}},
        "4": {"class_type": "LoadImage", "inputs": {"image": image}},
        "5": {"class_type": "TextEncodeFibo2",
              "inputs": {"clip": ["2", 0], "prompt": prompt, "negative_prompt": "",
                         "vae": ["3", 0], "images.image_1": ["4", 0],
                         "width": 0, "height": 0, "batch_size": 1}},
        "6": {"class_type": "KSampler",
              "inputs": {"model": ["1", 0], "positive": ["5", 0],
                         "negative": ["5", 1], "latent_image": ["5", 2],
                         "seed": seed, "steps": steps, "cfg": 1.0,
                         "sampler_name": "euler", "scheduler": "simple",
                         "denoise": 1.0}},
        "7": {"class_type": "VAEDecode", "inputs": {"samples": ["6", 0], "vae": ["3", 0]}},
        "8": {"class_type": "SaveImage",
              "inputs": {"images": ["7", 0], "filename_prefix": prefix}},
    }  # fmt: skip


def _request(server: str, path: str, data: bytes | None = None, headers=None):
    req = urllib.request.Request(server + path, data=data, headers=headers or {})
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.read()


def upload(server: str, path: Path) -> str:
    boundary = uuid.uuid4().hex
    body = b"".join(
        [
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"overwrite\"\r\n\r\ntrue\r\n".encode(),
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"subfolder\"\r\n\r\nmcu-product\r\n".encode(),
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"image\"; filename=\"{path.name}\"\r\n"
            "Content-Type: image/png\r\n\r\n".encode(),
            path.read_bytes(),
            f"\r\n--{boundary}--\r\n".encode(),
        ]
    )  # fmt: skip
    r = json.loads(
        _request(
            server,
            "/upload/image",
            body,
            {"Content-Type": f"multipart/form-data; boundary={boundary}"},
        )
    )
    return f"{r['subfolder']}/{r['name']}" if r.get("subfolder") else r["name"]


def run(server: str, prompt: dict, timeout: float = 600) -> bytes:
    """Queue, wait for the history entry, fetch the one saved image."""
    r = json.loads(_request(server, "/prompt", json.dumps({"prompt": prompt}).encode(),
                            {"Content-Type": "application/json"}))  # fmt: skip
    pid = r["prompt_id"]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        hist = json.loads(_request(server, f"/history/{pid}"))
        if pid in hist:
            entry = hist[pid]
            status = entry.get("status", {})
            if status.get("status_str") == "error":
                raise SystemExit(f"ComfyUI error: {json.dumps(status)[:800]}")
            images = [i for o in entry["outputs"].values() for i in o.get("images", [])]
            if images:
                q = urllib.parse.urlencode(
                    {k: images[0][k] for k in ("filename", "subfolder", "type")}
                )
                return _request(server, f"/view?{q}")
        time.sleep(2)
    raise SystemExit(f"prompt {pid}: no result in {timeout:.0f} s")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("project", help="e.g. packages/robotics/balancebot")
    ap.add_argument("--server", default="http://popos:8188")
    ap.add_argument("--style", default="studio")
    ap.add_argument("--only", nargs="*", default=[], help="view ids")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--steps", type=int, default=8)
    a = ap.parse_args()
    out = REPO_ROOT / a.project / "hardware" / "enclosure" / "output" / "product"
    spec = json.loads((out / "product.json").read_text())
    photos = out / "photos"
    photos.mkdir(exist_ok=True)
    for v in spec["views"]:
        if a.only and v["id"] not in a.only:
            continue
        name = upload(a.server, out / f"{v['id']}-plate.png")
        t = time.monotonic()
        png = run(a.server, graph(name, v["instructions"][a.style], a.seed, a.steps,
                                  f"mcu-product/{v['id']}-{a.style}"))  # fmt: skip
        dest = photos / f"{v['id']}-{a.style}-s{a.seed}.png"
        dest.write_bytes(png)
        print(f"{dest.relative_to(REPO_ROOT)}  {time.monotonic() - t:.1f} s")


if __name__ == "__main__":
    main()
