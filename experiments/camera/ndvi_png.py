#!/usr/bin/env python3
"""Label frame (.npz from cam_counter.py) -> false-colour NDVI PNG.

Container python. Usage: ndvi_png.py <out.png> <frame.npz> [<frame.npz> ...]
Several frames are tiled left to right. NDVI by label lookup (PLAN.md):
leaves/other oaks 0.80, target-set bark 0.25, lesion 0.05, ground 0.30,
background (0) no data (dark grey), anything else 0.10. Palette is the usual
red (low) -> yellow -> green (high) NDVI ramp. Lesions get a thin white box.
"""
import sys
import numpy as np
from PIL import Image, ImageDraw


def ndvi_of(lab):
    v = np.full(lab.shape, 0.10)
    v[lab == 1] = 0.30
    v[lab == 2] = 0.80
    v[(lab >= 130) & (lab <= 148)] = 0.25
    v[(lab >= 10) & (lab <= 123)] = 0.05
    v[lab == 0] = np.nan
    return v


def ramp(v):
    # piecewise: 0 red (215,48,39) -> 0.4 yellow (254,224,139) -> 0.9 green (26,152,80)
    stops = np.array([0.0, 0.4, 0.9])
    cols = np.array([[215, 48, 39], [254, 224, 139], [26, 152, 80]], float)
    x = np.clip(np.nan_to_num(v, nan=0.0), 0, 0.9)
    out = np.stack([np.interp(x, stops, cols[:, c]) for c in range(3)], -1)
    out[np.isnan(v)] = (40, 44, 52)
    return out.astype(np.uint8)


def render(npz):
    d = np.load(npz)
    lab = d["labels"]
    img = Image.fromarray(ramp(ndvi_of(lab)))
    dr = ImageDraw.Draw(img)
    for L in np.unique(lab):
        if 10 <= L <= 123:
            ys, xs = np.nonzero(lab == L)
            dr.rectangle([xs.min() - 3, ys.min() - 3, xs.max() + 3, ys.max() + 3], outline=(255, 255, 255))
    dr.text((6, 6), "t=%.1f s" % float(d["t"]), fill=(255, 255, 255))
    return img


def main():
    out, frames = sys.argv[1], sys.argv[2:]
    ims = [render(f) for f in frames]
    W = sum(i.width for i in ims) + 4 * (len(ims) - 1)
    canvas = Image.new("RGB", (W, ims[0].height), (255, 255, 255))
    x = 0
    for i in ims:
        canvas.paste(i, (x, 0)); x += i.width + 4
    canvas.save(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
