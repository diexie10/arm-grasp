# -*- coding: utf-8 -*-
"""build_multiscale_dataset.py — 从原始训练数据生成多尺度裁剪数据集。

5个尺度: orig(原图)/half(1/2)/third(1/3)/quarter(1/4)/fifth(1/5)
每个尺度都以木块中心裁剪，OBB标签坐标自动转换。
输出到 yolo_multiscale/ 目录，可直接用于 YOLO 训练。
"""
import os
import cv2
import numpy as np

# === Paths ===
SRC_IMG = r"C:\Users\diexie\Desktop\yolo数据库图片\dataset\images"
SRC_OBB = r"C:\Users\diexie\Desktop\yolo数据库图片\dataset\labels_obb"
SRC_HBOX = r"C:\Users\diexie\Desktop\yolo数据库图片\dataset\labels"
DST = r"C:\Users\diexie\Desktop\arm-grasp\pc\yolo_multiscale"

SCALES = {
    "orig": 1.0,
    "half": 0.5,
    "third": 1.0 / 3,
    "quarter": 0.25,
    "fifth": 0.2,
}

def main():
    os.makedirs(DST, exist_ok=True)

    total = 0
    for split in ["train", "val"]:
        img_dir = os.path.join(SRC_IMG, split)
        imgs = sorted([f for f in os.listdir(img_dir) if f.endswith(".jpg")])
        print(f"{split}: {len(imgs)} source images")

        out_img_dir = os.path.join(DST, split, "images")
        out_lbl_dir = os.path.join(DST, split, "labels")
        os.makedirs(out_img_dir, exist_ok=True)
        os.makedirs(out_lbl_dir, exist_ok=True)

        for name in imgs:
            # Load image
            buf = np.fromfile(os.path.join(img_dir, name), dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                continue
            h, w = img.shape[:2]

            # Load horizontal box label (for center point)
            hbox_path = os.path.join(SRC_HBOX, split, name.replace(".jpg", ".txt"))
            with open(hbox_path, "r") as f:
                parts = f.readline().strip().split()
            xc, yc = float(parts[1]), float(parts[2])

            # Load OBB label (4 corner points, normalized)
            obb_path = os.path.join(SRC_OBB, split, name.replace(".jpg", ".txt"))
            with open(obb_path, "r") as f:
                obb_line = f.readline().strip()
            obb_coords = [float(p) for p in obb_line.split()[1:]]
            pts = np.array(obb_coords).reshape(4, 2)

            for scale_name, scale_frac in SCALES.items():
                if scale_name == "orig":
                    crop_img = img
                    x_off, y_off = 0, 0
                    crop_w, crop_h = w, h
                else:
                    cw = int(w * scale_frac)
                    ch = int(h * scale_frac)
                    cx_px = int(xc * w)
                    cy_px = int(yc * h)
                    x1 = max(0, cx_px - cw // 2)
                    y1 = max(0, cy_px - ch // 2)
                    x2 = min(w, x1 + cw)
                    y2 = min(h, y1 + ch)
                    crop_img = img[y1:y2, x1:x2]
                    x_off, y_off = x1, y1
                    crop_w, crop_h = crop_img.shape[1], crop_img.shape[0]
                    if crop_w == 0 or crop_h == 0:
                        continue

                # Convert OBB to crop coordinates
                pts_px = pts * [w, h]
                pts_px[:, 0] -= x_off
                pts_px[:, 1] -= y_off
                pts_norm = pts_px / [crop_w, crop_h]
                pts_norm = np.clip(pts_norm, 0, 1)

                # Save image
                out_name = f"{scale_name}_{name}"
                ok, buf_out = cv2.imencode(".jpg", crop_img)
                if ok:
                    buf_out.tofile(os.path.join(out_img_dir, out_name))

                # Save label
                label = "0 " + " ".join(f"{p:.6f}" for p in pts_norm.flatten())
                with open(os.path.join(out_lbl_dir, out_name.replace(".jpg", ".txt")), "w") as f:
                    f.write(label + "\n")

                total += 1

    # Write data.yaml
    yaml = f"""path: {DST}
train: train/images
val: val/images
names:
  0: block
"""
    with open(os.path.join(DST, "data.yaml"), "w") as f:
        f.write(yaml)

    print(f"\nDone: {total} images generated ({total // len(SCALES)} per scale)")
    print(f"Dataset: {DST}")

if __name__ == "__main__":
    main()
