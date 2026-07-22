# YOLO Weights

The trained model is intentionally not committed to Git. Put the approved
YOLOv5 checkpoint at:

```text
weights/best_v5.pt
```

The verified development checkpoint had:

```text
filename: best_v5.pt
size:     3,856,751 bytes
sha256:   462f6d8bf76a1a9fcbace5b4f0b4930b0750acb9d399420446350532185ce988
```

You may keep the file elsewhere and set `YOLO_WEIGHTS` instead:

```bash
YOLO_WEIGHTS="$HOME/models/best_v5.pt" yolo-basket-gazebo
```

Do not publish a trained checkpoint until its dataset, model license, and team
distribution policy have been confirmed.
