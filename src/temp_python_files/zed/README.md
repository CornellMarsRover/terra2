# ZED Point Cloud Options

This repo has two explicit files:

- `jetson_zed_sdk_pointcloud.py`: For NVIDIA Jetson/Linux/Windows with ZED SDK (`pyzed.sl`) for true depth + point cloud.
- `mac_opencv_pointcloud.py`: For macOS OpenCV stereo from USB/UVC (approximate depth scale).

## 1) UVC fallback on macOS (no ZED SDK)

Create/activate virtualenv and install deps:

```bash
cd /Users/agupta/Desktop/DepthPerceptionZed
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install opencv-python numpy
```

Run fallback point-cloud app (all args optional):

```bash
python mac_opencv_pointcloud.py
```

Optional example (override defaults when needed):

```bash
python mac_opencv_pointcloud.py --device 0 --width 2560 --height 720 --fps 30 --snapshot-interval 5 --min-z 0.2 --max-z 8
```

Controls:

- `q` or `Esc`: quit
- `p`: save current point cloud snapshot to `captures_uvc/*.ply`

Live diagnostics are printed once per second:

- `valid_z`: pixels with finite depth
- `in_range`: points within `--min-z` and `--max-z` that will be exported

Notes:

- This point cloud is an approximation from stereo disparity (not ZED SDK depth quality).
- For better metric scaling, tune if needed:

```bash
python mac_opencv_pointcloud.py --baseline-m 0.12 --fx 700
```

## 2) Full ZED SDK path (Linux/Windows + NVIDIA)

If you later move to a supported system with CUDA:

```bash
python jetson_zed_sdk_pointcloud.py
```

This uses native `pyzed.sl` depth and `XYZRGBA` point cloud.

Optional example (override defaults when needed):

```bash
python jetson_zed_sdk_pointcloud.py --resolution HD1080 --fps 30 --depth-mode NEURAL --out-dir captures
```

### SSH Remote Viewing (Jetson)

If you are SSH'ed into the Jetson and want to view feed on your laptop without X11, run:

```bash
python jetson_zed_sdk_pointcloud.py --headless --stream-port 8080
```

Then on your laptop browser open:

```text
http://<jetson-ip>:8080/
```

Remote stream now includes:

- Top: `Left | Depth` feed
- Bottom: live `Point Cloud Preview`
