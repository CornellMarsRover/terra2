# Mock-rover assets

`sample_video.h264` is the pre-encoded H.264 Annex-B bytestream the mock
camera replayer loops over. **It is not checked into the repo** — generate
it once with:

```bash
./scripts/fetch_sample_video.sh
```

from the repo root. The script uses `ffmpeg`'s built-in `testsrc` source
(no network required) to produce a 10 s, 640×480, ~30 fps test pattern as
a raw H.264 bytestream with 4-byte start codes.

## Why this path lives inside the Python package dir

The replayer resolves the asset via `Path(__file__).resolve().parent.parent
/ "assets" / "sample_video.h264"`. When installed with
`colcon build --symlink-install`, `__file__` points at the real source
tree (under `src/urc_mock_rover/urc_mock_rover/drivers/camera_replayer.py`),
so the asset is found next to the source. No `data_files` entry is needed.

## Regenerate

Re-run the fetch script any time you want a fresh file — for example if
you need a different resolution, framerate, or duration. Edit the
`ffmpeg` args in `scripts/fetch_sample_video.sh` and re-run.

## Why not commit the file

- H.264 binaries aren't review-friendly.
- `testsrc` is deterministic; every dev gets byte-identical output.
- Keeps repo size stable.
