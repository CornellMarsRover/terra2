import os
from pathlib import Path

TXT_GLOB = "*.txt"


def mounted_media_dirs():
    user = os.environ.get("USER")
    roots = []

    if user:
        roots.extend([
            Path("/media") / user,
            Path("/run/media") / user,
        ])

    roots.extend([
        Path("/media"),
        Path("/mnt"),
    ])

    return [root for root in roots if root.is_dir()]


def find_usb_text_file():
    for root in mounted_media_dirs():
        for file_path in sorted(root.rglob(TXT_GLOB)):
            if file_path.is_file():
                return file_path
    return None


def read_usb_file():
    usb_file = find_usb_text_file()

    # Case 1: Found on USB
    if usb_file is not None:
        contents = usb_file.read_text()
        print(f"FOUND FILE: {usb_file}")
        print(contents)
        return contents

    # Case 2: fallback locations
    candidates = [
        Path.cwd() / "test.txt",
        Path(os.environ.get("COLCON_PREFIX_PATH", "").split(":")[0]).parent / "text.txt"
        if os.environ.get("COLCON_PREFIX_PATH") else None,
        Path(__file__).resolve().parents[3] / "text.txt",
    ]

    for file_path in candidates:
        if file_path and file_path.is_file():
            contents = file_path.read_text()
            print(f"FOUND FALLBACK FILE: {file_path}")
            print(contents)
            return contents

    # Case 3: nothing found
    print("No .txt file found on mounted USB media")
    return None


def main():
    read_usb_file()


if __name__ == "__main__":
    main()
