import os
from pathlib import Path

TXT_SUFFIX = ".txt"


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


def is_text_file(file_path):
    return file_path.is_file() and file_path.suffix.lower() == TXT_SUFFIX


def find_usb_text_file():
    roots = mounted_media_dirs()

    if not roots:
        print("No mounted media directories found.")
        print("Checked: /media/$USER, /run/media/$USER, /media, /mnt")
        return None

    print("Searching mounted media directories:")
    for root in roots:
        print(f"  - {root}")

    text_files = []
    for root in roots:
        try:
            text_files.extend(file_path for file_path in root.rglob("*") if is_text_file(file_path))
        except OSError as exc:
            print(f"Could not search {root}: {exc}")

    for file_path in sorted(text_files):
        return file_path

    return None


def read_usb_file():
    usb_file = find_usb_text_file()

    # Case 1: Found on USB
    if usb_file is not None:
        try:
            contents = usb_file.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            print(f"Found text file but could not read it: {usb_file}")
            print(f"Read error: {exc}")
            return None

        print(f"Found text file: {usb_file}")
        print("----- FILE CONTENTS START -----")
        print(contents)
        print("----- FILE CONTENTS END -----")
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
            try:
                contents = file_path.read_text(encoding="utf-8", errors="replace")
            except OSError as exc:
                print(f"Found fallback file but could not read it: {file_path}")
                print(f"Read error: {exc}")
                return None

            print(f"Found fallback file: {file_path}")
            print("----- FILE CONTENTS START -----")
            print(contents)
            print("----- FILE CONTENTS END -----")
            return contents

    # Case 3: nothing found
    print("No .txt file found on mounted USB media.")
    return None


def main():
    read_usb_file()


if __name__ == "__main__":
    main()
