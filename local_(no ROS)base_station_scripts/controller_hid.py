import os


DUALSENSE_VENDOR_IDS = {0x054C}
DUALSENSE_NAME_HINTS = ("dualsense", "wireless controller")


def load_hidapi():
    try:
        import hidapi
    except ModuleNotFoundError as exc:
        if exc.name != "hidapi":
            raise
        raise SystemExit(
            "Missing Python dependency: hidapi\n"
            "Install local controller dependencies with:\n"
            "  python3 -m pip install -r requirements.txt"
        ) from exc
    except OSError as exc:
        raise SystemExit(
            "Missing system HID library for Python hidapi.\n"
            "Install it with:\n"
            "  sudo apt install libhidapi-hidraw0 libhidapi-libusb0"
        ) from exc
    return hidapi


def _decode(value):
    if isinstance(value, bytes):
        return value.decode(errors="replace")
    return value


def _field(device, name, default=None):
    if isinstance(device, dict):
        return device.get(name, default)
    return getattr(device, name, default)


def _path_text(device):
    return _decode(_field(device, "path", ""))


def is_dualsense(device):
    vendor_id = _field(device, "vendor_id")
    product = str(_decode(_field(device, "product_string", "")) or "").lower()
    manufacturer = str(_decode(_field(device, "manufacturer_string", "")) or "").lower()

    if vendor_id in DUALSENSE_VENDOR_IDS and any(hint in product for hint in DUALSENSE_NAME_HINTS):
        return True
    return any(hint in product for hint in DUALSENSE_NAME_HINTS) and "sony" in manufacturer


def list_dualsense_devices():
    hidapi = load_hidapi()
    devices = []
    for device in hidapi.enumerate():
        if is_dualsense(device):
            devices.append(device)
    devices.sort(key=_path_text)
    return devices


def describe_device(device):
    path = _path_text(device)
    manufacturer = _decode(_field(device, "manufacturer_string", "")) or "unknown manufacturer"
    product = _decode(_field(device, "product_string", "")) or "unknown product"
    serial = _decode(_field(device, "serial_number", "")) or "no serial"
    vendor_id = _field(device, "vendor_id")
    product_id = _field(device, "product_id")

    ids = ""
    if vendor_id is not None and product_id is not None:
        ids = f" [{int(vendor_id):04x}:{int(product_id):04x}]"

    return f"{path} - {manufacturer} {product}{ids}, serial={serial}"


def print_dualsense_devices():
    devices = list_dualsense_devices()
    if not devices:
        print("No DualSense controllers found by hidapi.")
        return

    for index, device in enumerate(devices):
        print(f"{index}: {describe_device(device)}")


def resolve_controller_path(role, explicit_path=None, serial=None, index=0, legacy_path=None):
    if explicit_path:
        return explicit_path

    devices = list_dualsense_devices()
    if serial:
        for device in devices:
            device_serial = _decode(_field(device, "serial_number", ""))
            if device_serial == serial:
                return _path_text(device)
        known = "\n".join(f"  {describe_device(device)}" for device in devices) or "  none"
        raise RuntimeError(
            f"Could not find {role} controller with serial {serial!r}.\nKnown DualSense devices:\n{known}"
        )

    if 0 <= index < len(devices):
        return _path_text(devices[index])

    if legacy_path and os.path.exists(legacy_path):
        return legacy_path

    known = "\n".join(f"  {i}: {describe_device(device)}" for i, device in enumerate(devices)) or "  none"
    raise RuntimeError(
        f"Could not auto-select {role} controller index {index}.\n"
        f"Known DualSense devices:\n{known}\n"
        "Pass --controller-path /dev/hidrawN, --controller-index N, or --controller-serial SERIAL."
    )


def open_dualsense(pydualsense_cls, role, explicit_path=None, serial=None, index=0, legacy_path=None):
    controller_path = resolve_controller_path(
        role,
        explicit_path=explicit_path,
        serial=serial,
        index=index,
        legacy_path=legacy_path,
    )

    ds = pydualsense_cls()

    def find_device_by_path():
        hidapi = load_hidapi()
        return hidapi.Device(path=os.fsencode(controller_path))

    ds._pydualsense__find_device = find_device_by_path
    try:
        ds.init()
    except OSError as exc:
        script_name = "ee_control.py" if role == "arm" else "drive_control.py"
        raise SystemExit(
            f"Could not open {role} controller at {controller_path}.\n"
            "Linux can see the controller, but this user may not have permission "
            "to open the hidraw device.\n"
            "For a quick test, try:\n"
            f"  sudo python3 {script_name} --controller-path {controller_path}\n"
            "For the normal fix, install the udev rule in this folder's README."
        ) from exc
    print(f"Using {role} controller: {controller_path}")
    return ds
