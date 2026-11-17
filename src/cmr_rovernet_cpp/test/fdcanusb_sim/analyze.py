#!/usr/bin/env python3
"""Compare the CAN frames the Python and C++ drive stacks sent per scenario step."""
import struct, sys
base = sys.argv[1]

def decode(hexs):
    """moteus multiplex write frame -> readable"""
    b = bytes.fromhex(hexs)
    if hexs.lower() == "010000":
        return "STOP"
    out = []; i = 0
    try:
        while i < len(b):
            op = b[i]; i += 1
            if op == 0x50: continue  # nop/pad
            base_, n = op & 0xF0, op & 0x03
            typ = (op >> 2) & 0x03
            if base_ == 0x00:  # write
                if n == 0: n = b[i]; i += 1
                reg = b[i]; i += 1
                size = [1, 2, 4, 4][typ]
                for k in range(n):
                    raw = b[i:i + size]; i += size
                    v = struct.unpack("<f", raw)[0] if typ == 3 else int.from_bytes(raw, "little", signed=True)
                    out.append(f"r{reg + k:02x}={v:.6g}" if typ == 3 else f"r{reg + k:02x}={v}")
            elif base_ == 0x10:  # read
                if n == 0: n = b[i]; i += 1
                reg = b[i]; i += 1
                out.append(f"read{n}@{reg:02x}")
            else:
                out.append(f"?{op:02x}"); break
    except Exception as e:
        out.append(f"<{e}>")
    return " ".join(out)

def load(impl):
    steps = []
    for l in open(f"{base}/out_{impl}/markers.log"):
        t, _, name = l.split(None, 2)
        steps.append((float(t), name.strip()))
    frames = []
    for l in open(f"{base}/out_{impl}/frames.log"):
        p = l.split()
        if len(p) < 5 or p[1] != "can": continue
        frames.append((float(p[0]), int(p[3], 16) & 0x7F, p[4].lower()))
    # cycles: split when a motor id repeats
    cycles = []; cur = {}; t0 = None
    for t, mid, data in frames:
        if mid in cur or (mid == 1 and cur):
            cycles.append((t0, cur)); cur = {}
        if not cur: t0 = t
        cur[mid] = data
    if cur: cycles.append((t0, cur))
    return steps, cycles

def summarize(impl):
    steps, cycles = load(impl)
    res = {}
    for i, (t, name) in enumerate(steps):
        t_end = steps[i + 1][0] if i + 1 < len(steps) else 1e18
        seq = []; first = {}
        for ct, c in cycles:
            if t <= ct < t_end:
                key = tuple(sorted(c.items()))
                if not seq or seq[-1] != key:
                    seq.append(key)
                    first.setdefault(len(seq) - 1, ct - t)
        res[name] = (seq, first)
    # A step window can start with one more cycle of the previous step's final
    # state (worker phase vs. step boundary): drop that carry-over.
    prev_last = None
    for _, name in steps:
        seq, first = res[name]
        full_last = seq[-1] if seq else prev_last
        if seq and prev_last is not None and seq[0] == prev_last:
            seq = seq[1:]
            first = {k - 1: v for k, v in first.items() if k >= 1}
        res[name] = (seq, first)
        prev_last = full_last
    return [n for _, n in steps], res

names, py = summarize("py")
# The step in which the node starts is not comparable (start-up time differs,
# so the steer queries / first stops land before or after the first marker).
names = names[1:]
_, cpp = summarize("cpp")
ok = True
for name in names:
    ps, pf = py[name]; cs, cf = cpp[name]
    # the state carried over from the previous step may or may not be seen once more
    same = ps == cs
    flag = "same" if same else "DIFF"
    if not same:
        # tolerate only a differing number of carried-over/transitional duplicates
        if set(ps) == set(cs) and ps[-1:] == cs[-1:]:
            flag = "same-set(order/transition differs)"
        else:
            ok = False
    print(f"== {name}: {flag}")
    for label, seq, first in (("py ", ps, pf), ("cpp", cs, cf)):
        if same and label == "cpp":
            print("   cpp: identical; first-seen " + ", ".join(f"{first[k]*1000:.0f}ms" for k in sorted(first)))
            continue
        for k, key in enumerate(seq):
            d = dict(key)
            if all(v == "010000" for v in d.values()):
                txt = f"STOP x{len(d)} ids={sorted(d)}"
            else:
                txt = " | ".join(f"{m}: {decode(v)}" for m, v in sorted(d.items()))
            print(f"   {label} +{first[k]*1000:5.0f}ms {txt}")
# Topic-level comparison of the controller adapter / mux outputs.
for topic in ("teleop", "estop", "cmd_vel"):
    seqs = {}
    for impl in ("py", "cpp"):
        seq = []
        for l in open(f"{base}/out_{impl}/markers.log.topics"):
            p = l.split(None, 2)
            if p[1] == topic and (not seq or seq[-1] != p[2]):
                seq.append(p[2])
        seqs[impl] = seq
    same = seqs["py"] == seqs["cpp"]
    print(f"topic /cmd_vel{'' if topic == 'cmd_vel' else '/' + topic}: py {len(seqs['py'])} / cpp "
          f"{len(seqs['cpp'])} distinct consecutive messages: {'same' if same else 'DIFF'}")
    ok = ok and same
print("RESULT:", "ALL STEPS EQUIVALENT" if ok else "DIFFERENCES FOUND")
sys.exit(0 if ok else 1)
