#!/usr/bin/env python3
"""从 src/shoot_sim/launch 解析参数，生成 release_params.py 供 PyInstaller 打包
与 referee 一致：参数来源为 src，改 src 的 launch 后运行 update_release 即打包进二进制"""
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_LAUNCH_DIR = os.path.join(WS, "src", "shoot_sim", "launch")
OUT_PATH = os.path.join(WS, "src", "shoot_sim", "scripts", "release_params.py")

# release 专用覆盖（与 dev 不同）：包名、gazebo_world_2026_race 传入的 arg
SHOOT_SIM_OVERRIDES = {
    "world_frame": "odom",
    "shoot_frame": "shout_link",
    "shoot_axis": "x",
    "shoot_height": 0.26,
    "spawn_offset": 0.3,
}


def _parse_value(s):
    s = s.strip()
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        pass
    if s.lower() in ("true", "yes"):
        return True
    if s.lower() in ("false", "no"):
        return False
    return s


def _resolve_texture_path(val):
    """$(find shoot_sim)/materials/scripts/xxx.material -> materials/scripts/xxx.material（race 包内相对路径）"""
    if not isinstance(val, str):
        return val
    m = re.search(r"\$\(find\s+\w+\)/(.+)", val)
    if m:
        return m.group(1).strip()
    return val


def main():
    params = {}

    for fname in ("shoot_sim.launch", "shoot_targets.launch"):
        path = os.path.join(SRC_LAUNCH_DIR, fname)
        if not os.path.isfile(path):
            print("警告: 未找到 %s" % path, file=sys.stderr)
            continue
        with open(path, "r", encoding="utf-8") as f:
            content = f.read()
        for m in re.finditer(r'<param\s+name="([^"]+)"\s+value="([^"]*)"', content):
            name, val = m.group(1), m.group(2)
            if name == "shoot_height" and "shoot_sim" in fname:
                continue
            v = _parse_value(val)
            v = _resolve_texture_path(v) if "texture" in name.lower() else v
            params[name] = v
        for m in re.finditer(r'<arg\s+name="([^"]+)"\s+default="([^"]*)"', content):
            name, val = m.group(1), m.group(2)
            if name in params:
                continue
            params[name] = _parse_value(val)

    params["package_name"] = "shoot_sim_race"
    params.update(SHOOT_SIM_OVERRIDES)  # 覆盖 $(arg xxx) 等未解析值

    lines = [
        "# 自动生成，请勿编辑。由 src/shoot_sim/launch 解析生成。",
        "# race 版：参数嵌入二进制，改 src 的 launch 后运行 update_release 即生效。",
        "RELEASE_PARAMS = {",
    ]
    for k, v in sorted(params.items()):
        if isinstance(v, str):
            lines.append('    "%s": %r,' % (k, v))
        elif isinstance(v, (int, float)):
            lines.append('    "%s": %s,' % (k, v))
        elif isinstance(v, bool):
            lines.append('    "%s": %s,' % (k, "True" if v else "False"))
        else:
            lines.append('    "%s": %r,' % (k, v))
    lines.append("}")

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("已生成 %s（从 %s）" % (OUT_PATH, SRC_LAUNCH_DIR))


if __name__ == "__main__":
    main()
