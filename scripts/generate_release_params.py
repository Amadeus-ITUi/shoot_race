#!/usr/bin/env python3
"""从 src/referee_system/launch/referee.launch 解析参数，生成 release_params.py 供 PyInstaller 打包"""
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_PATH = os.path.join(WS, "src", "referee_system", "launch", "referee.launch")
OUT_PATH = os.path.join(WS, "src", "referee_system", "scripts", "release_params.py")

# release 专用覆盖（与 dev 不同）
RELEASE_OVERRIDES = {
    "package_name": "referee_system_race",
    "ob_a_map_bumper_topic": "/map_bumper",
    "ob_a_abot_bumper_topic": "/abot_bumper",
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
    return s


def main():
    if not os.path.isfile(LAUNCH_PATH):
        print("错误: 未找到 %s" % LAUNCH_PATH, file=sys.stderr)
        sys.exit(1)
    with open(LAUNCH_PATH, "r", encoding="utf-8") as f:
        content = f.read()
    # 解析 <param name="x" value="y" />
    params = {}
    for m in re.finditer(r'<param\s+name="([^"]+)"\s+value="([^"]*)"', content):
        params[m.group(1)] = _parse_value(m.group(2))
    params.update(RELEASE_OVERRIDES)  # release 专用覆盖

    lines = [
        "# 自动生成，请勿编辑。由 src/referee_system/launch/referee.launch 解析生成。",
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
    print("已生成 %s（从 %s）" % (OUT_PATH, LAUNCH_PATH))


if __name__ == "__main__":
    main()
