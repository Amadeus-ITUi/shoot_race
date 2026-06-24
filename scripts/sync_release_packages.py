#!/usr/bin/env python3
"""从 src 同步到 release_packages，构建时自动执行。参数以 src 为准，打包时嵌入二进制。"""
import os
import shutil

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_REF_LAUNCH = os.path.join(WS, "src", "referee_system", "launch", "referee.launch")
REL_REF_LAUNCH = os.path.join(WS, "release_packages", "referee_system_race", "launch", "referee.launch")
SRC_SHOOT_DIR = os.path.join(WS, "src", "shoot_sim", "launch")
REL_SHOOT_DIR = os.path.join(WS, "release_packages", "shoot_sim_race", "launch")


def sync_referee():
    with open(SRC_REF_LAUNCH, "r", encoding="utf-8") as f:
        content = f.read()
    content = content.replace('pkg="referee_system"', 'pkg="referee_system_race"')
    content = content.replace('type="referee_node.py"', 'type="referee_node"')
    content = content.replace(
        '<node pkg="referee_system_race" type="referee_node" name="referee" output="screen">',
        '<node pkg="referee_system_race" type="referee_node" name="referee" output="screen">\n    <param name="package_name" value="referee_system_race" />'
    )
    os.makedirs(os.path.dirname(REL_REF_LAUNCH), exist_ok=True)
    with open(REL_REF_LAUNCH, "w", encoding="utf-8") as f:
        f.write(content)
    print("已同步 %s -> %s" % (SRC_REF_LAUNCH, REL_REF_LAUNCH))


def sync_shoot_sim():
    """src/shoot_sim/launch -> release_packages/shoot_sim_race/launch，替换 pkg、type、$(find shoot_sim)"""
    os.makedirs(REL_SHOOT_DIR, exist_ok=True)
    for fname in ("shoot_sim.launch", "shoot_targets.launch"):
        src = os.path.join(SRC_SHOOT_DIR, fname)
        dst = os.path.join(REL_SHOOT_DIR, fname)
        if not os.path.isfile(src):
            print("警告: 未找到 %s" % src)
            continue
        with open(src, "r", encoding="utf-8") as f:
            content = f.read()
        content = content.replace('pkg="shoot_sim"', 'pkg="shoot_sim_race"')
        content = content.replace('type="shoot_sim_node.py"', 'type="shoot_sim_node"')
        content = content.replace('type="shoot_targets_node.py"', 'type="shoot_targets_node"')
        content = content.replace("$(find shoot_sim)", "$(find shoot_sim_race)")
        with open(dst, "w", encoding="utf-8") as f:
            f.write(content)
        print("已同步 %s -> %s" % (src, dst))


def main():
    sync_referee()
    sync_shoot_sim()


if __name__ == "__main__":
    main()
