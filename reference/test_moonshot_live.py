#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""使用真实 Moonshot API 检查九张任务板参考图。"""

import os
import sys
import time

import cv2

from moonshot_vision import MoonshotVisionRecognizer, OBJECT_TO_REGION


def main():
    if not os.environ.get("MOONSHOT_API_KEY"):
        print("请先设置 MOONSHOT_API_KEY", file=sys.stderr)
        return 2

    root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "release", "share", "referee_system", "images")
    )
    recognizer = MoonshotVisionRecognizer(retries=2)
    failures = []
    for expected_name, expected_region in OBJECT_TO_REGION.items():
        path = os.path.join(root, expected_name + ".jpg")
        image = cv2.imread(path)
        if image is None:
            failures.append((expected_name, "图片读取失败"))
            continue
        started = time.monotonic()
        try:
            result = recognizer.recognize(image)
            elapsed = time.monotonic() - started
            passed = (
                result.object_name == expected_name
                and result.region == expected_region
            )
            print(
                "{} expected={}/{} actual={}/{} confidence={:.2f} time={:.2f}s {}".format(
                    expected_name,
                    expected_name,
                    expected_region,
                    result.object_name,
                    result.region,
                    result.confidence,
                    elapsed,
                    "PASS" if passed else "FAIL",
                )
            )
            if not passed:
                failures.append((expected_name, result.object_name))
        except Exception as exc:
            failures.append((expected_name, str(exc)))
            print("{} ERROR {}".format(expected_name, exc))

    if failures:
        print("失败项: {}".format(failures))
        return 1
    print("九张参考图全部识别正确")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
