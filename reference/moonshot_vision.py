#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Moonshot/Kimi 任务板视觉识别客户端。"""

import base64
import json
import os
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import requests


MOONSHOT_ENDPOINT = "https://api.moonshot.cn/v1/chat/completions"
DEFAULT_MODEL = "kimi-k2.6"

OBJECT_TO_REGION = {
    "ak47": 1,
    "helmet": 1,
    "pack": 1,
    "aid": 2,
    "gauze": 2,
    "iv": 2,
    "mag": 3,
    "box": 3,
    "belt": 3,
}

SYSTEM_PROMPT = """你是机器人比赛的任务板视觉分类器。
你的唯一任务是识别图片中的主要军用物品，并从以下九个规范名称中选择一个：
ak47, helmet, pack, aid, gauze, iv, mag, box, belt。

分类说明：
- ak47：AK-47 步枪或外形明确的突击步枪
- helmet：军用头盔
- pack：普通黑绿配色战术背包，通常没有红十字标志
- aid：正面带一个或多个醒目红十字的绿色军用医疗包/急救背包
- gauze：纱布卷、绷带或止血带
- iv：输液袋、输液瓶或输液装置
- mag：枪械弹匣
- box：弹药箱、军用收纳箱
- belt：成排连接的弹链

必须只输出一个 JSON 对象，不得输出 Markdown、代码块或额外文字。
JSON 格式必须严格为：
{"object_name":"九个规范名称之一","confidence":0.0,"description":"简短中文描述"}

confidence 必须是 0 到 1 之间的数字。看不清时仍选择最可能的规范名称，但降低 confidence。
特别注意 aid 与 pack 的区别：只要绿色包体正面有醒目的红十字，应判为 aid；
没有红十字、呈普通黑绿战术双肩包外形时，才判为 pack。

忽略墙壁、任务板边框、背景、文字和机器人部件，只判断任务板图片中的主要物品。"""

USER_PROMPT = """识别这张机器人相机图片中的任务板物品。
只返回规定的 JSON 对象。"""


@dataclass(frozen=True)
class RecognitionResult:
    object_name: str
    region: int
    confidence: float
    description: str


class MoonshotVisionError(RuntimeError):
    pass


class MoonshotVisionRecognizer:
    def __init__(
        self,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        timeout: float = 30.0,
        retries: int = 3,
        min_confidence: float = 0.55,
        session=None,
    ):
        self.api_key = api_key or os.environ.get("MOONSHOT_API_KEY", "")
        self.model = model or os.environ.get("MOONSHOT_VISION_MODEL", DEFAULT_MODEL)
        self.timeout = float(timeout)
        self.retries = max(1, int(retries))
        self.min_confidence = float(min_confidence)
        self.session = session or requests.Session()

    @staticmethod
    def _encode_image(cv_image) -> str:
        if cv_image is None or getattr(cv_image, "size", 0) == 0:
            raise MoonshotVisionError("相机图片为空")

        # 限制长边，降低上传耗时和视觉 token；保持任务板细节。
        height, width = cv_image.shape[:2]
        max_side = 1280
        if max(height, width) > max_side:
            scale = max_side / float(max(height, width))
            cv_image = cv2.resize(
                cv_image,
                (int(round(width * scale)), int(round(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        ok, encoded = cv2.imencode(
            ".jpg",
            cv_image,
            [int(cv2.IMWRITE_JPEG_QUALITY), 92],
        )
        if not ok:
            raise MoonshotVisionError("图片 JPEG 编码失败")
        return "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")

    def _payload(self, image_url: str) -> dict:
        return {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": image_url}},
                        {"type": "text", "text": USER_PROMPT},
                    ],
                },
            ],
            "response_format": {"type": "json_object"},
            "thinking": {"type": "disabled"},
            "max_tokens": 256,
        }

    @staticmethod
    def _parse_content(content: str) -> RecognitionResult:
        try:
            data = json.loads(content)
        except (TypeError, json.JSONDecodeError) as exc:
            raise MoonshotVisionError("模型返回的内容不是严格 JSON") from exc

        if not isinstance(data, dict):
            raise MoonshotVisionError("模型 JSON 顶层必须是对象")
        if set(data) != {"object_name", "confidence", "description"}:
            raise MoonshotVisionError("模型 JSON 字段不符合约定")

        object_name = data["object_name"]
        confidence = data["confidence"]
        description = data["description"]
        if not isinstance(object_name, str):
            raise MoonshotVisionError("object_name 必须是字符串")
        object_name = object_name.strip().lower()
        if object_name not in OBJECT_TO_REGION:
            raise MoonshotVisionError("未知物品类别: {}".format(object_name))
        if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
            raise MoonshotVisionError("confidence 必须是数字")
        confidence = float(confidence)
        if not 0.0 <= confidence <= 1.0:
            raise MoonshotVisionError("confidence 必须在 0 到 1 之间")
        if not isinstance(description, str) or not description.strip():
            raise MoonshotVisionError("description 必须是非空字符串")

        return RecognitionResult(
            object_name=object_name,
            region=OBJECT_TO_REGION[object_name],
            confidence=confidence,
            description=description.strip(),
        )

    def recognize(self, cv_image) -> RecognitionResult:
        if not self.api_key:
            raise MoonshotVisionError("未设置环境变量 MOONSHOT_API_KEY")

        image_url = self._encode_image(cv_image)
        payload = self._payload(image_url)
        last_error = None

        for attempt in range(1, self.retries + 1):
            try:
                response = self.session.post(
                    MOONSHOT_ENDPOINT,
                    headers={
                        "Authorization": "Bearer {}".format(self.api_key),
                        "Content-Type": "application/json",
                    },
                    json=payload,
                    timeout=self.timeout,
                )
                if response.status_code != 200:
                    # 不记录响应正文，避免服务端错误信息意外包含敏感数据。
                    raise MoonshotVisionError(
                        "Moonshot API HTTP {}".format(response.status_code)
                    )
                body = response.json()
                content = body["choices"][0]["message"]["content"]
                result = self._parse_content(content)
                if result.confidence < self.min_confidence:
                    raise MoonshotVisionError(
                        "识别置信度过低: {:.2f}".format(result.confidence)
                    )
                return result
            except (
                KeyError,
                IndexError,
                ValueError,
                requests.RequestException,
                MoonshotVisionError,
            ) as exc:
                last_error = exc
                if attempt < self.retries:
                    time.sleep(0.5 * attempt)

        raise MoonshotVisionError(
            "Moonshot 视觉识别失败（已重试 {} 次）: {}".format(
                self.retries,
                last_error,
            )
        )
