#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import unittest

import numpy as np

from moonshot_vision import (
    MoonshotVisionError,
    MoonshotVisionRecognizer,
    OBJECT_TO_REGION,
)


class FakeResponse:
    def __init__(self, content, status_code=200):
        self.status_code = status_code
        self._content = content

    def json(self):
        return {"choices": [{"message": {"content": self._content}}]}


class FakeSession:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def post(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return self.responses.pop(0)


class MoonshotVisionTests(unittest.TestCase):
    def test_all_canonical_names_map_to_valid_regions(self):
        self.assertEqual(set(OBJECT_TO_REGION.values()), {1, 2, 3})
        self.assertEqual(len(OBJECT_TO_REGION), 9)

    def test_strict_json_result(self):
        content = json.dumps(
            {
                "object_name": "helmet",
                "confidence": 0.91,
                "description": "军用头盔",
            }
        )
        session = FakeSession([FakeResponse(content)])
        recognizer = MoonshotVisionRecognizer(
            api_key="test-key",
            retries=1,
            session=session,
        )
        result = recognizer.recognize(np.zeros((32, 32, 3), dtype=np.uint8))
        self.assertEqual(result.object_name, "helmet")
        self.assertEqual(result.region, 1)
        payload = session.calls[0][1]["json"]
        self.assertEqual(payload["response_format"], {"type": "json_object"})
        self.assertEqual(payload["thinking"], {"type": "disabled"})

    def test_rejects_extra_text_and_retries(self):
        bad = '```json\n{"object_name":"iv","confidence":0.9,"description":"输液袋"}\n```'
        good = '{"object_name":"iv","confidence":0.9,"description":"输液袋"}'
        session = FakeSession([FakeResponse(bad), FakeResponse(good)])
        recognizer = MoonshotVisionRecognizer(
            api_key="test-key",
            retries=2,
            session=session,
        )
        result = recognizer.recognize(np.zeros((32, 32, 3), dtype=np.uint8))
        self.assertEqual(result.region, 2)
        self.assertEqual(len(session.calls), 2)

    def test_rejects_unknown_class(self):
        recognizer = MoonshotVisionRecognizer(api_key="test-key")
        with self.assertRaises(MoonshotVisionError):
            recognizer._parse_content(
                '{"object_name":"rifle","confidence":0.9,"description":"步枪"}'
            )

    def test_rejects_region_injected_by_model(self):
        recognizer = MoonshotVisionRecognizer(api_key="test-key")
        with self.assertRaises(MoonshotVisionError):
            recognizer._parse_content(
                '{"object_name":"ak47","confidence":0.9,'
                '"description":"步枪","region":3}'
            )


if __name__ == "__main__":
    unittest.main()
