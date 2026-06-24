#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

import race_params as params


class RaceParamsTests(unittest.TestCase):
    def test_navigation_goals_have_xy_yaw(self):
        goals = [
            params.SHOOT_1_GOAL,
            params.SHOOT_2_GOAL,
            params.SHOOT_3_INSPECTION_GOAL,
            params.SHOOT_3_SHOOT_GOAL,
            params.FINISH_GOAL,
        ] + params.NARROW_PASSAGE_GOALS
        self.assertTrue(all(len(goal) == 3 for goal in goals))

    def test_shoot_tolerances_are_positive(self):
        self.assertGreater(params.SHOOT1_HORIZONTAL_TOLERANCE, 0)
        self.assertGreater(params.SHOOT2_HORIZONTAL_TOLERANCE, 0)
        self.assertGreater(params.SHOOT2_VERTICAL_TOLERANCE, 0)
        self.assertGreater(params.SHOOT3_X_TOLERANCE, 0)

    def test_moving_regions_are_complete(self):
        self.assertEqual(set(params.SHOOT3_REGION_X_OFFSET), {1, 2, 3})
        self.assertLess(params.SHOOT3_REGION_X_OFFSET[1], 0)
        self.assertEqual(params.SHOOT3_REGION_X_OFFSET[2], 0)
        self.assertGreater(params.SHOOT3_REGION_X_OFFSET[3], 0)

    def test_speed_limits_and_confidence_are_valid(self):
        self.assertGreater(params.FINE_ANGULAR_MAX, params.FINE_ANGULAR_MIN)
        self.assertGreater(params.DIRECT_YAW_MAX, params.DIRECT_YAW_MIN)
        self.assertTrue(0 <= params.MOONSHOT_MIN_CONFIDENCE <= 1)


if __name__ == "__main__":
    unittest.main()
