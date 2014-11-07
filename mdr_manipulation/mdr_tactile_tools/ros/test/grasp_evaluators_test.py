#!/usr/bin/env python
"""
Test unit for the monitors.py module.

"""

PKG = 'mdr_tactile_grasp'

import unittest
import rosunit
import numpy
import mdr_tactile_tools.grasp_evaluators
import mdr_tactile_tools_common.tactile_filters as tactile_filters


class TestGraspEvaluators(unittest.TestCase):
    """
    Tests each method in the monitors.py module.

    """

    def test_detect_contact_all(self):
        """
        Tests that the 'detect_contact' function returns correctly that all
        tactile sensors have a contact.

        """
        thresholds = [20] * 6
        contact_matrix = [[80, 45, 35], [200, 450, 355], [90, 475, 385],
                          [200, 5, 85], [110, 145, 25], [50, 450, 305]]
        result = [1] * 6

        self.assertEqual(tactile_filters.detect_contact(
            contact_matrix, thresholds), result)

    def test_detect_contact_all(self):
        """
        Tests that the 'detect_contact' function returns correctly that none
        tactile sensors have a contact.

        """
        thresholds = [20] * 6
        contact_matrix = [[20, 15, 5], [8, 15, 9], [7, 15, 8],
                          [16, 15, 15], [20, 4, 5], [20, 5, 4]]
        result = [0] * 6

        self.assertEqual(tactile_filters.detect_contact(
            contact_matrix, thresholds), result)

    def test_detect_contact_proximal(self):
        """
        Tests that the 'detect_contact' function returns correctly that only
        the proximal tactile sensors have a contact.

        """
        thresholds = [20] * 6
        contact_matrix = [[20, 15, 5], [200, 5, 85], [7, 15, 8],
                          [110, 145, 25], [20, 4, 5], [50, 450, 305]]
        result = [0, 1, 0, 1, 0, 1]

        self.assertEqual(tactile_filters.detect_contact(
            contact_matrix, thresholds), result)

    def test_detect_contact_distal(self):
        """
        Tests that the 'detect_contact' function returns correctly that only
        the distal tactile sensors have a contact.

        """
        thresholds = [20] * 6
        contact_matrix = [[90, 475, 385], [8, 15, 9], [200, 450, 355],
                          [16, 15, 15], [80, 45, 35], [20, 5, 4]]
        result = [1, 0, 1, 0, 1, 0]

        self.assertEqual(tactile_filters.detect_contact(
            contact_matrix, thresholds), result)

    def test_true_cylindrical_grasp(self):
        """
        Tests for positive cases of the 'evaluate_cylindrical_grasp' function.

        """
        contact_matrix_1 = [1, 0, 1, 0, 1, 0]
        contact_matrix_2 = [1, 0, 0, 1, 0, 0]
        contact_matrix_3 = [0, 1, 1, 0, 1, 1]
        contact_matrix_4 = [1, 1, 1, 1, 1, 1]

        result = True

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_cylindrical_grasp(
                contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_cylindrical_grasp(
                contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_cylindrical_grasp(
                contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_cylindrical_grasp(
                contact_matrix_4), result)

    def test_false_cylindrical_grasp(self):
        """
        Tests for false cases of the 'evaluate_cylindrical_grasp' function.

        """
        contact_matrix_1 = [1, 1, 0, 0, 0, 0]
        contact_matrix_2 = [0, 0, 0, 0, 0, 0]
        contact_matrix_3 = [0, 0, 1, 1, 1, 1]
        contact_matrix_4 = [0, 0, 0, 1, 0, 1]

        result = False

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_cylindrical_grasp(
                contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_cylindrical_grasp(
                contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_cylindrical_grasp(
                contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_cylindrical_grasp(
                contact_matrix_4), result)

    def test_true_spherical_grasp(self):
        """
        Tests for true cases of the 'evaluate_spherical_grasp' function.

        """
        contact_matrix_1 = [1, 0, 1, 0, 1, 0]
        contact_matrix_2 = [1, 0, 0, 1, 0, 1]
        contact_matrix_3 = [0, 1, 1, 0, 1, 1]
        contact_matrix_4 = [1, 1, 1, 1, 1, 1]

        result = True

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_spherical_grasp(
                contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_spherical_grasp(
                contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_spherical_grasp(
                contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_spherical_grasp(
                contact_matrix_4), result)

    def test_false_spherical_grasp(self):
        """
        Tests for false cases of the 'evaluate_spherical_grasp' function.

        """
        contact_matrix_1 = [0, 1, 0, 0, 1, 0]
        contact_matrix_2 = [0, 0, 0, 0, 0, 0]
        contact_matrix_3 = [0, 0, 1, 1, 1, 1]
        contact_matrix_4 = [1, 1, 0, 0, 0, 1]

        result = False

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_spherical_grasp(
                contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_spherical_grasp(
                contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_spherical_grasp(
                contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_spherical_grasp(
                contact_matrix_4), result)

    def test_true_precision_grasp(self):
        """
        Tests for true cases of the 'evaluate_precision_grasp' function.

        """
        contact_matrix_1 = [0, 0, 0, 1, 0, 1]
        contact_matrix_2 = [1, 0, 1, 1, 0, 1]
        contact_matrix_3 = [1, 1, 1, 1, 1, 1]
        contact_matrix_4 = [1, 1, 0, 1, 0, 1]

        result = True

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_precision_grasp(
                contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_precision_grasp(
                contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_precision_grasp(
                contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_precision_grasp(
                contact_matrix_4), result)

    def test_false_precision_grasp(self):
        """
        Tests for false cases of the 'evaluate_precision_grasp' function.

        """
        contact_matrix_1 = [0, 1, 0, 0, 1, 0]
        contact_matrix_2 = [0, 0, 0, 0, 0, 0]
        contact_matrix_3 = [1, 1, 1, 0, 1, 1]
        contact_matrix_4 = [1, 1, 0, 1, 0, 0]

        result = False

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_precision_grasp(
                contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_precision_grasp(
                contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_precision_grasp(
                contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.evaluate_precision_grasp(
                contact_matrix_4), result)

    def test_true_cylindrical_grasp(self):
        """
        Tests for true cases of the 'calculate_grasp_status' function for
        a 'cylindrical' grasp.

        """
        grasp = 'cylindrical'
        contact_matrix_1 = [1, 0, 1, 0, 1, 0]
        contact_matrix_2 = [1, 0, 0, 1, 0, 0]
        contact_matrix_3 = [0, 1, 1, 0, 1, 1]
        contact_matrix_4 = [1, 1, 1, 1, 1, 1]

        result = 'e_grasped'

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_4), result)

    def test_false_cylindrical_grasp(self):
        """
        Tests for false cases of the 'calculate_grasp_status' function for
        a 'cylindrical' grasp.

        """
        grasp = 'cylindrical'
        contact_matrix_1 = [1, 1, 0, 0, 0, 0]
        contact_matrix_2 = [0, 0, 0, 0, 0, 0]
        contact_matrix_3 = [0, 0, 1, 1, 1, 1]
        contact_matrix_4 = [0, 0, 0, 1, 0, 1]

        result = 'e_not_grasped'

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_4), result)

    def test_true_spherical_grasp(self):
        """
        Tests for true cases of the 'calculate_grasp_status' function for
        a 'spherical' grasp.

        """
        grasp = 'spherical'
        contact_matrix_1 = [1, 0, 1, 0, 1, 0]
        contact_matrix_2 = [1, 0, 0, 1, 0, 1]
        contact_matrix_3 = [0, 1, 1, 0, 1, 1]
        contact_matrix_4 = [1, 1, 1, 1, 1, 1]

        result = 'e_grasped'

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_4), result)

    def test_false_spherical_grasp(self):
        """
        Tests for false cases of the 'calculate_grasp_status' function for
        a 'spherical' grasp.

        """
        grasp = 'spherical'
        contact_matrix_1 = [0, 1, 0, 0, 1, 0]
        contact_matrix_2 = [0, 0, 0, 0, 0, 0]
        contact_matrix_3 = [0, 0, 1, 1, 1, 1]
        contact_matrix_4 = [1, 1, 0, 0, 0, 1]

        result = 'e_not_grasped'

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_4), result)

    def test_true_precision_grasp(self):
        """
        Tests for true cases of the 'calculate_grasp_status' function for
        a 'precision' grasp.

        """
        grasp = 'precision'
        contact_matrix_1 = [0, 0, 0, 1, 0, 1]
        contact_matrix_2 = [1, 0, 1, 1, 0, 1]
        contact_matrix_3 = [1, 1, 1, 1, 1, 1]
        contact_matrix_4 = [1, 1, 0, 1, 0, 1]

        result = 'e_grasped'

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_4), result)

    def test_false_precision_grasp(self):
        """
        Tests for false cases of the 'calculate_grasp_status' function for
        a 'precision' grasp.

        """
        grasp = 'precision'
        contact_matrix_1 = [0, 1, 0, 0, 1, 0]
        contact_matrix_2 = [0, 0, 0, 0, 0, 0]
        contact_matrix_3 = [1, 1, 1, 0, 1, 1]
        contact_matrix_4 = [1, 1, 0, 1, 0, 0]

        result = 'e_not_grasped'

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp, contact_matrix_4), result)

    def test_invalid_grasp(self):
        """
        Tests for false cases of the 'calculate_grasp_status' function for
        a 'spherical' grasp.

        """
        grasp_1 = 'precision3'
        grasp_2 = 'cyleidirc'
        grasp_3 = 'cylindric'
        grasp_4 = 'spheicirl'
        contact_matrix_1 = [0, 1, 0, 0, 1, 0]
        contact_matrix_2 = [0, 0, 0, 0, 0, 0]
        contact_matrix_3 = [1, 1, 1, 0, 1, 1]
        contact_matrix_4 = [1, 1, 0, 1, 0, 0]

        result = 'e_failed'

        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp_1, contact_matrix_1), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp_2, contact_matrix_2), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp_3, contact_matrix_3), result)
        self.assertEqual(
            mdr_tactile_tools.grasp_evaluators.calculate_grasp_status(
                grasp_4, contact_matrix_4), result)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_grasp_evaluators', TestGraspEvaluators)
