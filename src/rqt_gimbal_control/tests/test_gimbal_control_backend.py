import unittest
import sys
import os

# Add the package source to the Python path to allow importing the module.
# This is a more robust way to handle the path for running tests without a full colcon build/install.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from rqt_gimbal_control.gimbal_control_backend import RangeTranslate


class TestRangeTranslate(unittest.TestCase):
    """Unit tests for the RangeTranslate class."""

    def test_positive_range_mapping(self):
        """Test mapping from a standard [0, 100] range to a positive output range [0, 180]."""
        translator = RangeTranslate(min_in=0, max_in=100, min_out=0, max_out=180)
        self.assertAlmostEqual(translator.translate(0), 0, msg="Min input should map to min output")
        self.assertAlmostEqual(translator.translate(100), 180, msg="Max input should map to max output")
        self.assertAlmostEqual(translator.translate(50), 90, msg="Mid input should map to mid output")
        self.assertAlmostEqual(translator.translate(25), 45, msg="Quarter input should map to quarter output")

    def test_mixed_positive_negative_range(self):
        """Test mapping to a range that crosses zero, like [-90, 90]."""
        translator = RangeTranslate(min_in=0, max_in=100, min_out=-90, max_out=90)
        self.assertAlmostEqual(translator.translate(0), -90, msg="Min input should map to min output")
        self.assertAlmostEqual(translator.translate(100), 90, msg="Max input should map to max output")
        self.assertAlmostEqual(translator.translate(50), 0, msg="Mid input should map to mid output (zero)")
        self.assertAlmostEqual(translator.translate(25), -45, msg="Quarter input should map correctly")
        self.assertAlmostEqual(translator.translate(75), 45, msg="Three-quarter input should map correctly")

    def test_fully_negative_range(self):
        """Test mapping to a range that is entirely negative, like [-50, -10]."""
        translator = RangeTranslate(min_in=0, max_in=100, min_out=-50, max_out=-10)
        self.assertAlmostEqual(translator.translate(0), -50, msg="Min input should map to min output")
        self.assertAlmostEqual(translator.translate(100), -10, msg="Max input should map to max output")
        self.assertAlmostEqual(translator.translate(50), -30, msg="Mid input should map to mid output")

    def test_inverted_output_range(self):
        """Test mapping to an inverted output range where min_out > max_out."""
        translator = RangeTranslate(min_in=0, max_in=100, min_out=100, max_out=0)
        self.assertAlmostEqual(translator.translate(0), 100, msg="Min input should map to inverted max output")
        self.assertAlmostEqual(translator.translate(100), 0, msg="Max input should map to inverted min output")
        self.assertAlmostEqual(translator.translate(50), 50, msg="Midpoint should remain the same")
        self.assertAlmostEqual(translator.translate(25), 75, msg="Quarter input should map correctly to inverted range")

    def test_non_zero_based_input_range(self):
        """Test mapping from an input range that does not start at zero, like [-1, 1]."""
        translator = RangeTranslate(min_in=-1, max_in=1, min_out=0, max_out=100)
        self.assertAlmostEqual(translator.translate(-1), 0, msg="Min input should map to min output")
        self.assertAlmostEqual(translator.translate(1), 100, msg="Max input should map to max output")
        self.assertAlmostEqual(translator.translate(0), 50, msg="Mid input should map to mid output")

    def test_floating_point_ranges_and_values(self):
        """Test mapping with floating point numbers for ranges and values."""
        translator = RangeTranslate(min_in=0.5, max_in=1.5, min_out=-10.0, max_out=10.0)
        self.assertAlmostEqual(translator.translate(0.5), -10.0)
        self.assertAlmostEqual(translator.translate(1.5), 10.0)
        self.assertAlmostEqual(translator.translate(1.0), 0.0)
        self.assertAlmostEqual(translator.translate(0.75), -5.0)

    def test_zero_length_input_range_error(self):
        """Test that a zero-length input range (min_in == max_in) raises a ZeroDivisionError."""
        translator = RangeTranslate(min_in=50, max_in=50, min_out=0, max_out=100)
        with self.assertRaises(ZeroDivisionError, msg="Translating with a zero-width input range should raise ZeroDivisionError"):
            translator.translate(50)