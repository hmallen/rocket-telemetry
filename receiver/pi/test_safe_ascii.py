import sys
import unittest
from unittest.mock import MagicMock

# Mock dependencies before importing the module under test
sys.modules["spidev"] = MagicMock()
sys.modules["RPi"] = MagicMock()
sys.modules["RPi.GPIO"] = MagicMock()
sys.modules["smbus2"] = MagicMock()

# Now we can import the module
from lora_receive_pi import safe_ascii

class TestSafeAscii(unittest.TestCase):
    def test_printable_ascii(self):
        """Test that printable ASCII characters are preserved."""
        data = b"Hello World"
        self.assertEqual(safe_ascii(data), "Hello World")

    def test_non_printable(self):
        """Test that non-printable characters are replaced with '.'."""
        # 0x00 to 0x1F are non-printable
        data = b"\x00\x01\x1F"
        self.assertEqual(safe_ascii(data), "...")
        # 0x7F (DEL) is non-printable
        data = b"\x7F"
        self.assertEqual(safe_ascii(data), ".")
        # 0x80+ are non-printable (for ASCII)
        data = b"\x80\xFF"
        self.assertEqual(safe_ascii(data), "..")

    def test_mixed_content(self):
        """Test a mix of printable and non-printable characters."""
        data = b"Data\x00Value"
        self.assertEqual(safe_ascii(data), "Data.Value")

    def test_boundaries(self):
        """Test the exact boundaries of printable ASCII (32-126)."""
        # 31 is non-printable, 32 (space) is printable
        # 126 (~) is printable, 127 (DEL) is non-printable
        data = bytes([31, 32, 126, 127])
        self.assertEqual(safe_ascii(data), ". ~.")

    def test_empty_input(self):
        """Test that empty input returns empty string."""
        self.assertEqual(safe_ascii(b""), "")

    def test_bytearray_input(self):
        """Test that the function handles bytearray input correctly."""
        data = bytearray(b"Test")
        self.assertEqual(safe_ascii(data), "Test")

if __name__ == "__main__":
    unittest.main()
