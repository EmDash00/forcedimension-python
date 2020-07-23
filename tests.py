import unittest

import runtime


class TestRuntimeUtils(unittest.TestCase):
    def test_version_tuple(self):
        for i in range(10):
            for j in range(10):
                for k in range(10):
                    test = runtime.version_tuple("{}.{}.{}".format(i, j, k))
                    actual = (i, j, k)

                    self.assertEqual(test, actual)

        self.assertRaises(ValueError, runtime.version_tuple, "0.0.-51")


if __name__ == "__main__":
    unittest.main()
