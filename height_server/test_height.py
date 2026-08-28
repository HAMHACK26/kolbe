import os
import shutil
import tempfile
import unittest


TEST_CACHE = tempfile.mkdtemp(prefix="kolbe-height-tests-")
os.environ["SECRET"] = "test-only-secret"
os.environ["CACHE_DIR"] = TEST_CACHE

import height  # noqa: E402


def tearDownModule():
    shutil.rmtree(TEST_CACHE, ignore_errors=True)


class HeightServerTests(unittest.TestCase):
    def test_bbox_is_centered_and_has_requested_size(self):
        west, south, east, north = height.bbox_from_center(59.3293, 18.0686)

        self.assertAlmostEqual((south + north) / 2, 59.3293)
        self.assertAlmostEqual((west + east) / 2, 18.0686)
        self.assertAlmostEqual((north - south) * 111.32, height.RADIUS_KM * 2)

    def test_bbox_rejects_coordinates_outside_sweden(self):
        with self.assertRaises(ValueError):
            height.bbox_from_center(48.8566, 2.3522)

    def test_asset_filter_keeps_only_geotiffs(self):
        features = [
            {
                "assets": {
                    "terrain": {
                        "href": "https://example.test/terrain.tif?token=temporary",
                        "type": "image/tiff; application=geotiff",
                    },
                    "point-cloud": {
                        "href": "https://example.test/forest.laz",
                        "type": "application/vnd.laszip+copc",
                        "roles": ["data"],
                    },
                    "metadata": {
                        "href": "https://example.test/terrain.json",
                        "type": "application/json",
                    },
                }
            }
        ]

        self.assertEqual(
            height.asset_urls(features),
            ["https://example.test/terrain.tif?token=temporary"],
        )

    def test_tile_cache_key_ignores_temporary_query_parameters(self):
        first = height.tile_cache_path("https://example.test/a.tif?token=one")
        second = height.tile_cache_path("https://example.test/a.tif?token=two")

        self.assertEqual(first, second)

    def test_progress_protocol_contains_expected_fields(self):
        height.set_progress("Downloading", done=2, total=9, current="tile.tif")

        self.assertEqual(
            height.progress_text().decode("utf-8"),
            "phase=Downloading\ndone=2\ntotal=9\ncurrent=tile.tif\n",
        )


if __name__ == "__main__":
    unittest.main()
