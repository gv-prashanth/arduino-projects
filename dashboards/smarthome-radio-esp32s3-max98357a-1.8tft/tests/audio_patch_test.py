"""Verify the dependency patch against installed source without modifying it."""

import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("audio_patch", PROJECT / "scripts" / "patch_audioi2s.py")
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)


class AudioPatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        installed = PROJECT / ".pio" / "libdeps" / "esp32s3-n16r8" / "ESP32-audioI2S" / "src" / "Audio.cpp"
        if not installed.is_file():
            raise RuntimeError("Run pio run first to install the pinned audio dependency")
        data = installed.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if digest == patch.PATCHED_SHA256:
            data = data.replace(patch.NEW_COPY, patch.OLD_COPY)
        if hashlib.sha256(data).hexdigest() != patch.ORIGINAL_SHA256:
            raise RuntimeError("Installed dependency is not the reviewed audio source")
        cls.original = data

    def test_patch_is_exact_and_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "Audio.cpp"
            source.write_bytes(self.original)
            self.assertTrue(patch.patch_audio_source(source))
            expected = self.original.replace(patch.OLD_COPY, patch.NEW_COPY)
            self.assertEqual(source.read_bytes(), expected)
            self.assertEqual(expected.count(patch.NEW_COPY), 2)
            self.assertFalse(patch.patch_audio_source(source))
            self.assertEqual(source.read_bytes(), expected)

    def test_unreviewed_source_is_rejected_without_modification(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "Audio.cpp"
            modified = self.original + b"\n// Unexpected dependency modification\n"
            source.write_bytes(modified)
            with self.assertRaisesRegex(RuntimeError, "refusing to patch"):
                patch.patch_audio_source(source)
            self.assertEqual(source.read_bytes(), modified)


if __name__ == "__main__":
    unittest.main()
