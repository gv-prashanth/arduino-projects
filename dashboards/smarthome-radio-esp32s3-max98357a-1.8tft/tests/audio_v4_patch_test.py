"""Test manifest integrity and patch real 4.0.0 fixtures in temporary directories.

Run with --library PATH or AUDIOI2S_V4_LIBRARY=PATH to exercise the exact upstream
source. Without that optional installed dependency, source integration tests skip.
"""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "scripts" / "patch_audioi2s_v4.py"
spec = importlib.util.spec_from_file_location("audio_v4_patch", SCRIPT)
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)
LIBRARY = os.environ.get("AUDIOI2S_V4_LIBRARY")


class ManifestTests(unittest.TestCase):
    def test_helper_and_manifest_are_consistent(self):
        self.assertEqual(patch.MANIFEST["schema_version"], 1)
        self.assertEqual(patch.MANIFEST["library_version"], "4.0.0")
        self.assertEqual(patch.digest(patch.HELPER.read_bytes()), patch.MANIFEST["helper"]["sha256"])
        self.assertEqual(set(patch.MANIFEST["files"]), {"Audio.h", "Audio.cpp"})
        for name, item in patch.MANIFEST["files"].items():
            self.assertEqual(item["path"], "src/" + name)
            self.assertNotEqual(item["original_sha256"], item["patched_sha256"])
            self.assertTrue(item["replacements"])

    def test_cli_rejects_a_missing_library(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, str(SCRIPT), "--library", directory, "--check"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 1)
            self.assertIn("library.properties", result.stderr)
            self.assertEqual(list(Path(directory).iterdir()), [])


class SourcePatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not LIBRARY:
            raise unittest.SkipTest("Pass --library PATH or AUDIOI2S_V4_LIBRARY for exact 4.0.0 source tests")
        _, source_dir = patch.library_paths(LIBRARY)
        cls.originals = {}
        for name, expected in patch.ORIGINAL_SHA256.items():
            data = (source_dir / name).read_bytes()
            if patch.digest(data) != expected:
                data = (source_dir / (name + patch.BACKUP_SUFFIX)).read_bytes()
            if patch.digest(data) != expected:
                raise RuntimeError("Source fixture does not match reviewed upstream 4.0.0: " + name)
            cls.originals[name] = data

    def fixture(self, directory):
        root = Path(directory) / "ESP32-audioI2S-master"
        source = root / "src"
        source.mkdir(parents=True)
        (root / "library.properties").write_text("name=ESP32-audioI2S-master\nversion=4.0.0\n")
        for name, data in self.originals.items():
            (source / name).write_bytes(data)
        return root, source

    @staticmethod
    def snapshot(root):
        return {str(path.relative_to(root)): path.read_bytes() for path in root.rglob("*") if path.is_file()}

    def test_exact_patch_backups_and_idempotence(self):
        with tempfile.TemporaryDirectory() as directory:
            root, source = self.fixture(directory)
            self.assertEqual(patch.patch_library(root), "patched")
            for name, original in self.originals.items():
                self.assertEqual((source / (name + patch.BACKUP_SUFFIX)).read_bytes(), original)
                self.assertEqual(patch.digest((source / name).read_bytes()), patch.PATCHED_SHA256[name])
            self.assertEqual((source / patch.HELPER_NAME).read_bytes(), patch.HELPER.read_bytes())
            before = self.snapshot(root)
            self.assertEqual(patch.patch_library(root), "already patched")
            self.assertEqual(patch.patch_library(source, check=True), "already patched")
            self.assertEqual(self.snapshot(root), before)

    def test_check_does_not_write_and_cli_accepts_src(self):
        with tempfile.TemporaryDirectory() as directory:
            root, source = self.fixture(directory)
            before = self.snapshot(root)
            self.assertEqual(patch.patch_library(root, check=True), "pristine")
            result = subprocess.run([sys.executable, str(SCRIPT), "--library", str(source), "--check"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("pristine (check only)", result.stdout)
            self.assertEqual(self.snapshot(root), before)

    def test_unknown_version_and_modified_sources_are_rejected_before_writing(self):
        for modified in ["version", "Audio.h", "Audio.cpp"]:
            with self.subTest(modified=modified), tempfile.TemporaryDirectory() as directory:
                root, source = self.fixture(directory)
                if modified == "version":
                    (root / "library.properties").write_text("version=4.0.1\n")
                else:
                    with (source / modified).open("ab") as handle:
                        handle.write(b"\n// unreviewed edit\n")
                before = self.snapshot(root)
                with self.assertRaisesRegex(RuntimeError, "refusing to patch"):
                    patch.patch_library(root)
                self.assertEqual(self.snapshot(root), before)

    def test_partial_install_and_tampered_backups_or_helper_are_rejected(self):
        for modified in ["partial", "Audio.h", "Audio.cpp", "backup", "helper", "missing backup"]:
            with self.subTest(modified=modified), tempfile.TemporaryDirectory() as directory:
                root, source = self.fixture(directory)
                if modified == "partial":
                    (source / patch.HELPER_NAME).write_bytes(patch.HELPER.read_bytes())
                else:
                    patch.patch_library(root)
                    target = {
                        "Audio.h": source / "Audio.h",
                        "Audio.cpp": source / "Audio.cpp",
                        "backup": source / ("Audio.h" + patch.BACKUP_SUFFIX),
                        "helper": source / patch.HELPER_NAME,
                        "missing backup": source / ("Audio.cpp" + patch.BACKUP_SUFFIX),
                    }[modified]
                    if modified == "missing backup":
                        target.unlink()
                    else:
                        with target.open("ab") as handle:
                            handle.write(b"\n// unexpected edit\n")
                before = self.snapshot(root)
                with self.assertRaisesRegex(RuntimeError, "refusing to patch"):
                    patch.patch_library(root)
                self.assertEqual(self.snapshot(root), before)

    def test_write_failures_restore_the_complete_original_library(self):
        for failing_name in ["Audio.h" + patch.BACKUP_SUFFIX,
                             "Audio.cpp" + patch.BACKUP_SUFFIX,
                             patch.HELPER_NAME, "Audio.h", "Audio.cpp"]:
            with self.subTest(failing_name=failing_name), tempfile.TemporaryDirectory() as directory:
                root, _ = self.fixture(directory)
                before = self.snapshot(root)
                real_write = Path.write_bytes
                failed = [False]

                def interrupted_write(path, data):
                    if path.name == failing_name and not failed[0]:
                        failed[0] = True
                        real_write(path, data[:17])  # Simulate a partially completed write.
                        raise OSError("simulated full disk")
                    return real_write(path, data)

                with mock.patch.object(Path, "write_bytes", interrupted_write):
                    with self.assertRaisesRegex(RuntimeError, "original library restored"):
                        patch.patch_library(root)
                self.assertTrue(failed[0])
                self.assertEqual(self.snapshot(root), before)
                self.assertEqual(patch.patch_library(root, check=True), "pristine")

    def test_gate_preserves_network_pump_decoder_lock_and_pcm_drain_order(self):
        cpp = patch.patched_source("Audio.cpp", self.originals["Audio.cpp"]).decode()
        play = cpp[cpp.index("void Audio::playAudioData() {"):cpp.index("std::vector<ps_ptr<char>> Audio::readHeader()")]
        self.assertLess(play.index("if (m_validSamples)"), play.index("m_radioStreamReserve.update("))
        self.assertLess(play.index("xSemaphoreTake(mutex_audioTaskIsDecoding"), play.index("m_radioStreamReserve.update("))
        self.assertLess(play.index("m_radioStreamReserve.update("), play.index("if (m_f_allDataReceived)"))
        self.assertIn("InBuff.bufferFilled()", play)
        self.assertIn("if (reserveWaiting) goto exit", play)
        self.assertIn("xSemaphoreGive(mutex_audioTaskIsDecoding)", play)
        # Added code never changes the pump-enabling stream flag or resets the decoder.
        additions = "\n".join(item["new"].replace(item["old"], "") for item in patch.MANIFEST["files"]["Audio.cpp"]["replacements"])
        self.assertNotIn("m_f_stream =", additions)
        self.assertNotIn("m_decoder->clear", additions)
        self.assertNotIn("vTaskDelay", additions)
        process = cpp[cpp.index("void Audio::processWebStream() {"):cpp.index("void Audio::processWebFile() {")]
        self.assertLess(process.index("m_radioStreamTerminal.store"), process.index("if (m_f_chunked"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--library")
    args, remaining = parser.parse_known_args()
    LIBRARY = args.library or LIBRARY
    unittest.main(argv=[sys.argv[0]] + remaining)
