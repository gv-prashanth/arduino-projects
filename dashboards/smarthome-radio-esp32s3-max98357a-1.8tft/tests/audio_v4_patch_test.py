"""Test manifest integrity and patch supported audioI2S fixtures in temporary directories.

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

    def test_supported_profiles_have_independent_exact_hashes(self):
        self.assertEqual(set(patch.MANIFESTS), {"4.0.0", "4.0.1"})
        profile = patch.MANIFESTS["4.0.1"]
        self.assertEqual(profile["upstream_commit"], "5bf339285bf681d8be2cc84a5f7529a12f2f7514")
        self.assertEqual(profile["files"]["Audio.h"]["original_sha256"],
                         "119d69420fbc6d68d8f1fc8c10db2d11f18abc10cb19450a4d65f1741d512c09")
        self.assertEqual(profile["files"]["Audio.cpp"]["original_sha256"],
                         "08481cdff892d4bb05f9a9bcc5dd76c20a8526f0bcdbf8f4b42a9f586f1bddad")
        for version, profile in patch.MANIFESTS.items():
            self.assertEqual(profile["schema_version"], 1)
            self.assertEqual(profile["library_version"], version)
            helper = patch.MANIFEST_PATH.parent / profile["helper"]["source"]
            self.assertEqual(patch.digest(helper.read_bytes()), profile["helper"]["sha256"])
            self.assertEqual(set(profile["files"]), {"Audio.h", "Audio.cpp"})

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
            raise unittest.SkipTest("Pass --library PATH or AUDIOI2S_V4_LIBRARY for exact supported source tests")
        _, source_dir, cls.manifest = patch.library_info(LIBRARY)
        cls.version = cls.manifest["library_version"]
        cls.original_hashes = {name: item["original_sha256"] for name, item in cls.manifest["files"].items()}
        cls.patched_hashes = {name: item["patched_sha256"] for name, item in cls.manifest["files"].items()}
        cls.originals = {}
        for name, expected in cls.original_hashes.items():
            data = (source_dir / name).read_bytes()
            if patch.digest(data) != expected:
                data = (source_dir / (name + patch.BACKUP_SUFFIX)).read_bytes()
            if patch.digest(data) != expected:
                raise RuntimeError("Source fixture does not match reviewed upstream " + cls.version + ": " + name)
            cls.originals[name] = data

    def fixture(self, directory):
        root = Path(directory) / "ESP32-audioI2S-master"
        source = root / "src"
        source.mkdir(parents=True)
        (root / "library.properties").write_text("name=ESP32-audioI2S-master\nversion=" + self.version + "\n")
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
                self.assertEqual(patch.digest((source / name).read_bytes()), self.patched_hashes[name])
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
                    (root / "library.properties").write_text("version=4.0.2\n")
                else:
                    with (source / modified).open("ab") as handle:
                        handle.write(b"\n// unreviewed edit\n")
                before = self.snapshot(root)
                with self.assertRaisesRegex(RuntimeError, "refusing to patch"):
                    patch.patch_library(root)
                self.assertEqual(self.snapshot(root), before)

    def test_version_formatting_is_accepted_without_changing_properties(self):
        for properties in [b"name=ESP32-audioI2S-master\r\nversion=4.0.0\r\n",
                           b"\xef\xbb\xbfversion=4.0.0\r\n",
                           b"name=ESP32-audioI2S-master\n \tversion \t= \t4.0.0 \t\n"]:
            with self.subTest(properties=properties), tempfile.TemporaryDirectory() as directory:
                root, _ = self.fixture(directory)
                properties = properties.replace(b"4.0.0", self.version.encode())
                (root / "library.properties").write_bytes(properties)
                before = self.snapshot(root)
                self.assertEqual(patch.patch_library(root, check=True), "pristine")
                self.assertEqual(self.snapshot(root), before)
                self.assertEqual(patch.patch_library(root), "patched")
                self.assertEqual((root / "library.properties").read_bytes(), properties)

    def test_version_errors_report_actual_value_and_path_without_writing(self):
        for properties, found in [(b"version=4.0.2\n", "'4.0.2'"),
                                  (b"name=ESP32-audioI2S-master\n", "<missing>"),
                                  (b"version=\nname=ESP32-audioI2S-master\n", "''"),
                                  (b"version=4.0.0\nversion=4.0.1\n", "'4.0.0', '4.0.1'")]:
            with self.subTest(properties=properties), tempfile.TemporaryDirectory() as directory:
                root, _ = self.fixture(directory)
                (root / "library.properties").write_bytes(properties)
                before = self.snapshot(root)
                for check in [True, False]:
                    with self.assertRaises(RuntimeError) as error:
                        patch.patch_library(root, check=check)
                    self.assertIn("found " + found, str(error.exception))
                    self.assertIn(str(root / "library.properties"), str(error.exception))
                    self.assertIn("refusing to patch", str(error.exception))
                    self.assertEqual(self.snapshot(root), before)

    def test_declared_profile_must_match_both_sources_and_original_backups(self):
        other_version = next(version for version in patch.MANIFESTS if version != self.version)
        for installed in [False, True]:
            with self.subTest(installed=installed), tempfile.TemporaryDirectory() as directory:
                root, _ = self.fixture(directory)
                if installed:
                    patch.patch_library(root)
                (root / "library.properties").write_text("version=" + other_version + "\n")
                before = self.snapshot(root)
                for check in [True, False]:
                    with self.assertRaises(RuntimeError) as error:
                        patch.patch_library(root, check=check)
                    self.assertIn("declared version " + other_version, str(error.exception))
                    self.assertIn("Audio.h SHA256=", str(error.exception))
                    self.assertIn("Audio.cpp SHA256=", str(error.exception))
                    self.assertEqual(self.snapshot(root), before)

    def test_mixed_pristine_and_patched_files_are_rejected(self):
        for name in self.originals:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                root, source = self.fixture(directory)
                (source / name).write_bytes(patch.patched_source(name, self.originals[name], self.manifest))
                before = self.snapshot(root)
                for check in [True, False]:
                    with self.assertRaisesRegex(RuntimeError, "refusing to patch"):
                        patch.patch_library(root, check=check)
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
        cpp = patch.patched_source("Audio.cpp", self.originals["Audio.cpp"], self.manifest).decode()
        play = cpp[cpp.index("void Audio::playAudioData() {"):cpp.index("std::vector<ps_ptr<char>> Audio::readHeader()")]
        self.assertLess(play.index("if (m_validSamples)"), play.index("m_radioStreamReserve.update("))
        self.assertLess(play.index("xSemaphoreTake(mutex_audioTaskIsDecoding"), play.index("m_radioStreamReserve.update("))
        self.assertLess(play.index("m_radioStreamReserve.update("), play.index("if (m_f_allDataReceived)"))
        self.assertIn("InBuff.bufferFilled()", play)
        self.assertIn("if (reserveWaiting) goto exit", play)
        self.assertIn("xSemaphoreGive(mutex_audioTaskIsDecoding)", play)
        # Added code never changes the pump-enabling stream flag or resets the decoder.
        additions = "\n".join(item["new"].replace(item["old"], "") for item in self.manifest["files"]["Audio.cpp"]["replacements"])
        self.assertNotIn("m_f_stream =", additions)
        self.assertNotIn("m_decoder->clear", additions)
        self.assertNotIn("vTaskDelay", additions)
        process = cpp[cpp.index("void Audio::processWebStream() {"):cpp.index("void Audio::processWebFile() {")]
        self.assertLess(process.index("m_radioStreamTerminal.store"), process.index("if (m_f_chunked"))

    def test_v401_advertised_bitrate_is_private_and_preserved_at_first_play(self):
        if self.version != "4.0.1":
            self.skipTest("4.0.1-specific bitrate accounting")
        cpp = patch.patched_source("Audio.cpp", self.originals["Audio.cpp"], self.manifest).decode()
        first_play = cpp[cpp.index("if (m_f_firstPlayCall)"):cpp.index("if (!isFile() && !isStream())")]
        self.assertNotIn("m_radioStreamAdvertisedBitrate", first_play)
        # Preserve upstream's public bitrate accounting while capturing the
        # advertised rate independently for the compressed reserve target.
        self.assertIn('//    if (name.equals_icase("icy-br")) { m_nominal_bitrate', cpp)
        self.assertIn("UINT32_MAX / 1000", cpp)
        self.assertIn("m_radioStreamAdvertisedBitrate.load", cpp)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--library")
    args, remaining = parser.parse_known_args()
    LIBRARY = args.library or LIBRARY
    unittest.main(argv=[sys.argv[0]] + remaining)
