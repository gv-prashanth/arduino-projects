"""Exercise the native PowerShell installer against exact audioI2S 4.x files.

Run with --powershell PATH (or POWERSHELL) and --library PATH (or
AUDIOI2S_V4_LIBRARY). Without an available PowerShell runtime or source fixture,
the integration suite skips. Source fixtures may be pristine or already patched
with verified original backups. All mutations occur in temporary directories.
Pass --other-library PATH (or AUDIOI2S_V4_OTHER_LIBRARY) to test mixed versions.
"""

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "scripts" / "patch_audioi2s_v4.ps1"
PYTHON_SCRIPT = PROJECT / "scripts" / "patch_audioi2s_v4.py"
module_spec = importlib.util.spec_from_file_location("audio_v4_patch", PYTHON_SCRIPT)
patch = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(patch)
POWERSHELL = os.environ.get("POWERSHELL") or shutil.which("pwsh") or shutil.which("powershell")
LIBRARY = os.environ.get("AUDIOI2S_V4_LIBRARY")
OTHER_LIBRARY = os.environ.get("AUDIOI2S_V4_OTHER_LIBRARY")


class PowerShellInstallerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not POWERSHELL:
            raise unittest.SkipTest("Pass --powershell PATH or POWERSHELL for the native installer tests")
        if not LIBRARY:
            raise unittest.SkipTest("Pass --library PATH or AUDIOI2S_V4_LIBRARY for exact 4.x source tests")
        _, source, cls.manifest = patch.library_info(LIBRARY)
        cls.version = cls.manifest["library_version"]
        cls.patched_hashes = {name: item["patched_sha256"] for name, item in cls.manifest["files"].items()}
        cls.backup_suffix = cls.manifest["backup_suffix"]
        cls.helper_name = Path(cls.manifest["helper"]["destination"]).name
        cls.helper = PROJECT / "scripts" / "audioi2s_v4" / cls.manifest["helper"]["source"]
        cls.catalog = json.loads((PROJECT / "scripts" / "audioi2s_v4" / "catalog.json").read_text())
        cls.originals = cls.read_originals(source, cls.manifest)
        cls.other_originals = None
        if OTHER_LIBRARY:
            _, other_source, cls.other_manifest = patch.library_info(OTHER_LIBRARY)
            if cls.other_manifest["library_version"] == cls.version:
                raise RuntimeError("--other-library must select a different reviewed profile")
            cls.other_originals = cls.read_originals(other_source, cls.other_manifest)

    @staticmethod
    def read_originals(source, manifest):
        originals = {}
        for name, item in manifest["files"].items():
            expected = item["original_sha256"]
            candidate = source / name
            if patch.digest(candidate.read_bytes()) != expected:
                candidate = source / (name + manifest["backup_suffix"])
            data = candidate.read_bytes()
            if patch.digest(data) != expected:
                raise RuntimeError("Unreviewed source fixture: " + name)
            originals[name] = data
        return originals

    def fixture(self, directory, name="ESP32 audio library \u00fc"):
        root = Path(directory) / name
        (root / "src").mkdir(parents=True)
        (root / "library.properties").write_text("name=ESP32-audioI2S-master\nversion=" + self.version + "\n")
        for name, data in self.originals.items():
            (root / "src" / name).write_bytes(data)
        return root

    @staticmethod
    def snapshot(root):
        return {str(path.relative_to(root)): path.read_bytes() for path in root.rglob("*") if path.is_file()}

    def run_installer(self, library, check=False, expected=0):
        command = [str(POWERSHELL), "-NoLogo", "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", str(SCRIPT),
                   "-LibraryPath", str(library)]
        if check:
            command.append("-Check")
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        return result

    def test_check_patch_backups_idempotence_and_src_path(self):
        with tempfile.TemporaryDirectory(prefix="gts powershell ") as directory:
            root = self.fixture(directory)
            before = self.snapshot(root)
            self.assertIn("pristine (check only)", self.run_installer(root, check=True).stdout)
            self.assertEqual(self.snapshot(root), before)
            self.assertIn("patched;", self.run_installer(root).stdout)
            for name, original in self.originals.items():
                self.assertEqual((root / "src" / (name + self.backup_suffix)).read_bytes(), original)
                self.assertEqual(hashlib.sha256((root / "src" / name).read_bytes()).hexdigest(),
                                 self.patched_hashes[name])
            self.assertEqual((root / "src" / self.helper_name).read_bytes(), self.helper.read_bytes())
            after = self.snapshot(root)
            self.assertIn("already patched", self.run_installer(root).stdout)
            self.assertIn("already patched", self.run_installer(root, check=True).stdout)
            self.assertIn("already patched", self.run_installer(root / "src", check=True).stdout)
            self.assertEqual(self.snapshot(root), after)

    def test_python_and_powershell_install_identical_bytes(self):
        with tempfile.TemporaryDirectory(prefix="gts powershell ") as directory:
            native = self.fixture(directory, "native")
            python = self.fixture(directory, "python")
            self.run_installer(native)
            result = subprocess.run([sys.executable, str(PYTHON_SCRIPT), "--library", str(python)],
                                    capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(self.snapshot(native), self.snapshot(python))

    def test_version_formatting_variants_are_accepted_without_writing(self):
        cases = {
            "CRLF": b"name=test\r\nversion=4.0.0\r\n",
            "UTF8 BOM": b"\xef\xbb\xbfversion=4.0.0\r\nname=test\r\n",
            "BOM before version": b"name=test\r\n\xef\xbb\xbfversion=4.0.0\r\n",
            "leading spaces": b"  version=4.0.0\r\n",
            "leading tab": b"\tversion=4.0.0\r\n",
            "spaces around equals": b"version \t= \t4.0.0 \t\r\n",
            "no final newline": b"version=4.0.0",
        }
        for name, properties in cases.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory(prefix="gts powershell ") as directory:
                root = self.fixture(directory)
                (root / "library.properties").write_bytes(properties.replace(b"4.0.0", self.version.encode("ascii")))
                before = self.snapshot(root)
                self.assertIn("pristine (check only)", self.run_installer(root, check=True).stdout)
                self.assertEqual(self.snapshot(root), before)

    def test_version_errors_report_found_value_and_full_properties_path_without_writing(self):
        cases = {
            "missing": (b"name=test\r\n", "<missing>"),
            "empty": (b"version=\r\nname=test\r\n", "<empty>"),
            "whitespace only": (b" \tversion \t= \t \r\n", "<empty>"),
            "duplicate": (b"version=4.0.0\r\nversion=4.0.1\r\n", "'4.0.0', '4.0.1'"),
            "different": (b"version=3.4.4\r\n", "'3.4.4'"),
            "unsupported": (b"version=4.0.2\r\n", "'4.0.2'"),
            "suffix": (b"version=4.0.0-master\r\n", "'4.0.0-master'"),
        }
        for name, (properties, found) in cases.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory(prefix="gts powershell ") as directory:
                root = self.fixture(directory)
                properties_path = root / "library.properties"
                properties_path.write_bytes(properties)
                before = self.snapshot(root)
                for check in (True, False):
                    error = self.run_installer(root, check=check, expected=1).stderr
                    self.assertIn("Expected ESP32-audioI2S version ", error)
                    for version in self.catalog["profiles"]:
                        self.assertIn(version, error)
                    self.assertIn("found " + found, error)
                    self.assertIn(str(properties_path), error)
                    self.assertIn("refusing to patch", error)
                    self.assertEqual(self.snapshot(root), before)

    def test_wrong_version_and_unknown_or_partial_installations_are_read_only(self):
        cases = ["version", "source", "partial source", "unexpected helper", "unexpected backup",
                 "installed helper", "original backup", "missing backup"]
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory(prefix="gts powershell ") as directory:
                root = self.fixture(directory)
                source = root / "src"
                if case == "version":
                    (root / "library.properties").write_text("version=4.0.2\n")
                elif case == "source":
                    with (source / "Audio.cpp").open("ab") as handle:
                        handle.write(b"\n// unreviewed modification\n")
                elif case == "partial source":
                    (source / "Audio.h").write_bytes(patch.patched_source("Audio.h", self.originals["Audio.h"], self.manifest))
                elif case == "unexpected helper":
                    (source / self.helper_name).write_bytes(b"unexpected helper")
                elif case == "unexpected backup":
                    (source / ("Audio.h" + self.backup_suffix)).write_bytes(b"unexpected backup")
                else:
                    self.run_installer(root)
                    target = {
                        "installed helper": source / self.helper_name,
                        "original backup": source / ("Audio.h" + self.backup_suffix),
                        "missing backup": source / ("Audio.cpp" + self.backup_suffix),
                    }[case]
                    if case == "missing backup":
                        target.unlink()
                    else:
                        target.write_bytes(b"unexpected modification")
                before = self.snapshot(root)
                for check in (True, False):
                    error = self.run_installer(root, check=check, expected=1).stderr
                    self.assertIn("ESP32-audioI2S reserve patch:", error)
                    if case in ("source", "partial source"):
                        for name in self.originals:
                            self.assertIn(name + "=" + patch.digest((source / name).read_bytes()), error)
                    self.assertEqual(self.snapshot(root), before)

    def test_supported_version_metadata_cannot_select_other_source_hashes(self):
        other_version = next(version for version in self.catalog["profiles"] if version != self.version)
        for installed in (False, True):
            with self.subTest(installed=installed), tempfile.TemporaryDirectory(prefix="gts powershell ") as directory:
                root = self.fixture(directory)
                if installed:
                    self.run_installer(root)
                (root / "library.properties").write_text("version=" + other_version + "\n")
                before = self.snapshot(root)
                for check in (True, False):
                    error = self.run_installer(root, check=check, expected=1).stderr
                    self.assertIn("Unknown or partial source modification for version " + other_version, error)
                    for name in self.originals:
                        self.assertIn(name + "=" + patch.digest((root / "src" / name).read_bytes()), error)
                    self.assertEqual(self.snapshot(root), before)

    def test_mixed_version_sources_and_wrong_profile_backups_are_rejected(self):
        if self.other_originals is None:
            self.skipTest("Pass --other-library PATH or AUDIOI2S_V4_OTHER_LIBRARY for mixed-version checks")
        for case in ("Audio.h", "Audio.cpp", "backup"):
            with self.subTest(case=case), tempfile.TemporaryDirectory(prefix="gts powershell ") as directory:
                root = self.fixture(directory)
                source = root / "src"
                if case == "backup":
                    self.run_installer(root)
                    (source / ("Audio.h" + self.backup_suffix)).write_bytes(self.other_originals["Audio.h"])
                else:
                    (source / case).write_bytes(self.other_originals[case])
                before = self.snapshot(root)
                for check in (True, False):
                    error = self.run_installer(root, check=check, expected=1).stderr
                    if case == "backup":
                        self.assertIn("Original backup mismatch: Audio.h", error)
                    else:
                        self.assertIn("Unknown or partial source modification for version " + self.version, error)
                        for name in self.originals:
                            self.assertIn(name + "=" + patch.digest((source / name).read_bytes()), error)
                    self.assertEqual(self.snapshot(root), before)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--powershell")
    parser.add_argument("--library")
    parser.add_argument("--other-library")
    args, remaining = parser.parse_known_args()
    POWERSHELL = args.powershell or POWERSHELL
    LIBRARY = args.library or LIBRARY
    OTHER_LIBRARY = args.other_library or OTHER_LIBRARY
    unittest.main(argv=[sys.argv[0]] + remaining)
