"""Exercise the native PowerShell installer against exact audioI2S 4.0.0 files.

Run with --powershell PATH (or POWERSHELL) and --library PATH (or
AUDIOI2S_V4_LIBRARY). Without an available PowerShell runtime or source fixture,
the integration suite skips. Source fixtures may be pristine or already patched
with verified original backups. All mutations occur in temporary directories.
"""

import argparse
import hashlib
import importlib.util
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


class PowerShellInstallerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not POWERSHELL:
            raise unittest.SkipTest("Pass --powershell PATH or POWERSHELL for the native installer tests")
        if not LIBRARY:
            raise unittest.SkipTest("Pass --library PATH or AUDIOI2S_V4_LIBRARY for exact 4.0.0 source tests")
        _, source = patch.library_paths(LIBRARY)
        cls.originals = {}
        for name, expected in patch.ORIGINAL_SHA256.items():
            candidate = source / name
            if patch.digest(candidate.read_bytes()) != expected:
                candidate = source / (name + patch.BACKUP_SUFFIX)
            data = candidate.read_bytes()
            if patch.digest(data) != expected:
                raise RuntimeError("Unreviewed source fixture: " + name)
            cls.originals[name] = data

    def fixture(self, directory, name="ESP32 audio library \u00fc"):
        root = Path(directory) / name
        (root / "src").mkdir(parents=True)
        (root / "library.properties").write_text("name=ESP32-audioI2S-master\nversion=4.0.0\n")
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
                self.assertEqual((root / "src" / (name + patch.BACKUP_SUFFIX)).read_bytes(), original)
                self.assertEqual(hashlib.sha256((root / "src" / name).read_bytes()).hexdigest(),
                                 patch.PATCHED_SHA256[name])
            self.assertEqual((root / "src" / patch.HELPER_NAME).read_bytes(), patch.HELPER.read_bytes())
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
                (root / "library.properties").write_bytes(properties)
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
                    self.assertIn("Expected ESP32-audioI2S version 4.0.0", error)
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
                    (root / "library.properties").write_text("version=4.0.1\n")
                elif case == "source":
                    with (source / "Audio.cpp").open("ab") as handle:
                        handle.write(b"\n// unreviewed modification\n")
                elif case == "partial source":
                    (source / "Audio.h").write_bytes(patch.patched_source("Audio.h", self.originals["Audio.h"]))
                elif case == "unexpected helper":
                    (source / patch.HELPER_NAME).write_bytes(b"unexpected helper")
                elif case == "unexpected backup":
                    (source / ("Audio.h" + patch.BACKUP_SUFFIX)).write_bytes(b"unexpected backup")
                else:
                    self.run_installer(root)
                    target = {
                        "installed helper": source / patch.HELPER_NAME,
                        "original backup": source / ("Audio.h" + patch.BACKUP_SUFFIX),
                        "missing backup": source / ("Audio.cpp" + patch.BACKUP_SUFFIX),
                    }[case]
                    if case == "missing backup":
                        target.unlink()
                    else:
                        target.write_bytes(b"unexpected modification")
                before = self.snapshot(root)
                for check in (True, False):
                    self.assertIn("ESP32-audioI2S reserve patch:",
                                  self.run_installer(root, check=check, expected=1).stderr)
                    self.assertEqual(self.snapshot(root), before)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--powershell")
    parser.add_argument("--library")
    args, remaining = parser.parse_known_args()
    POWERSHELL = args.powershell or POWERSHELL
    LIBRARY = args.library or LIBRARY
    unittest.main(argv=[sys.argv[0]] + remaining)
