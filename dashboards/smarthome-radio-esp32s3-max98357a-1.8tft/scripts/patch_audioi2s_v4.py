"""Install the reviewed live-MP3 reserve patch into ESP32-audioI2S 4.0.0.

Arduino IDE: python3 scripts/patch_audioi2s_v4.py --library /path/to/ESP32-audioI2S
The path may point at the library root or its src directory. No downloaded code
is trusted: both source files must match the exact reviewed upstream hashes.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys


MANIFEST_PATH = Path(__file__).resolve().parent / "audioi2s_v4" / "patch.json"
MANIFEST = json.loads(MANIFEST_PATH.read_text())
AUDIO_COMMIT = MANIFEST["upstream_commit"]
ORIGINAL_SHA256 = {name: item["original_sha256"] for name, item in MANIFEST["files"].items()}
PATCHED_SHA256 = {name: item["patched_sha256"] for name, item in MANIFEST["files"].items()}
HELPER_NAME = Path(MANIFEST["helper"]["destination"]).name
BACKUP_SUFFIX = MANIFEST["backup_suffix"]
HELPER = MANIFEST_PATH.parent / MANIFEST["helper"]["source"]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def replace_once(data, old, new):
    old, new = old.encode("utf-8"), new.encode("utf-8")
    if data.count(old) != 1:
        raise RuntimeError("Reviewed audio patch insertion point is missing or ambiguous")
    return data.replace(old, new, 1)


def patched_source(name, original):
    """Produce a deterministic patch only from the reviewed pristine bytes."""
    if digest(original) != ORIGINAL_SHA256[name]:
        raise RuntimeError("Unreviewed pristine " + name + "; refusing to patch")
    data = original
    for replacement in MANIFEST["files"][name]["replacements"]:
        data = replace_once(data, replacement["old"], replacement["new"])
    if digest(data) != PATCHED_SHA256[name]:
        raise RuntimeError("Reviewed patch manifest result hash mismatch: " + name)
    return data


def library_paths(library):
    root = Path(library).expanduser().resolve()
    if root.name == "src" and (root.parent / "library.properties").is_file():
        root = root.parent
    properties = root / "library.properties"
    if not properties.is_file():
        raise RuntimeError("Expected an ESP32-audioI2S library containing library.properties: " + str(root))
    # Match horizontal whitespace only: an empty version must not swallow the
    # next property line. UTF-8 BOMs and Windows line endings are harmless.
    versions = [value.strip() for value in re.findall(
        r"^[ \t\ufeff]*version[ \t]*=[ \t]*([^\r\n]*)",
        properties.read_text(encoding="utf-8-sig"), re.MULTILINE)]
    if versions != [MANIFEST["library_version"]]:
        found = ", ".join(repr(value) for value in versions) if versions else "<missing>"
        raise RuntimeError("Expected ESP32-audioI2S version " + MANIFEST["library_version"]
                           + "; found " + found + " in " + str(properties) + "; refusing to patch")
    return root, root / "src"


def patch_library(library, check=False):
    """Return 'pristine', 'patched', or 'already patched'; check never writes.

    Backups are retained beside each source file. All files are validated before
    any mutation. A partial installation is rejected, preserving the evidence
    and original backups rather than guessing how to repair it.
    """
    _, source_dir = library_paths(library)
    helper = HELPER.read_bytes()
    if digest(helper) != MANIFEST["helper"]["sha256"]:
        raise RuntimeError("Reviewed patch helper hash mismatch; refusing to patch")
    sources = {name: (source_dir / name).read_bytes() for name in ORIGINAL_SHA256}
    backups = {name: source_dir / (name + BACKUP_SUFFIX) for name in ORIGINAL_SHA256}
    pristine = all(digest(data) == ORIGINAL_SHA256[name] for name, data in sources.items())
    backup_exists = [path.exists() for path in backups.values()]
    installed_helper = source_dir / HELPER_NAME

    if pristine:
        if any(backup_exists) or installed_helper.exists():
            raise RuntimeError("Partial or unexpected audio patch installation; refusing to patch")
        patched = {name: patched_source(name, data) for name, data in sources.items()}
        if check:
            return "pristine"
        attempted_backups = []
        attempted_sources = []
        attempted_helper = False
        try:
            for name, data in sources.items():
                attempted_backups.append(backups[name])
                backups[name].write_bytes(data)
            attempted_helper = True
            installed_helper.write_bytes(helper)
            for name, data in patched.items():
                attempted_sources.append(name)
                (source_dir / name).write_bytes(data)
        except OSError as error:
            rollback_errors = []
            for name in reversed(attempted_sources):
                try:
                    (source_dir / name).write_bytes(sources[name])
                except OSError as rollback_error:
                    rollback_errors.append(str(rollback_error))
            if attempted_helper:
                try:
                    if installed_helper.exists():
                        installed_helper.unlink()
                except OSError as rollback_error:
                    rollback_errors.append(str(rollback_error))
            # Preserve pristine backups if restoring any source failed.
            if not rollback_errors:
                for backup in attempted_backups:
                    try:
                        if backup.exists():
                            backup.unlink()
                    except OSError as rollback_error:
                        rollback_errors.append(str(rollback_error))
            message = "Audio patch write failed: " + str(error)
            if rollback_errors:
                message += "; rollback incomplete (keep original backups): " + "; ".join(rollback_errors)
            else:
                message += "; original library restored"
            raise RuntimeError(message) from error
        return "patched"

    if not all(backup_exists) or not installed_helper.is_file():
        raise RuntimeError("Unknown or partial audio source modification; refusing to patch")
    originals = {name: path.read_bytes() for name, path in backups.items()}
    for name, original in originals.items():
        if digest(original) != ORIGINAL_SHA256[name]:
            raise RuntimeError("Audio patch backup hash mismatch; refusing to patch " + name)
        if sources[name] != patched_source(name, original):
            raise RuntimeError("Unknown or partial audio source modification; refusing to patch " + name)
    if installed_helper.read_bytes() != helper:
        raise RuntimeError("Audio patch helper mismatch; refusing to patch")
    return "already patched"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True, type=Path, help="ESP32-audioI2S 4.0.0 library root (or src)")
    parser.add_argument("--check", action="store_true", help="verify sources and installation state without writing")
    args = parser.parse_args(argv)
    try:
        result = patch_library(args.library, check=args.check)
    except (OSError, RuntimeError) as error:
        print("ESP32-audioI2S reserve patch: " + str(error), file=sys.stderr)
        return 1
    print("ESP32-audioI2S 4.0.0 reserve patch: " + result + (" (check only)" if args.check else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
