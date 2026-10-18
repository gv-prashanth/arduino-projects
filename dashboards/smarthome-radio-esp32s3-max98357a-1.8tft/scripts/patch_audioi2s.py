"""Apply the reviewed URL-copy fix to the exact pinned ESP32-audioI2S source.

PlatformIO installs dependencies before running this pre-build script. An
unexpected dependency revision or local modification fails the build instead of
silently applying the patch to an unreviewed implementation.
"""

import hashlib
from pathlib import Path


AUDIO_COMMIT = "928c420d49fce2a09fa91f490b9fcabed6447c67"
ORIGINAL_SHA256 = "ffde34db30d3fbae7a9ac45b8b9fb94d6b89cdd06192a811cd3e57138cd2afbf"
PATCHED_SHA256 = "b41dfb979766f101f39779b009c7d89248ba10cfb6a4bf5d15ac48888d0a5931"
OLD_COPY = b"memcpy(extension, h_host + pos_slash, extLen);"
NEW_COPY = b"memcpy(extension, h_host + pos_slash, strlen(h_host + pos_slash) + 1);"


def patch_audio_source(source_path):
    """Return True when patched; return False for the exact already-patched file."""
    source_path = Path(source_path)
    data = source_path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest == PATCHED_SHA256:
        return False
    if digest != ORIGINAL_SHA256:
        raise RuntimeError(
            "ESP32-audioI2S Audio.cpp does not match pinned commit "
            + AUDIO_COMMIT
            + " (or its reviewed patch); refusing to patch SHA256 "
            + digest
        )
    if data.count(OLD_COPY) != 2:
        raise RuntimeError("Expected both connecttohost/httpPrint URL-copy sites")
    patched = data.replace(OLD_COPY, NEW_COPY)
    if hashlib.sha256(patched).hexdigest() != PATCHED_SHA256:
        raise RuntimeError("ESP32-audioI2S patch result failed its SHA256 check")
    source_path.write_bytes(patched)
    return True


if "Import" in globals():
    Import("env")
    audio_source = (
        Path(env.subst("$PROJECT_LIBDEPS_DIR"))
        / env.subst("$PIOENV")
        / "ESP32-audioI2S"
        / "src"
        / "Audio.cpp"
    )
    if patch_audio_source(audio_source):
        print("Applied pinned ESP32-audioI2S URL-copy safety patch (2 sites)")
