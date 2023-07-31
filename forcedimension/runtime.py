import ctypes
import glob
import os
import platform
import re
import sys
import typing
from functools import lru_cache

from forcedimension.containers import VersionTuple

VERSION_TARGET_FULL = "3.14.0-1681794874"
VERSION_TARGET = VERSION_TARGET_FULL.partition("-")[0]


def version_tuple(version_string: str):
    search_pattern = r"(\d+)\.(\d+)\.(\d+)-(\d+)"

    if ((res := re.search(search_pattern, version_string)) is not None):
        if (len(res.groups()) != 4):
            raise ValueError("Invalid version string.")
    else:
        raise ValueError("Invalid version string.")

    return VersionTuple(*(int(v) for v in res.groups()))


@lru_cache
def load(lib_name, search_dirs=(), silent=False):

    try:
        if __sphinx_build__:  # type: ignore
            from mock import Mock
            return Mock()
    except NameError:
        pass

    if (sys.platform == "win32"):
        lib_ext = ".dll"
        lib_dir = "bin"

        if platform.architecture()[0] == "64bit":
            lib_name = lib_name[3:] + "64"
    else:
        lib_ext = ".so"
        lib_dir = "lib"

    search_dirs = list(search_dirs)

    if sys.platform == "win32":
        search_dirs.append(
            glob.glob(
                "{}\\sdk-*".format(
                    os.path.join(
                        "c:",
                        os.sep,
                        "Program Files",
                        "Force Dimension"
                    )
                )
            )[0]
        )
    elif sys.platform.startswith("linux"):
        search_dirs.extend([
            "/usr/local",
            "/usr",

        ])

        if (os.environ.get("FORCEDIM_SDK")):
            search_dirs.append(
                os.path.realpath(os.path.join(
                    typing.cast(str, os.environ.get("FORCEDIM_SDK")),
                    "lib",
                    "release",
                    "lin-x86_64-gcc"))
            )
    else:
        if not silent:
            sys.stderr.write(
                "Unsupported platform. Only Windows and Linux is supported."
            )

        return None

    for directory in search_dirs:

        if directory is None:
            continue

        if not os.path.isdir(directory):
            continue

        directory = os.path.abspath(directory)
        lib_path = os.path.join(directory, lib_dir, lib_name + lib_ext)

        if (os.path.isfile(lib_path)):
            if sys.platform == "win32":

                path = os.getenv("PATH")
                if (path is not None and directory not in path):
                    os.environ["PATH"] = f"{path};{directory}"

            try:
                lib = ctypes.CDLL(lib_path)
            except OSError:
                if silent:
                    break
                else:
                    raise RuntimeError("Library could not be loaded. Do you"
                                       "have missing dependencies?\n"
                                       "Ensure you have libusb-1.")

            major = ctypes.c_int()
            minor = ctypes.c_int()
            release = ctypes.c_int()
            revision = ctypes.c_int()

            lib.dhdGetSDKVersion(major, minor, release, revision)

            version = VersionTuple(
                major.value,
                minor.value,
                release.value,
                revision.value
            )

            if (version < version_tuple(VERSION_TARGET_FULL)):
                if not silent:
                    sys.stderr.write(
                        "Invalid version. v{}.{}.{}-{} found "
                        "but {} is required.\n".format(
                            *version,
                            VERSION_TARGET_FULL
                        )
                    )

                return None

            return lib
    if (not silent):
        sys.stderr.write(
            f"Could not find {lib_name}. Is it installed?\n"
        )
    return None
