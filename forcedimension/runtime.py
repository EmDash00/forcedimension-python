import ctypes
from functools import lru_cache
import glob
import os
import platform
import re
import sys
from typing import NamedTuple

VERSION_TARGET_FULL = "3.14.0-1681794874)"
VERSION_TARGET = VERSION_TARGET_FULL.partition("-")[0]


class _Version(NamedTuple):
    major: int
    minor: int
    release: int
    revision: int = 0

    def __eq__(self, v):
        if self == v:
            return True
        else:
            return False

    def __lt__(self, v):
        if self.major < v.major:
            return True

        if self.major == v.major:
            if self.minor < v.minor:
                return True
            else:
                return False
        else:
            return False

    def __le__(self, v):
        if self.major <= v.major:
            print('hi')
            if self.minor <= v.minor:
                print('hi')
                return True
            else:
                return False
        else:
            return False

    def __ge__(self, v):
        if self.major >= v.major:
            if self.minor >= v.minor:
                return True
            else:
                return False
        else:
            return False

    def __gt__(self, v):
        if self.major > v.major:
            return True

        if self.major == v.major:
            if self.minor > v.minor:
                return True
            else:
                return False
        else:
            return False

    def __ne__(self, v):
        if self != v:
            return True
        else:
            return False


def version_tuple(version_string: str):
    res = re.search(r"(\d+)\.(\d+)\.(\d+)-(\d+)", version_string)

    if (res.groups() is not None):
        if (len(res.groups()) != 4):
            raise ValueError("Invalid version string.")
    else:
        raise ValueError("Invalid version string.")

    return _Version(*(int(v) for v in res.groups()))


@lru_cache
def load(lib_name, search_dirs=(), silent=False):
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
                    os.environ.get("FORCEDIM_SDK"),
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
                    os.environ["PATH"] = "{};{}".format(path, directory)

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

            lib.dhdGetSDKVersion(ctypes.byref(major),
                                 ctypes.byref(minor),
                                 ctypes.byref(release),
                                 ctypes.byref(revision))

            version = _Version(
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
            "Could not find {}. Is it installed?\n".format(lib_name))
    return None
