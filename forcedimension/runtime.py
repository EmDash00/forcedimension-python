import ctypes
import os
import sys
import re

VERSION_TARGET_FULL = "3.9.1-3454"
VERSION_TARGET = VERSION_TARGET_FULL.partition("-")[0]


def version_tuple(version_string):
    res = re.search("(\d+)\.(\d+)\.(\d+)\-(\d+)", version_string)

    if (res.groups()):
        if (len(res.groups()) != 4):
            raise ValueError("Invalid version string.")
    else:
        raise ValueError("Invalid version string.")

    return tuple(int(v) for v in res.groups())


def load(lib_name, search_dirs=(), silent=False):
    if (sys.platform == "win32"):
        lib_ext = ".dll"
        lib_dir = "bin"
    else:
        lib_ext = ".so"
        lib_dir = "lib"

    search_dirs = list(search_dirs)

    if sys.platform == "win32":
        search_dirs.append(os.path.join(
            "c:",
            "Program Files",
            "ForceDimension",
            "sdk-{}".format(VERSION_TARGET)))
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

    search_path = os.environ.get('PATH', '')
    for directory in search_dirs:

        if directory is None:
            continue

        if not os.path.isdir(directory):
            continue

        directory = os.path.abspath(directory)
        lib_path = os.path.join(directory, lib_dir, lib_name + lib_ext)

        if (os.path.isfile(lib_path)):
            if sys.platform == "win32":
                os.environ["PATH"] = search_path + ";" + directory

            try:
                lib = ctypes.CDLL(lib_path)
            except OSError:
                if silent:
                    break
                else:
                    raise RuntimeError("Library could not be loaded. Do you"
                                       "have missing dependencies?\n"
                                       "Ensure you have libusb-1.")

            if (lib_name == "libdhd"):
                major = ctypes.c_int()
                minor = ctypes.c_int()
                release = ctypes.c_int()
                revision = ctypes.c_int()

                lib.dhdGetSDKVersion(ctypes.byref(major),
                                     ctypes.byref(minor),
                                     ctypes.byref(release),
                                     ctypes.byref(revision))

                version = (
                    major.value,
                    minor.value,
                    release.value,
                    revision.value
                )

                if (not version == version_tuple(VERSION_TARGET_FULL)):
                    if not silent:
                        sys.stderr.write(
                            "Invalid version {}.{}.{}-{} found"
                            "but {} is required.\n".format(
                                *version,
                                VERSION_TARGET_FULL
                            )
                        )

                    return None

            return lib
    if (not silent):
        sys.stderr.write("Could not find {}.\n".format(lib_name))
    return None
