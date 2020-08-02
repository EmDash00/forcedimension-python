"""
.. module::bindings
   :platform: Windows, Unix
   :synopsis: libdhd Python bindings

.. moduleauthor:: Drason Chow <drasonchow@gmail.com>
"""

__all__ = ['constants', 'standard', 'expert']


import forcedimension.runtime as runtime

# Load the runtime from the backend
_libdhd = runtime.load("libdhd")
