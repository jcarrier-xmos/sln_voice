.. _sln_voice_system_prerequisites_programming:

#############
Prerequisites
#############

It is recommended that you download and install the latest release of the `XTC Tools <https://www.xmos.com/software/tools/>`_.  XTC Tools 15.1.4 or newer are required for building, running, flashing and debugging the example applications.

`CMake 3.21 <https://cmake.org/download/>`_ or newer is also required for building the example applications.

*******
Windows
*******

A standard C/C++ compiler is required to build applications for the host PC.  Windows users may use `Build Tools for Visual Studio <https://docs.microsoft.com/en-us/cpp/build/building-on-the-command-line?view=msvc-170#download-and-install-the-tools>`__ command-line interface.

XCORE-VOICE host build should also work using other Windows GNU development environments like GNU Make, MinGW or Cygwin.

libusb
======

The DFU feature of XCORE-VOICE requires `dfu-util <https://dfu-util.sourceforge.net/>`_ which requires ``libusb v1.0``. ``libusb`` requires the installation of a driver for use on a Windows host. Driver installation should be done using a third-party installation tool like `Zadig <https://zadig.akeo.ie/>`_.

*****
macOS
*****

A standard C/C++ compiler is required to build applications for the host PC.  Mac users may use the Xcode command-line tools.
