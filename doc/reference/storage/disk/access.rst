.. _disk_access_api:

Disk Access
###########

Overview
********

The disk access API provides access to storage devices.

SD Card support
***************

Zephyr has support for some SD card controllers and support for interfacing
SD cards via SPI. These drivers use disk driver interface and a file system
can access the SD cards via disk access API.
Both standard and high-capacity SD cards are supported.

.. note:: The system does not support inserting or removing cards while the
   system is running. The cards must be present at boot and must not be
   removed. This may be fixed in future releases.

   FAT filesystems are not power safe so the filesystem may become
   corrupted if power is lost or if the card is removed.

SD Card support via SPI
=======================

Example devicetree fragment below shows how to add SD card node to ``spi1``
interface. Example uses pin ``PA27`` for chip select, and runs the SPI bus
at 24 MHz once the SD card has been initialized:

.. code-block:: devicetree

    &spi1 {
            status = "okay";
            cs-gpios = <&porta 27 GPIO_ACTIVE_LOW>;

            sdhc0: sdhc@0 {
                    compatible = "zephyr,mmc-spi-slot";
                    reg = <0>;
                    status = "okay";
                    label = "SDHC0";
                    spi-max-frequency = <24000000>;
            };
    };

The SD card will be automatically detected and initialized by the
filesystem driver when the board boots.

To read and write files and directories, see the :ref:`file_system_api` in
:zephyr_file:`include/fs.h` such as :c:func:`fs_open()`,
:c:func:`fs_read()`, and :c:func:`fs_write()`.

Disk Access API Configuration Options
*************************************

Related configuration options:

* :option:`CONFIG_DISK_ACCESS`

API Reference
*************

.. doxygengroup:: disk_access_interface

Disk Driver Configuration Options
*********************************

Related driver configuration options:

* :option:`CONFIG_DISK_DRIVERS`

Disk Driver Interface
*********************

.. doxygengroup:: disk_driver_interface
