.. _ram_access:

RAM access
###########

Overview
********

A simple sample that can be used with any :ref:`supported board <boards>` and
runs simple RAM access instructions.

Building and Running
********************

This application can be built and executed on QEMU as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/ram_access
   :host-os: unix
   :board: qemu_x86
   :goals: run
   :compact:

To build for another board, change "qemu_x86" above to that board's name.

Sample Output
=============

.. code-block:: console

    tbias 57

    Test: direct 512 NOPs
        Take 1276 cycles

    Test: loop 512 NOPs
        Take 2073 cycles

    Test: loop RD 16384 bytes from cached DRAM
        Take 42144 cycles, throughput 77.75 MB/s

    Test: loop RD 16384 bytes from non-cached DRAM
        Take 98464 cycles, throughput 33.27 MB/s

    Test: loop WR 16384 bytes to cached DRAM
        Take 24686 cycles, throughput 132.73 MB/s

    Test: loop WR 16384 bytes to non-cached DRAM
        Take 24644 cycles, throughput 132.96 MB/s

Exit QEMU by pressing :kbd:`CTRL+A` :kbd:`x`.
