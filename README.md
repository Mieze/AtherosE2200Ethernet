AtherosE2200Ethernet
====================

Qualcomm Atheros Killer E2200 driver for OS X

Key Features of the Driver
  - Supports Qualcomm Atheros AR816x, AR817x, Killer E220x, Killer E2400 and Killer E2500.
  - Support for multisegment packets relieving the network stack of unnecessary copy operations when assembling packets for transmission.
  - No-copy receive and transmit. Only small packets are copied on reception because creating a copy is more efficient than allocating a new buffer.
  - TCP, UDP and IPv4 checksum offload (receive and transmit).
  - Support for TCP/IPv6 and UDP/IPv6 checksum offload.
  - Makes use of the chip's TCP Segmentation Offload (TSO) feature with IPv4 and IPv6 in order to reduce CPU load while sending large amounts of data.
  - Fully optimized for Mountain Lion, Mavericks and Yosemite (64bit architecture) but version 1.x.x should work with Lion too, provided you build from source with the 10.7 SDK.
  - Wake on LAN support.
  - VLAN support is implemented but untested as I have no need for it.
  - Supports Energy Efficient Ethernet (EEE).
  - Supports jumbo frames up to 9000 bytes.
  - The driver is published under GPLv2.

Known Issues
  - None.

FAQ
  - Could you add support for AR813x and AR815x? Sorry, no, because I used a different linux driver as the code base than Shailua which doesn't support these chips so that it would be too much work to add support for them.

Installation
  1. Goto /S/L/E and delete ALXEthernet.kext.
  2. Recreate the kernel cache.
  3. Open System Preferences and delete the corresponding network interface, e. g. en0.
  4. Reboot.
  5. Install the new driver and recreate the kernel cache.
  6. Reboot
  7. Open System Preferences again, select Network and check if the new network interface has been created automatically or create it manually now.
  8.Configure the interface.

Troubleshooting
  - Disabling Energy Efficient Ethernet (EEE) may be required to avoid situations in which the link gets lost randomly.
  Make sure you have followed the installation instructions especially when you have issues with certain domains while the others are working fine.
  - Use the debug version to collect log data when trying to track down problems. The kernel log messages can be retrieved with "grep kernel /var/log/system.log" in Terminal. Include the log data when asking for support or giving feedback. I'm an engineer, not a clairvoyant.
  - Check your BIOS settings. You might want to disable Network Boot and the UEFI Network Stack as these can interfere with the driver.
  - Double check that you have removed any ALXEthernet.kext from your system because it could prevent the driver from working properly.
  - Verify your bootloader configuration, in particular the kernel flags. Avoid using npci=0x2000 or npci=0x3000. 
  - In Terminal run netstat -s in order to display network statistics. Carefully examine the data for any unusual activity like a high number of packets with bad IP header checksums, etc.
  - In case auto-configuration of the link layer connection doesn't work it might be necessary to select the medium manually in System Preferences under Network for the interface.
  - Use Wireshark to create a packet dump in order to collect diagnostic information.
  - Keep in mind that there are many manufacturers of network equipment. Although Ethernet is an IEEE standard, different implementations may show different behavior causing incompatibilities. In case you are having trouble try a different switch or a different cable.

Changelog
 - Version 2.2.1 (2017-02-01)
   - Enabled polled receive mode.
 - Version 2.2.0 (2016-09-17)
    - Added support for Killer E2500 and jumbo frames.
 - Version 2.1.0d1 (2015-11-29)
    - Supports Energy Efficient Ethernet (EEE).
    - Added support for Killer E2400.
 - Version 2.0.1 (2015-08-12)
    - Improved flow control support in 100MBit mode.
 - Version 2.0.0 (2015-04-21)
    - Uses Apple's private driver interface introduced with 10.8.
    - Supports packet scheduling with QFQ.
    - Please note that 2.0.0 is identical to 2.0.0d1. Only the version number has changed.
 - Version 1.0.1 (2015-03-01)
    - Reworked media selection and reporting.
    - Improved flow control support.
    - Resolved the NIC disabled by BIOS issue. 
 - Version 1.0.0 (2014-09-22)
    - Final release.
 - Version 1.0.0d7 (2014-08-18)
    - Fixed Wake on LAN.
 - Version 1.0.0d6 (2014-08-16)
    - Detects situations when the BIOS left the NIC disabled and outputs an error messages.
    - Small optimizations and improved error handling.
 - Version 1.0.0d5 (2014-08-13)
    - Removed the mbuf_pullup() call in outputPacket() as the NIC seems to accept packets with noncontiguous headers.
 - Version 1.0.0d4 (2014-08-12)
    - Fixed TSO with IPv4 and IPv6.
 - Version 1.0.0d3 (2014-08-10)
    - Added support for TCP and UDP checksum offload over IPv6.
    - Cleaned up the code and improved error handling.
