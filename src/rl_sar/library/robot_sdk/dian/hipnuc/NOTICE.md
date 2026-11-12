# HIPNUC HI91 driver provenance

Copied, with permission to use the deployment reference, from the existing robot's
`custom_quad/include/custom_quad_imu.hpp`, `src/custom_quad_imu.cpp`, and
`src/custom_quad_imu_serial.cpp` on 2026-11-11. The files retain their original
Apache-2.0 identifiers and namespace.

Their HI91 packet/CRC implementation cites HiPNUC (Copyright 2006-2024),
`atom01_deploy/src/imu/src/drivers/hipnuc/hipnuc_dec.{c,h}`, Apache-2.0.
The full license is available at the root of this repository.

SerialReader restores the original termios settings and takes the same device flock
as the existing controller. No native sensor configuration commands are sent.
