# motor-control-iio-modules

This repository contains the Kernel drivers for the Kv260 Camera Control
Application. The drivers are built on target using DKMS (Dynamic Kernel Module Support) framework.
They are packaged for debian installation as shown in the next section

# Steps to build and install dkms drivers on target

```
  $ sudo apt install dkms
  $ sudo rm -r /usr/src/ar1335-module-0.1
  $ sudo mkdir -p /usr/src/ar1335-module-0.1
  $ git clone  https://gitenterprise.xilinx.com/SOM/ar1335-module
  $ sudo cp ar1335-module/src/* /usr/src/ar1335-module-0.1/
  $ sudo cp ar1335-module/debian/ar1335-module.dkms
/usr/src/ar1335-module-0.1/dkms.conf
  $ sudo dkms add -m ar1335-module -v 0.1
  $ sudo dkms build -m ar1335-module -v 0.1
  $ sudo dkms install -m ar1335-module -v 0.1
```
* `xmutil loadapp kv260-bist` will load the dkms installed drivers on target

# Verify module installation

```
$ modinfo ar1335
filename:       /lib/modules/5.15.0-1027-xilinx-zynqmp/updates/dkms/ar1335.ko
license:        GPL v2
description:    V4L driver for camera sensor AR1335
author:         Anil Kumar Mamidala <amamidal@xilinx.com>
srcversion:     28A54931BD728E00B47D4D7
alias:          of:N*T*Car1335C*
alias:          of:N*T*Car1335
depends:
name:           ar1335
vermagic:       5.15.0-1027-xilinx-zynqmp SMP mod_unload modversions aarch64
```

# License

(C) Copyright 2023 - 2024 Advanced Micro Devices, Inc.\
SPDX-License-Identifier: GPL-2.0
