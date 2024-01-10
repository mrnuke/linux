.. SPDX-License-Identifier: GPL-2.0

===================================================
PPE Driver for Qualcomm PPE Ethernet Network Family
===================================================

Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.

Author: Lei Wei <quic_leiwei@quicinc.com>


Contents
========

- `Overview`_
- `PPE Driver Supported SoCs`_
- `Enabling the Driver`_
- `Supported Features`_
- `PPE MAC and UNIPHY Interface`_
- `Debugging`_
- `APIs used by PPE EDMA driver`_
- `Exported PPE Device Operations`_


Overview
========

This file describes the Qualcomm PPE (Packet Process Engine) driver.

PPE Architecture
----------------

PPE supports maximum 6 MACs (GMACs or XGMACs) which are connected with 6 PHYs through 3 UNIPHYs,
the 6 PHYs correspond to 6 Panel ethernet ports from port1 to port6. Some IPQ platforms will have
less than 6 MACs and 3 UNIPHYs.

PPE supports to forward the ingress packets to EDMA network driver through intenal cpu port0. It
also supports to forward packets between the PPE ports.

A simplified example view of the PPE interfaces with PHYs and net devices::

   +--------+ +--------+ +--------+ +--------+ +--------+   +--------+    start/stop   +---------+
   | netdev | | netdev | | netdev | | netdev | | netdev |   | netdev | <-------------- |         |
   +--------+ +--------+ +--------+ +--------+ +--------+   +--------+                 |         |
                                |                                                      |         |
                                P0                                                     |         |
                                |                                                      |         |
 +-------------------------------------------------------------------------+           |         |
 |                                                                         |           |         |
 |                        PPE(packet process engine)                       |           |         |
 |                                                                         |           |         |
 |                                                                         |           |         |
 |    +------+ +------+ +------+ +------+    +------+       +------+       |  mac ops  | phylink |
 |    | MAC0 | | MAC1 | | MAC2 | | MAC3 |    | MAC4 |       | MAC5 |       | <-------- |         |
 |    +------+ +------+ +------+ +------+    +------+       +------+       |           |         |
 |       |        |        |        |           |              |           |           |         |
 +-------------------------------------------------------------------------+           |         |
         |        |        |        |           |              |                       |         |
      +-------------------------------+  +-------------+ +-------------+               |         |
      |        [ QSGMII ]             |  | [ USXGMII ] | | [ USXGMII ] |    pcs ops    |         |
      |         UNIPHY0               |  |   UNIPHY1   | |   UNIPHY2   | <------------ |         |
      |                               |  |             | |             |               |         |
      +-------------------------------+  +-------------+ +-------------+               |         |
         |        |        |        |           |             |                        |         |
  +----------------------------------------------------------------------+             |         |
  |   +------+ +------+ +------+ +------+    +------+      +------+      | link change |         |
  |   | PHY0 | | PHY1 | | PHY2 | | PHY3 |    | PHY4 |      | PHY5 |      | ----------> |         |
  |   +------+ +------+ +------+ +------+    +------+      +------+      |             |         |
  |      |        |        |        |           |             | MDIO bus |             |         |
  +----------------------------------------------------------------------+             +---------+
         |         |       |        |           |             |
         P1        P2      P3       P4          P5            P6


PPE Driver Overview
-------------------

PPE driver is the PPE platform driver which executes the initialization of the PPE
hardware including the PPE clock, Queue management, Buffer management, TDM,
scheduler, RSS, MAC and UNIPHY.

PPE driver provides PPE MAC and UNIPHY PCS operations for PHYLINK. It also exports
a set of PPE operations. These operations can be used by network driver to drive
PPE.

PPE driver exports functions to get the PPE device and PPE device operations. Other
network drivers such as PPE EDMA network driver can use the exported functions to
get the PPE device and PPE device operations.

PPE driver provides debugfs file to get the PPE packet counters. These packet counters
include port Rx and Tx counters, CPU code counters and queue counters.


PPE Driver Supported SoCs
=========================

The PPE drivers enable the PPE engine present on the following SoCs:

- IPQ9574
- IPQ5332


Enabling the Driver
===================

The driver is enabled automatically by the network driver which depends on it such as
Qualcomm EDMA driver.

The driver is located in the menu structure at:

  -> Device Drivers
    -> Network device support (NETDEVICES [=y])
      -> Ethernet driver support
        -> Qualcomm devices
          -> Qualcomm Technologies, Inc. PPE Ethernet support

If this driver is built as a module, we can use below commands to install and remove
it:

- insmod qcom-ppe.ko
- rmmod qcom-ppe.ko

Please note that this driver should be installed before the Qualcomm EDMA driver
and removed after the Qualcomm EDMA driver if it is built as a module.


Supported Features
==================

PPE Interface mode - SGMII, 2500BASEX, USXGMII, 10GBASER, QSGMII and QUSGMII.

PPE port link speed from 10Mbps to 10000Mbps.

PPE Packets forwarding and offloading.

PPE RSS.

PPE Buffer Manager.

PPE Queues Manager and Scheduler.


PPE MAC and UNIPHY Interface
============================

PPE MAC and UNIPHY supports various interface mode including SGMII, 2500BASEX, USXGMII,
10GBASER, QSGMII and QUSGMII. A various link including external PHY mode, 2.5G fixed
link mode and 10G SFP mode are also supported.

PPE driver provides two phylink ops functions to PPE EDMA network driver to setup and
destroy phylink for each PPE ports. EDMA neworking driver will setup the phylink when
each net device created and destroy the phylink when each net device removed.

 - .phylink_setup() will lookup the PPE port device tree for PHYLINK-compatible of
   binding (phy-handle) and create and return a PHYLINK instance associated with the
   received net device for each port.

 - .phylink_destroy() will disconnect and destroy the PHYLINK instance for each port.

The PPE phylink MAC ops and UNIPHY PCS ops functions are implemented in the PPE driver
to drive PPE MAC and UNIPHY to interact with PHYLINK framework.


Debugging
=========

PPE packet counters can be checked by debugfs file ``/sys/kernel/debug/ppe/packet_counter``.

PPE MAC statistics can be checked by ``ethtool -S ethX``.

PPE UNIPHY clock and PPE port clock rate can be checked by the clock debugfs file
in ``/sys/kernel/debug/clk/``.

PPE port link state and PHY features can be also checked by ``ethtool ethX``.

The SFP module state can be checked by the debugfs file in ``/sys/kernel/debug/@sfp/state``,
where the ``@sfp`` is the SFP DTS node name.


APIs used by PPE EDMA driver
============================

PPE driver also exports a set of APIs. Whether PPE driver have been probed can be
checked by below API, the PPE EDMA driver uses it to query and ascertain that the
PPE driver is installed and probed before going ahead with its initialization.
::

	bool ppe_is_probed(struct platform_device *pdev);

PPE driver private PPE device structure can be got by below API::

	struct ppe_device *ppe_dev_get(struct platform_device *pdev);

PPE device operations pointer can be got by below API::

	struct ppe_device_ops *ppe_ops_get(struct platform_device *pdev);

Above APIs are exported and also declared in the common ppe header file in
"include/linux/soc/qcom/ppe.h". The network driver which wants to drive PPE
can include this header file and get the PPE device structure by exported
"ppe_dev_get" API, and get the PPE device operations by exported
"ppe_ops_get" API.

PPE queue configuration operations pointer can be got by below API::

	const struct ppe_queue_ops *ppe_queue_config_ops_get(void);

This API is exported and is only used by PPE EDMA driver.


Exported PPE Device Operations
==============================

PPE driver provides a set of PPE device operations to drive the PPE. Higher level
network drivers such as PPE EDMA ethernet driver or a PPE DSA driver can use these
operations to drive the PPE. The definitions of these PPE device operations can be
found in "include/linux/soc/qcom/ppe.h". Below lists these PPE device operations.

phylink_setup::

	.phylink_setup(struct ppe_device *ppe_dev, struct net_device *netdev, int port);

The phylink_setup operation is used by the network driver such as PPE EDMA driver to
create PHYLINK when creating net device. This operation takes net device and the PPE
port id as input parameters, PPE driver will create and return a PHYLINK instance
associated with the net device and port.

phylink_destroy::

	.phylink_destroy(struct ppe_device *ppe_dev, int port);

The phylink_destroy operation is used by the network driver such as PPE EDMA driver
to destroy the PHYLINK.

phylink_mac_config::

	.phylink_mac_config(struct ppe_device *ppe_dev, int port, unsigned int mode,
			const struct phylink_link_state *state);

phylink_mac_link_up::

	.phylink_mac_link_up(struct ppe_device *ppe_dev, int port, struct phy_device *phy,
			unsigned int mode, phy_interface_t interface, int speed, int duplex,
			bool tx_pause, bool rx_pause);

phylink_mac_link_down::

	.phylink_mac_link_down(struct ppe_device *ppe_dev, int port,  unsigned int mode,
			phy_interface_t interface);

phylink_mac_select_pcs::

	.phylink_mac_select_pcs(struct ppe_device *ppe_dev, int port,
			phy_interface_t interface);

Above PHYLINK MAC operations are used by the network driver which creates the PHYLINK and
provides the PHYLINK MAC ops such as PPE DSA driver. In the network driver PHYLINK MAC ops,
it can use above PPE PHYLINK MAC operations to drive the PPE MAC for PHYLINK.

get_stats64::

	.get_stats64(struct ppe_device *ppe_dev, int port, struct rtnl_link_stats64 *s);

This operation is used by PPE network driver to get the PPE MAC statistics, for example
used in network driver net device ops.

get_strings::

	.get_strings(struct ppe_device *ppe_dev, int port, u32 stringset, u8 *data);

get_sset_count::

	.get_sset_count(struct ppe_device *ppe_dev, int port, int sset);

get_ethtool_stats::

	.get_ethtool_stats(struct ppe_device *ppe_dev, int port, u64 *data);

Above operations are used by PPE network driver to show PPE MAC statistics, for example
used in network driver ethtool ops.

set_mac_address::

	.set_mac_address(struct ppe_device *ppe_dev, int port, u8 *macaddr);

This operation is used by PPE network driver to set the PPE MAC address, for example
used in network driver net device ops.

set_mac_eee::

	.set_mac_eee(struct ppe_device *ppe_dev, int port, struct ethtool_eee *eee);

get_mac_eee::

	.get_mac_eee(struct ppe_device *ppe_dev, int port, struct ethtool_eee *eee);

Above operations are used by PPE network driver to set and get the PPE MAC eee, for example
used in network driver ethtool ops.

set_maxframe::

	.set_maxframe(struct ppe_device *ppe_dev, int port, int maxframe_size);

This operation is used by PPE network driver to set the PPE port maximum frame size, for
example used in network net device ops.
