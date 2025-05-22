.. SPDX-License-Identifier: GPL-2.0

===============================================
PPE Ethernet Driver for Qualcomm IPQ SoC Family
===============================================

Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.

Author: Lei Wei <quic_leiwei@quicinc.com>


Contents
========

- `PPE Overview`_
- `PPE Driver Overview`_
- `PPE Port MAC Interface`_
- `PPE Driver Supported SoCs`_
- `Enabling the Driver`_
- `Command Line Parameters`_
- `Ethernet DMA (EDMA)`_
- `Debugging`_


PPE Overview
============

IPQ (Qualcomm Internet Processor) SoC (System-on-Chip) series is Qualcomm's series of
networking SoC for Wi-Fi access points. The PPE (Packet Process Engine) is the Ethernet
packet process engine in the IPQ SoC.

Below is a simplified hardware diagram of IPQ9574 SoC which includes the PPE engine and
other blocks which are in the SoC but outside the PPE engine. These blocks work together
to enable the Ethernet for the IPQ SoC::

             +------+ +------+ +------+ +------+ +------+  +------+ start +-------+
             |netdev| |netdev| |netdev| |netdev| |netdev|  |netdev|<------|PHYLINK|
             +------+ +------+ +------+ +------+ +------+  +------+ stop  +-+-+-+-+
                                           |                                | | ^
 +-------+   +-------------------------+--------+----------------------+    | | |
 | GCC   |   |                         |  EDMA  |                      |    | | |
 +---+---+   |  PPE                    +---+----+                      |    | | |
     | clk   |                             |                           |    | | |
     +------>| +-----------------------+------+-----+---------------+  |    | | |
             | |   Switch Core         |Port0 |     |Port7(EIP FIFO)|  |    | | |
             | |                       +---+--+     +------+--------+  |    | | |
             | |                           |               |        |  |    | | |
 +-------+   | |                    +------+---------------+----+   |  |    | | |
 |CMN PLL|   | | +---+ +---+ +----+ | +--------+                |   |  |    | | |
 +---+---+   | | |BM | |QM | |SCH | | | L2/L3  |  .......       |   |  |    | | |
 |   |       | | +---+ +---+ +----+ | +--------+                |   |  |    | | |
 |   |       | |                    +------+--------------------+   |  |    | | |
 |   |       | |                           |                        |  |    | | |
 |   v       | | +-----+-+-----+-+-----+-+-+---+--+-----+-+-----+   |  |    | | |
 | +------+  | | |Port1| |Port2| |Port3| |Port4|  |Port5| |Port6|   |  |    | | |
 | |NSSCC |  | | +-----+ +-----+ +-----+ +-----+  +-----+ +-----+   |  | mac| | |
 | +-+-+--+  | | |MAC0 | |MAC1 | |MAC2 | |MAC3 |  |MAC4 | |MAC5 |   |  |<---+ | |
 | ^ | |clk  | | +-----+-+-----+-+-----+-+-----+--+-----+-+-----+   |  | ops  | |
 | | | +---->| +----|------|-------|-------|---------|--------|-----+  |      | |
 | | |       +---------------------------------------------------------+      | |
 | | |              |      |       |       |         |        |               | |
 | | |   MII clk    |      QSGMII               USXGMII   USXGMII             | |
 | | +------------->|      |       |       |         |        |               | |
 | |              +-------------------------+ +---------+ +---------+         | |
 | |125/312.5M clk|       (PCS0)            | | (PCS1)  | | (PCS2)  | pcs ops | |
 | +--------------+       UNIPHY0           | | UNIPHY1 | | UNIPHY2 |<--------+ |
 +--------------->|                         | |         | |         |           |
 | 31.25M ref clk +-------------------------+ +---------+ +---------+           |
 |                   |     |      |      |          |          |                |
 |              +-----------------------------------------------------+         |
 |25/50M ref clk| +-------------------------+    +------+   +------+  | link    |
 +------------->| |      QUAD PHY           |    | PHY4 |   | PHY5 |  |---------+
                | +-------------------------+    +------+   +------+  | change
                |                                                     |
                |                       MDIO bus                      |
                +-----------------------------------------------------+

The CMN (Common) PLL, NSSCC (Networking Sub System Clock Controller) and GCC (Global
Clock Controller) blocks are in the SoC and act as clock providers.

The UNIPHY block is in the SoC and provides the PCS (Physical Coding Sublayer) and
XPCS (10-Gigabit Physical Coding Sublayer) functions to support different interface
modes between the PPE MAC and the external PHY.

This documentation focuses on the descriptions of PPE engine and the PPE driver.

The Ethernet functionality in the PPE (Packet Process Engine) is comprised of three
components: the switch core, port wrapper and Ethernet DMA.

The Switch core in the IPQ9574 PPE has maximum of 6 front panel ports and two FIFO
interfaces. One of the two FIFO interfaces is used for Ethernet port to host CPU
communication using Ethernet DMA. The other one is used communicating to the EIP
engine which is used for IPsec offload. On the IPQ9574, the PPE includes 6 GMAC/XGMACs
that can be connected with external Ethernet PHY. Switch core also includes BM (Buffer
Management), QM (Queue Management) and SCH (Scheduler) modules for supporting the
packet processing.

The port wrapper provides connections from the 6 GMAC/XGMACS to UNIPHY (PCS) supporting
various modes such as SGMII/QSGMII/PSGMII/USXGMII/10G-BASER. There are 3 UNIPHY (PCS)
instances supported on the IPQ9574.

Ethernet DMA is used to transmit and receive packets between the Ethernet subsystem
and ARM host CPU.

The following lists the main blocks in the PPE engine which will be driven by this
PPE driver:

- BM
    BM is the hardware buffer manager for the PPE switch ports.
- QM
    Queue Manager for managing the egress hardware queues of the PPE switch ports.
- SCH
    The scheduler which manages the hardware traffic scheduling for the PPE switch ports.
- L2
    The L2 block performs the packet bridging in the switch core. The bridge domain is
    represented by the VSI (Virtual Switch Instance) domain in PPE. FDB learning can be
    enabled based on the VSI domain and bridge forwarding occurs within the VSI domain.
- MAC
    The PPE in the IPQ9574 supports up to six MACs (MAC0 to MAC5) which are corresponding
    to six switch ports (port1 to port6). The MAC block is connected with external PHY
    through the UNIPHY PCS block. Each MAC block includes the GMAC and XGMAC blocks and
    the switch port can select to use GMAC or XMAC through a MUX selection according to
    the external PHY's capability.
- EDMA (Ethernet DMA)
    The Ethernet DMA is used to transmit and receive Ethernet packets between the PPE
    ports and the ARM cores.

The received packet on a PPE MAC port can be forwarded to another PPE MAC port. It can
be also forwarded to internal switch port0 so that the packet can be delivered to the
ARM cores using the Ethernet DMA (EDMA) engine. The Ethernet DMA driver will deliver the
packet to the corresponding 'netdevice' interface.

The software instantiations of the PPE MAC (netdevice), PCS and external PHYs interact
with the Linux PHYLINK framework to manage the connectivity between the PPE ports and
the connected PHYs, and the port link states. This is also illustrated in above diagram.


PPE Driver Overview
===================
PPE driver is Ethernet driver for the Qualcomm IPQ SoC. It is a single platform driver
which includes the PPE part and Ethernet DMA part. The PPE part initializes and drives the
various blocks in PPE switch core such as BM/QM/L2 blocks and the PPE MACs. The EDMA part
drives the Ethernet DMA for packet transfer between PPE ports and ARM cores, and enables
the netdevice driver for the PPE ports.

The PPE driver files in drivers/net/ethernet/qualcomm/ppe/ are listed as below:

- Makefile
- ppe.c
- ppe.h
- ppe_config.c
- ppe_config.h
- ppe_debugfs.c
- ppe_debugfs.h
- ppe_port.c
- ppe_port.h
- ppe_regs.h

The ppe.c file contains the main PPE platform driver and undertakes the initialization of
PPE switch core blocks such as QM, BM and L2. The configuration APIs for these hardware
blocks are provided in the ppe_config.c file.

The ppe.h defines the PPE device data structure which will be used by PPE driver functions.

The ppe_debugfs.c enables the PPE statistics counters such as PPE port Rx and Tx counters,
CPU code counters and queue counters.

The ppe_port.c initializes PPE MAC ports and also provides a set of port phylink and MAC
functions.

Command Line Parameters
=======================

If the driver is built as a module, module parameters can be used by providing
them in the command line as mentioned below:

    insmod qcom-ppe.ko [<param1>=<VAL1> <param2>=<VAL2>]

Default values for these parameters are set in the driver itself.
Some of the parameters that can be changed by user are described below:

page_mode
---------
:param: page_mode
:Valid Range: 0-1 (0=off, 1=on)
:Default Value: 0

This parameter enables page mode.

rx_buff_size
------------
:param: rx_buff_size
:Valid Range: 0-9000
:Default Value: 0

This parameter sets the Rx buffer size for Jumbo MRU.

edma_rx_napi_budget
-------------------
:Valid Range: 16-512
:Default Value: 128

This parameter sets the Rx NAPI budget.

edma_tx_napi_budget
-------------------
:Valid Range: 16-512
:Default Value: 512

This parameter sets the Tx NAPI budget.

edma_rx_mitigation_pkt_cnt
--------------------------
:Valid Range: 0-256
:Default Value: 16

This parameter is Rx mitigation packet count value.

edma_rx_mitigation_timer
------------------------
:Valid Range: 0-1000
:Default Value: 25

This parameter is Rx mitigation timer value in microseconds.

edma_tx_mitigation_timer
------------------------
:Valid Range: 0-1000
:Default Value: 25

This parameter is Tx mitigation timer value in microseconds.

edma_tx_mitigation_pkt_cnt
--------------------------
:Valid Range: 0-256
:Default Value: 16

This parameter is Tx mitigation packet count value.

tx_requeue_stop
---------------
:Valid Range: 0-1
:Default Value: 1

This parameter stops requeueing of Tx packets.
It is recommended to disable this if a Qdisc is enabled on the ethernet netdevice.

Procfs Support
=======================

EDMA driver leverages the existing Linux kernel procfs file system support
to configure some of the features. The procfs interface creates the
/proc/sys/net/edma directory.

RPS (Receive Packet Steering): configure bitmap of cores for RPS.
The network packets can be distributed among multiple CPUs by setting
respective bitmap core value in this procfs entry
``/proc/sys/net/edma/rps_bitmap_cores``

PPE Port MAC Interface
======================

PPE driver sets up the PPE port instance for each PPE MAC port by parsing the "ethernet-ports"
DTS node. The port MAC interface provides PHYLINK functions which interact with Linux PHYLINK
framework and port MAC functions which are used by ethtool and netdev ops.

PHYLINK support
---------------
PPE Port PHYLINK supports various interface mode including QUSGMII, USXGMII, 2500BASEX,
10GBASER and so on. Various link mode including external PHY mode, fixed link mode and SFP
mode are also supported.

PPE driver uses below two functions to setup and destroy PHYLINK for the dedicated PPE port::

	- int ppe_port_phylink_setup(struct ppe_port *ppe_port, struct net_device *netdev);
	- void ppe_port_phylink_destroy(struct ppe_port *ppe_port)

PPE driver will create PHYLINK instance and PCS instance for the dedicated PPE MAC port
in the phylink setup function. The PPE PHYLINK MAC operations are provided when creating
the PHYLINK instance.

PPE driver will destroy PHYLINK and PCS instance for the dedicated PPE MAC port in the
phylink destroy function.

PPE port DTS node reference the PCS node and use the "ipq_pcs_get()" and "ipq_pcs_put()"
APIs exported by Qualcomm IPQ9574 PCS driver to get and put the PHYLINK PCS instance.

The PHYLINK PCS operations are provided by the IPQ9574 PCS driver while the PHYLINK MAC
operations are provided by PPE driver. Below is the PHYLINK MAC operations in PPE driver
which are used by PHYLINK.

ppe_port_mac_config::

	ppe_port_mac_config(struct phylink_config *config, unsigned int mode,
		const struct phylink_link_state *state);

This function is to perform PPE port MAC configurations.

ppe_port_mac_link_up::

	ppe_port_mac_link_up(struct phylink_config *config, struct phy_device *phy,
		unsigned int mode, phy_interface_t interface, int speed, int duplex,
		bool tx_pause, bool rx_pause);

ppe_port_mac_link_down::

	ppe_port_mac_link_down(struct phylink_config *config, unsigned int mode,
		phy_interface_t interface);

Above two functions are to perform PPE port MAC link up and link down configurations.

ppe_port_mac_select_pcs::

	ppe_port_mac_select_pcs(struct phylink_config *config, phy_interface_t interface);

This function is to select the corresponding PCS instance for the PPE port PHYLINK
instance.

PPE port MAC functions
----------------------

Below are the PPE port MAC functions which are used by ethtool and netdev ops.

get_stats64::

	void ppe_port_get_stats64(struct ppe_port *ppe_port, struct rtnl_link_stats64 *s);

get_sset_count::

	int ppe_port_get_sset_count(struct ppe_port *ppe_port, int sset);

get_strings::

	void ppe_port_get_strings(struct ppe_port *ppe_port, u32 stringset, u8 *data);


get_ethtool_stats::

	void ppe_port_get_ethtool_stats(struct ppe_port *ppe_port, u64 *data);

Above functions are used to get the MAC MIB statistics for the dedicated PPE port.

set_mac_address::

	int ppe_port_set_mac_address(struct ppe_port *ppe_port, u8 *macaddr);

This function is used to set the MAC address for the dedicated PPE port.

set_maxframe::

	int ppe_port_set_maxframe(struct ppe_port *ppe_port, int maxframe_size);

This function is used to set the maximum frame size for the dedicated PPE port.

PPE Driver Supported SoCs
=========================

The PPE driver supports the following IPQ SoC:

- IPQ9574


Enabling the Driver
===================

The driver is located in the menu structure at::

  -> Device Drivers
    -> Network device support (NETDEVICES [=y])
      -> Ethernet driver support
        -> Qualcomm devices
          -> Qualcomm Technologies, Inc. PPE Ethernet support

The PPE driver functionally depends on the CMN PLL and NSSCC clock controller drivers.
It also depends on the Qualcomm IPQ9574 PCS driver which enables PHYLINK PCS instance
for the PPE MAC port. Please make sure the dependent modules are installed before
installing the PPE driver module.

Ethernet DMA (EDMA)
===================

This is a common ethernet DMA block inside PPE, which is used to transmit and
receive packets between Ethernet ports in the PPE switch, and the ARM cores.

Transmit Process
----------------

Tx netdevice transmit ops is invoked when the kernel needs to transmit a packet;
it sets the descriptors in the ring and informs the DMA engine that there is a
packet ready to be transmitted.

By default, the driver sets the ``NETIF_F_SG`` bit in the features field of
the ``net_device`` structure, enabling the scatter-gather feature.

Receive Process
---------------

When One or more packets are received by by NIC interface then an RX ring
interrupt is generated to process the packet in host. EDMA driver will handle
this interrupt and schedule a NAPI to process the RX descriptors.

NAPI poll function then reaps the RX descriptors and then constructs SKB
which is then given to Linux networking stack for further processing.

Interrupt Mitigation
--------------------

EDMA HW supports interrupt mitigation configuration which dictates the packet
count or speed at which interrupt will be generated.

Jumbo and Segmentation Offloading
---------------------------------

Jumbo frames are supported. The netdevice feature flags support GSO.

TSO Support
-----------

TSO (TCP Segmentation Offload) feature is supported. HW does not support TSO
for packets with more than or equal to 32 segments. HW hangs up if it sees
more than 32 segments. So the driver implements a HW WAR to fallback to SW
GSO path in case it sees more than 32 TSO segments.

Debugging
=========

The PPE hardware counters are available in the debugfs and can be checked by the command
``cat /sys/kernel/debug/ppe/packet_counters``.

The PPE port MAC statistics can be checked by ethtool command ``ethtool -S ethX``.

The PPE port link state and PHY features can be checked by ethtool command ``ethtool ethX``.

PPE EDMA supported features can be checked by ``ethtool -k ethX``

The SFP module state can be checked by ``cat /sys/kernel/debug/@sfp/state``, where the
``@sfp`` is the SFP DTS node name.

PPE EDMA Tx ring, Rx ring and miscellaneous statistics can be checked by
``cat /sys/kernel/debug/edma-dp/stats/tx_ring_stats`` or
``cat /sys/kernel/debug/edma-dp/stats/rx_ring_stats`` or
``cat /sys/kernel/debug/edma-dp/stats/misc_stats``

Developer can also enable the CONFIG_DYNAMIC_DEBUG to get further debug
information.
