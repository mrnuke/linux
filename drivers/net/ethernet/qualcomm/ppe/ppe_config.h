/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __PPE_CONFIG_H__
#define __PPE_CONFIG_H__

#include <linux/types.h>

/* There are different queue config ranges for the destination port,
 * CPU code and service code.
 */
#define PPE_QUEUE_BASE_DEST_PORT		0
#define PPE_QUEUE_BASE_CPU_CODE			1024
#define PPE_QUEUE_BASE_SERVICE_CODE		2048

#define PPE_RSS_HASH_MODE_IPV4			BIT(0)
#define PPE_RSS_HASH_MODE_IPV6			BIT(1)
#define PPE_RSS_HASH_IP_LENGTH			4
#define PPE_RSS_HASH_TUPLES			5

#define PPE_RING_MAPPED_BP_QUEUE_WORD_COUNT	10

/**
 * struct ppe_qos_scheduler_cfg - PPE QoS scheduler configuration.
 * @flow_id: PPE flow ID.
 * @pri: Scheduler priority.
 * @drr_node_id: Node ID for scheduled traffic.
 * @drr_node_wt: weight for scheduled traffic.
 * @node_unit : Unit for scheduled traffic.
 * @node_frame_mode: Packet mode to be scheduled.
 *
 * PPE QoS feature supports the commit and exceed traffic.
 */
struct ppe_qos_scheduler_cfg {
	int flow_id;
	int pri;
	int drr_node_id;
	int drr_node_wt;
	int node_unit;
	int node_frame_mode;
};

/**
 * struct ppe_queue_ucast_dest - PPE unicast queue destination.
 * @src_profile: Source profile.
 * @service_code_en: Enable service code.
 * @service_code: Service code.
 * @cpu_code_en: Enable CPU code.
 * @cpu_code: CPU code.
 * @dest_port: destination port.
 *
 * PPE egress queue ID is decided by the egress port ID.
 */
struct ppe_queue_ucast_dest {
	int src_profile;
	bool service_code_en;
	int service_code;
	bool cpu_code_en;
	int cpu_code;
	int dest_port;
};

/* Hardware bitmaps for bypassing features of the ingress packet. */
enum ppe_sc_ingress_type {
	PPE_SC_BYPASS_INGRESS_VLAN_TAG_FMT_CHECK = 0,
	PPE_SC_BYPASS_INGRESS_VLAN_MEMBER_CHECK = 1,
	PPE_SC_BYPASS_INGRESS_VLAN_TRANSLATE = 2,
	PPE_SC_BYPASS_INGRESS_MY_MAC_CHECK = 3,
	PPE_SC_BYPASS_INGRESS_DIP_LOOKUP = 4,
	PPE_SC_BYPASS_INGRESS_FLOW_LOOKUP = 5,
	PPE_SC_BYPASS_INGRESS_FLOW_ACTION = 6,
	PPE_SC_BYPASS_INGRESS_ACL = 7,
	PPE_SC_BYPASS_INGRESS_FAKE_MAC_HEADER = 8,
	PPE_SC_BYPASS_INGRESS_SERVICE_CODE = 9,
	PPE_SC_BYPASS_INGRESS_WRONG_PKT_FMT_L2 = 10,
	PPE_SC_BYPASS_INGRESS_WRONG_PKT_FMT_L3_IPV4 = 11,
	PPE_SC_BYPASS_INGRESS_WRONG_PKT_FMT_L3_IPV6 = 12,
	PPE_SC_BYPASS_INGRESS_WRONG_PKT_FMT_L4 = 13,
	PPE_SC_BYPASS_INGRESS_FLOW_SERVICE_CODE = 14,
	PPE_SC_BYPASS_INGRESS_ACL_SERVICE_CODE = 15,
	PPE_SC_BYPASS_INGRESS_FAKE_L2_PROTO = 16,
	PPE_SC_BYPASS_INGRESS_PPPOE_TERMINATION = 17,
	PPE_SC_BYPASS_INGRESS_DEFAULT_VLAN = 18,
	PPE_SC_BYPASS_INGRESS_DEFAULT_PCP = 19,
	PPE_SC_BYPASS_INGRESS_VSI_ASSIGN = 20,
	/* Values 21-23 are not specified by hardware. */
	PPE_SC_BYPASS_INGRESS_VLAN_ASSIGN_FAIL = 24,
	PPE_SC_BYPASS_INGRESS_SOURCE_GUARD = 25,
	PPE_SC_BYPASS_INGRESS_MRU_MTU_CHECK = 26,
	PPE_SC_BYPASS_INGRESS_FLOW_SRC_CHECK = 27,
	PPE_SC_BYPASS_INGRESS_FLOW_QOS = 28,
	/* This must be last as it determines the size of the BITMAP. */
	PPE_SC_BYPASS_INGRESS_SIZE,
};

/* Hardware bitmaps for bypassing features of the egress packet. */
enum ppe_sc_egress_type {
	PPE_SC_BYPASS_EGRESS_VLAN_MEMBER_CHECK = 0,
	PPE_SC_BYPASS_EGRESS_VLAN_TRANSLATE = 1,
	PPE_SC_BYPASS_EGRESS_VLAN_TAG_FMT_CTRL = 2,
	PPE_SC_BYPASS_EGRESS_FDB_LEARN = 3,
	PPE_SC_BYPASS_EGRESS_FDB_REFRESH = 4,
	PPE_SC_BYPASS_EGRESS_L2_SOURCE_SECURITY = 5,
	PPE_SC_BYPASS_EGRESS_MANAGEMENT_FWD = 6,
	PPE_SC_BYPASS_EGRESS_BRIDGING_FWD = 7,
	PPE_SC_BYPASS_EGRESS_IN_STP_FLTR = 8,
	PPE_SC_BYPASS_EGRESS_EG_STP_FLTR = 9,
	PPE_SC_BYPASS_EGRESS_SOURCE_FLTR = 10,
	PPE_SC_BYPASS_EGRESS_POLICER = 11,
	PPE_SC_BYPASS_EGRESS_L2_PKT_EDIT = 12,
	PPE_SC_BYPASS_EGRESS_L3_PKT_EDIT = 13,
	PPE_SC_BYPASS_EGRESS_ACL_POST_ROUTING_CHECK = 14,
	PPE_SC_BYPASS_EGRESS_PORT_ISOLATION = 15,
	PPE_SC_BYPASS_EGRESS_PRE_ACL_QOS = 16,
	PPE_SC_BYPASS_EGRESS_POST_ACL_QOS = 17,
	PPE_SC_BYPASS_EGRESS_DSCP_QOS = 18,
	PPE_SC_BYPASS_EGRESS_PCP_QOS = 19,
	PPE_SC_BYPASS_EGRESS_PREHEADER_QOS = 20,
	PPE_SC_BYPASS_EGRESS_FAKE_MAC_DROP = 21,
	PPE_SC_BYPASS_EGRESS_TUNL_CONTEXT = 22,
	PPE_SC_BYPASS_EGRESS_FLOW_POLICER = 23,
	/* This must be last as it determines the size of the BITMAP. */
	PPE_SC_BYPASS_EGRESS_SIZE,
};

/* Hardware bitmaps for bypassing counter of packet. */
enum ppe_sc_counter_type {
	PPE_SC_BYPASS_COUNTER_RX_VLAN = 0,
	PPE_SC_BYPASS_COUNTER_RX = 1,
	PPE_SC_BYPASS_COUNTER_TX_VLAN = 2,
	PPE_SC_BYPASS_COUNTER_TX = 3,
	/* This must be last as it determines the size of the BITMAP. */
	PPE_SC_BYPASS_COUNTER_SIZE,
};

/* Hardware bitmaps for bypassing features of tunnel packet. */
enum ppe_sc_tunnel_type {
	PPE_SC_BYPASS_TUNNEL_SERVICE_CODE = 0,
	PPE_SC_BYPASS_TUNNEL_TUNNEL_HANDLE = 1,
	PPE_SC_BYPASS_TUNNEL_L3_IF_CHECK = 2,
	PPE_SC_BYPASS_TUNNEL_VLAN_CHECK = 3,
	PPE_SC_BYPASS_TUNNEL_DMAC_CHECK = 4,
	PPE_SC_BYPASS_TUNNEL_UDP_CSUM_0_CHECK = 5,
	PPE_SC_BYPASS_TUNNEL_TBL_DE_ACCE_CHECK = 6,
	PPE_SC_BYPASS_TUNNEL_PPPOE_MC_TERM_CHECK = 7,
	PPE_SC_BYPASS_TUNNEL_TTL_EXCEED_CHECK = 8,
	PPE_SC_BYPASS_TUNNEL_MAP_SRC_CHECK = 9,
	PPE_SC_BYPASS_TUNNEL_MAP_DST_CHECK = 10,
	PPE_SC_BYPASS_TUNNEL_LPM_DST_LOOKUP = 11,
	PPE_SC_BYPASS_TUNNEL_LPM_LOOKUP = 12,
	PPE_SC_BYPASS_TUNNEL_WRONG_PKT_FMT_L2 = 13,
	PPE_SC_BYPASS_TUNNEL_WRONG_PKT_FMT_L3_IPV4 = 14,
	PPE_SC_BYPASS_TUNNEL_WRONG_PKT_FMT_L3_IPV6 = 15,
	PPE_SC_BYPASS_TUNNEL_WRONG_PKT_FMT_L4 = 16,
	PPE_SC_BYPASS_TUNNEL_WRONG_PKT_FMT_TUNNEL = 17,
	/* Values 18-19 are not specified by hardware. */
	PPE_SC_BYPASS_TUNNEL_PRE_IPO = 20,
	/* This must be last as it determines the size of the BITMAP. */
	PPE_SC_BYPASS_TUNNEL_SIZE,
};

/**
 * struct ppe_sc_bypss - PPE service bypass bitmaps
 * @ingress: Bitmap of features that can be bypassed on the ingress packet.
 * @egress: Bitmap of features that can be bypassed on the egress packet.
 * @counter: Bitmap of features that can be bypassed on the counter type.
 * @tunnel: Bitmap of features that can be bypassed on the tunnel packet.
 */
struct ppe_sc_bypass {
	DECLARE_BITMAP(ingress, PPE_SC_BYPASS_INGRESS_SIZE);
	DECLARE_BITMAP(egress, PPE_SC_BYPASS_EGRESS_SIZE);
	DECLARE_BITMAP(counter, PPE_SC_BYPASS_COUNTER_SIZE);
	DECLARE_BITMAP(tunnel, PPE_SC_BYPASS_TUNNEL_SIZE);
};

/**
 * struct ppe_servcode_cfg - PPE service code configuration.
 * @dest_port_valid: Generate destination port or not.
 * @dest_port: Destination port ID.
 * @bitmaps: Bitmap of bypass features.
 * @is_src: Destination port acts as source port, packet sent to CPU.
 * @field_update_bitmap: Fields updated to the EDMA preheader.
 * @next_service_code: New service code.
 * @hw_service: Hardware functions selected.
 * @offset_sel: Packet offset selection.
 *
 * Service code is generated during the packet passing through PPE.
 */
struct ppe_servcode_cfg {
	bool dest_port_valid;
	int dest_port;
	struct ppe_sc_bypass bitmaps;
	bool is_src;
	int field_update_bitmap;
	int next_service_code;
	int hw_service;
	int offset_sel;
};

/* The action of packet received by PPE can be forwarded, dropped, copied
 * to CPU (enter multicast queue), redirected to CPU (enter unicast queue).
 */
enum ppe_action_type {
	PPE_ACTION_FORWARD = 0,
	PPE_ACTION_DROP = 1,
	PPE_ACTION_COPY_TO_CPU = 2,
	PPE_ACTION_REDIRECT_TO_CPU = 3,
};

/**
 * struct ppe_rss_hash_cfg - PPE RSS hash configuration.
 * @hash_mask: Mask of the generated hash value.
 * @hash_fragment_mode: Mode of the fragment packet for 3 tuples.
 * @hash_seed: Seed to generate RSS hash.
 * @hash_sip_mix: Source IP selection.
 * @hash_dip_mix: Destination IP selection.
 * @hash_protocol_mix: Protocol selection.
 * @hash_sport_mix: Source L4 port selection.
 * @hash_sport_mix: Destination L4 port selection.
 * @hash_fin_inner: RSS hash value first selection.
 * @hash_fin_outer: RSS hash value second selection.
 *
 * PPE RSS hash value is generated based on the RSS hash configuration
 * with the received packet.
 */
struct ppe_rss_hash_cfg {
	u32 hash_mask;
	bool hash_fragment_mode;
	u32 hash_seed;
	u8 hash_sip_mix[PPE_RSS_HASH_IP_LENGTH];
	u8 hash_dip_mix[PPE_RSS_HASH_IP_LENGTH];
	u8 hash_protocol_mix;
	u8 hash_sport_mix;
	u8 hash_dport_mix;
	u8 hash_fin_inner[PPE_RSS_HASH_TUPLES];
	u8 hash_fin_outer[PPE_RSS_HASH_TUPLES];
};

int ppe_hw_config(struct ppe_device *ppe_dev);
int ppe_queue_scheduler_set(struct ppe_device *ppe_dev,
			    int node_id, bool flow_level, int port,
			    struct ppe_qos_scheduler_cfg scheduler_cfg);
int ppe_queue_scheduler_get(struct ppe_device *ppe_dev,
			    int node_id, bool flow_level, int *port,
			    struct ppe_qos_scheduler_cfg *scheduler_cfg);
int ppe_queue_ucast_base_set(struct ppe_device *ppe_dev,
			     struct ppe_queue_ucast_dest queue_dst,
			     int queue_base,
			     int profile_id);
int ppe_queue_ucast_pri_class_set(struct ppe_device *ppe_dev,
				  int profile_id,
				  int priority,
				  int class_offset);
int ppe_queue_ucast_hash_class_set(struct ppe_device *ppe_dev,
				   int profile_id,
				   int rss_hash,
				   int class_offset);
int ppe_port_resource_get(struct ppe_device *ppe_dev, int port, int type,
			  int *res_start, int *res_end);
int ppe_servcode_config_set(struct ppe_device *ppe_dev,
			    int servcode,
			    struct ppe_servcode_cfg cfg);
int ppe_counter_set(struct ppe_device *ppe_dev, int port, bool enable);
int ppe_rss_hash_config_set(struct ppe_device *ppe_dev, int mode,
			    struct ppe_rss_hash_cfg hash_cfg);
int ppe_ring_queue_map_set(struct ppe_device *ppe_dev,
			   int ring_id,
			   u32 *queue_map);
#endif
