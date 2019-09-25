/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _NET_WLAN_WLAN_EXT_REQ_H_
#define _NET_WLAN_WLAN_EXT_REQ_H_

#include <stdint.h>
#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Wlan extended command definition
 */
typedef enum wlan_ext_cmd {
    WLAN_EXT_CMD_SET_PM_DTIM = 0,
    WLAN_EXT_CMD_GET_PM_DTIM,
    WLAN_EXT_CMD_SET_PS_CFG,
    WLAN_EXT_CMD_SET_AMPDU_TXNUM,
    WLAN_EXT_CMD_SET_TX_RETRY_CNT,
    WLAN_EXT_CMD_SET_PM_TX_NULL_PERIOD,
    WLAN_EXT_CMD_SET_BCN_WIN_US,
    WLAN_EXT_CMD_GET_BCN_STATUS,
    WLAN_EXT_CMD_SET_PHY_PARAM,
    WLAN_EXT_CMD_SET_SCAN_PARAM,
    WLAN_EXT_CMD_SET_LISTEN_INTERVAL,
    WLAN_EXT_CMD_SET_AUTO_SCAN,
    WLAN_EXT_CMD_SET_P2P_SVR,
    WLAN_EXT_CMD_SET_P2P_WKP_CFG,
    WLAN_EXT_CMD_SET_P2P_KPALIVE_CFG,
    WLAN_EXT_CMD_SET_P2P_HOST_SLEEP,
    WLAN_EXT_CMD_SET_BCN_WIN_CFG,
    WLAN_EXT_CMD_SET_FORCE_B_RATE,
    WLAN_EXT_CMD_SET_RCV_SPECIAL_FRM,
    WLAN_EXT_CMD_SET_SEND_RAW_FRM_CFG,
    WLAN_EXT_CMD_SET_SNIFF_SYNC_CFG,
    WLAN_EXT_CMD_SET_RCV_FRM_FILTER_CFG,
    WLAN_EXT_CMD_SET_POWER_LEVEL_TAB,
    WLAN_EXT_CMD_GET_POWER_LEVEL_TAB,
    WLAN_EXT_CMD_SET_SWITCH_CHN_CFG,
    WLAN_EXT_CMD_GET_CURRENT_CHN,
    WLAN_EXT_CMD_SET_SNIFF_KP_ACTIVE,
    WLAN_EXT_CMD_SET_FRM_FILTER,
    WLAN_EXT_CMD_SET_TEMP_FRM,
    WLAN_EXT_CMD_SET_UPDATE_TEMP_IE,
    WLAN_EXT_CMD_SET_SYNC_FRM_SEND,

    WLAN_EXT_CMD_GET_TX_RATE = 50,
    WLAN_EXT_CMD_GET_SIGNAL,
    WLAN_EXT_CMD_SET_MIXRATE,

    WLAN_EXT_CMD_SET_MBUF_LIMIT,
    WLAN_EXT_CMD_SET_AMPDU_REORDER_AGE,
    WLAN_EXT_CMD_SET_SCAN_FREQ,
    WLAN_EXT_CMD_SET_RX_STACK_SIZE,
} wlan_ext_cmd_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_PS_CFG
 */
typedef struct wlan_ext_ps_cfg {
	uint8_t ps_mode;
	uint8_t ps_idle_period;
	uint8_t ps_change_period;
} wlan_ext_ps_cfg_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_GET_PM_DTIM
 */
typedef struct wlan_ext_pm_dtim {
	uint8_t pm_join_dtim_period;
	uint8_t pm_dtim_period_extend;
} wlan_ext_pm_dtim_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_GET_BCN_STATUS
 */
typedef struct wlan_ext_bcn_status {
	uint32_t bcn_duration;
	int32_t  bcn_delay_max;
	int32_t  bcn_delay_sum;
	uint16_t bcn_delay_cnt[8];
	uint16_t bcn_rx_cnt;
	uint16_t bcn_miss_cnt;
} wlan_ext_bcn_status_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_GET_SIGNAL
 */
typedef struct wlan_ext_signal {
	int8_t rssi;	/* unit is 0.5db */
	int8_t noise;	/* unit is dbm */
} wlan_ext_signal_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_MBUF_LIMIT
 */
typedef struct wlan_ext_mbuf_limit {
	uint32_t tx;
	uint32_t rx;
	uint32_t txrx;
} wlan_ext_mbuf_limit_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_PHY_PARAM
 */
typedef struct wlan_ext_phy_param {
	uint8_t check_period;       /* stun checking period (in ms)
	                               MUST be divisible by 200 (required by fw) */
	uint8_t cca_threshold;      /* cca stun threshold */
	uint8_t ofdm_threshold;     /* ofdm stun threshold */
	uint8_t ofdm_rate_idx;      /* h/w rate index for transmitting null frame */
	uint8_t ofdm_retry_cnt;     /* retry count for transmitting null frame */
	uint8_t ofdm_max_interval;  /* max interval (in seconds) to applying ofdm
	                               workaround, 0 for no workaround */
} wlan_ext_phy_param_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SCAN_PARAM
 */
typedef struct wlan_ext_scan_param {
	uint8_t  num_probes;  /* number of probe requests (per SSID) sent to
	                         one channel, default to 2 */
	uint8_t  probe_delay; /* delay time (in us) before sending a probe request,
	                         default to 100 us */
	uint16_t min_dwell;   /* min channel dwell time (in ms), default to 15 ms */
	uint16_t max_dwell;   /* max channel dwell time (in ms), default to 35 ms */
} wlan_ext_scan_param_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SCAN_FREQ
 */
typedef struct wlan_ext_scan_freq {
	uint16_t freq_num;
	int32_t  *freq_list;
} wlan_ext_scan_freq_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_AUTO_SCAN
 */
typedef struct wlan_ext_auto_scan_param {
	uint8_t  auto_scan_enable;  /* enable auto scan, default disable(0) */
 	uint32_t auto_scan_interval; /* auto scan interval(in second), defualt 0s */
} wlan_ext_auto_scan_param_t;

#define IPC_P2P_KPALIVE_PAYLOAD_LEN_MAX 128
#define IPC_P2P_WUP_PAYLOAD_LEN_MAX     128
#define IPC_P2P_SERVER_MAX 3
/**
 * @brief Parameter for WLAN_EXT_CMD_SET_P2P_SVR
 */
typedef struct wlan_ext_p2p_svr
{
    uint16_t  Enable;
    uint16_t  IPIdInit;
    uint32_t  TcpSeqInit;
    uint32_t  TcpAckInit;
    uint8_t   DstIPv4Addr[4];
    uint16_t  SrcPort;
    uint16_t  DstPort;
    uint8_t   DstMacAddr[6];
    uint16_t  TcpOrUdp;//0:tcp  1:udp
} wlan_ext_p2p_svr_t;

typedef struct wlan_ext_p2p_svr_set
{
    uint8_t   EncHdrSize;
    uint8_t   EncTailSize;
    uint16_t  reserved1;
    uint8_t   SrcIPv4Addr[4];
    wlan_ext_p2p_svr_t  P2PServerCfgs[IPC_P2P_SERVER_MAX];
} wlan_ext_p2p_svr_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_P2P_WKP_CFG
 */
typedef struct wlan_ext_p2p_wkp_param_cfg
{
    uint16_t  Enable;
    uint16_t  PayloadLen;
    uint8_t   Payload[IPC_P2P_WUP_PAYLOAD_LEN_MAX];
} wlan_ext_p2p_wkp_param_cfg_t;

typedef struct wlan_ext_p2p_wkp_param_set
{
    wlan_ext_p2p_wkp_param_cfg_t P2PWkpParamCfgs[IPC_P2P_SERVER_MAX];
} wlan_ext_p2p_wkp_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_P2P_KPALIVE_CFG
 */
typedef struct wlan_ext_p2p_kpalive_param_cfg
{
    uint16_t  Enable;
    uint16_t  PayloadLen;
    uint8_t   Payload[IPC_P2P_KPALIVE_PAYLOAD_LEN_MAX];
} wlan_ext_p2p_kpalive_param_cfg_t;

typedef struct wlan_ext_p2p_kpalive_param_set
{
    uint8_t   KeepAlivePeriod_s;	  //unit:Second
    uint8_t   TxTimeOut_s;			  //unit:Second  Keep alive packet tx timeout value
    uint8_t   TxRetryLimit; 		  //keep alive packet tx retry limit
    uint8_t   reserved1;
    wlan_ext_p2p_kpalive_param_cfg_t P2PKeepAliveParamCfgs[IPC_P2P_SERVER_MAX];
} wlan_ext_p2p_kpalive_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_RCV_FRM_FILTER_CFG
 */
#define RCV_FRM_FILTER_FRAME_TYPE        (1 << 0)
#define RCV_FRM_FILTER_MAC_ADDRESS       (1 << 1)
#define RCV_FRM_FILTER_PAYLOAD           (1 << 2)
#define RCV_FRM_FILTER_IE                (1 << 3)

#define RCV_FRM_FILTER_MAC_ADDR_A1       (1 << 0)
#define RCV_FRM_FILTER_MAC_ADDR_A2       (1 << 1)
#define RCV_FRM_FILTER_MAC_ADDR_A3       (1 << 2)

//FrameType define
#define FILTER_D11_MGMT_TYPE           0
#define FILTER_D11_SUB_MGMT_ASRQ       (1 << (0x0 + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_ASRSP      (1 << (0x1 + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_RSRQ       (1 << (0x2 + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_RSRSP      (1 << (0x3 + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_PBRQ       (1 << (0x4 + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_PBRSP      (1 << (0x5 + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_BCN        (1 << (0x8 + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_ATIM       (1 << (0x9 + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_DAS        (1 << (0xa + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_AUTH       (1 << (0xb + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_DAUTH      (1 << (0xc + FILTER_D11_MGMT_TYPE))
#define FILTER_D11_SUB_MGMT_ACTION     (1 << (0xd + FILTER_D11_MGMT_TYPE))

#define FILTER_D11_DATA_TYPE           16
#define FILTER_D11_SUB_DATA            (1 << (0x0 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_DATA_CFACK      (1 << (0x1 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_DATA_CFPOLL     (1 << (0x2 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_DATA_CFAKPL     (1 << (0x3 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_DNUL            (1 << (0x4 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_DNUL_CFACK      (1 << (0x5 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_DNUL_CFPOLL     (1 << (0x6 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_DNUL_CFAKPL     (1 << (0x7 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_QDATA           (1 << (0x8 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_QDATA_CFACK     (1 << (0x9 + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_QDATA_CFPOLL    (1 << (0xa + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_QDATA_CFAKPL    (1 << (0xb + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_QDNUL           (1 << (0xc + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_QDNUL_CFACK     (1 << (0xd + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_QDNUL_CFPOLL    (1 << (0xe + FILTER_D11_DATA_TYPE))
#define FILTER_D11_SUB_QDNUL_CFAKPL    (1 << (0xf + FILTER_D11_DATA_TYPE))

#define RCV_FRM_FILTER_PAYLOAD_LEN_MAX     256
#define RCV_FRM_FILTER_IE_LEN_MAX          256
typedef struct rcv_frm_filter
{
    uint16_t  FilterEnable;
    uint16_t  AndOperationMask;
    uint16_t  OrOperationMask;
	uint16_t  Reserved;
    uint32_t  FrameType;
    uint8_t   MacAddrMask;
	uint8_t   Reserved1;
    uint8_t   MacAddrA1[6];
    uint8_t   MacAddrA2[6];
    uint8_t   MacAddrA3[6];
    union{
        struct {
		    uint16_t  PayloadOffset;
		    uint16_t  PayloadLength;
		    uint8_t   Payload[RCV_FRM_FILTER_PAYLOAD_LEN_MAX];
        } __packed PayloadCfg;
        struct {
		    uint8_t  ElementId;
		    uint8_t  Length;
			uint16_t Reserved;
		    uint8_t  OUI[RCV_FRM_FILTER_IE_LEN_MAX];
        } __packed IeCfg;
	} __packed;
} rcv_frm_filter_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_FRM_FILTER
 */
#define FRAME_FILTER_ENABLE           (1 << 0)
typedef struct wlan_ext_frm_filter_set
{
    uint32_t  Filter1Cfg;
    uint32_t  Filter2Cfg;
	rcv_frm_filter_t  Filter1;
	rcv_frm_filter_t  Filter2;
} wlan_ext_frm_filter_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_RCV_SPECIAL_FRM
 */
#define SEND_DUPLICATE_FRAME_TO_HOST_ENABLE    (1 << 0)
#define RECV_UNICAST_FRAME_ENABLE              (1 << 1)
#define RECV_BROADCAST_FRAME_ENABLE            (1 << 2)

typedef struct wlan_ext_rcv_spec_frm_param_set
{
    uint32_t  Enable;//0 or 1
    uint32_t  u32RecvSpecFrameCfg;
} wlan_ext_rcv_spec_frm_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SEND_RAW_FRM_CFG
 */
#define SEND_RAW_FRAME_ALLOCATE_SEQNUM        (1 << 0)    //wifi allocate tx frame sequence number, not from Host
#define SEND_RAW_FRAME_NO_ACK                 (1 << 1)    //wifi will not wait for ack after send raw frame if bit is be set
typedef struct wlan_ext_send_raw_frm_param_set
{
	uint8_t     Enable;
	uint8_t     Reserved;
	uint16_t    u16SendRawFrameCfg;//reserved for now
} wlan_ext_send_raw_frm_param_set_t;

//Band
#define SEND_RAW_FRAME_BAND_2G4   0
#define SEND_RAW_FRAME_BAND_5G    1

//Flag
#define SEND_RAW_FRAME_FLAG_SHORT_PREAMBLE   (1 << 2)//no use now
#define SEND_RAW_FRAME_USE_MAC_ADDR_1        (1 << 4)//no use now

#define SEND_RAW_FRAME_MAX_SWITCH_CHANNEL_TIME   5000
typedef struct wlan_ext_switch_channel_param_set
{
    uint8_t     Enable;
    uint8_t     Band;
    uint16_t    Flag;//reserved now
    uint32_t    ChannelNum;
    uint32_t    SwitchChannelTime;
} wlan_ext_switch_channel_param_set_t;

typedef struct wlan_ext_get_cur_channel
{
    uint16_t    Channel;
    uint16_t    Reserved;
}wlan_ext_get_cur_channel_t;


/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SNIFF_SYNC_CFG
 */
//u32SniffSyncCfg
#define SNIFF_SYNC_AT_HOST    (0x1)
#define SNIFF_SYNC_AT_WIFI    (0x0)
#define SNIFF_SYNC_METHOD     			(1 << 0)
#define SNIFF_SYNC_DUPLICATE_TO_HOST    (1 << 1)
#define SNIFF_SYNC_DISABLE_TIMER        (1 << 2)
#define SNIFF_SYNC_UPLOAD_DATA_FRM      (1 << 3)
#define SNIFF_SYNC_UNICAST_FRM          (1 << 4)
#define SNIFF_SYNC_MULTICAST_FRM        (1 << 5)

//SYNC_AT_HOST:Flag
#define SNIFF_SYNC_AT_HOST_FRAME_SEND_TO_HOST    (1 << 0)    //send frame to host if enable

//SYNC_AT_WIFI:Flag
#define SNIFF_SYNC_AT_WIFI_FRAME_SEND_TO_HOST         (1 << 0)
#define SNIFF_SYNC_AT_WIFI_SEND_HOST_LOST_SYNC        (1 << 1)
#define SNIFF_SYNC_AT_WIFI_ADAPTIVE_EXPANSION_ENABLE  (1 << 2)
#define SNIFF_SYNC_AT_WIFI_SEND_SYNC_INDC_TO_HOST     (1 << 3)
#define SNIFF_SYNC_AT_WIFI_UPLOAD_WHEN_PL_MISMATCH    (1 << 4)
#define SNIFF_SYNC_AT_WIFI_SYNC_USE_FILTER1           (1 << 5)
#define SNIFF_SYNC_AT_WIFI_SYNC_USE_FILTER2           (1 << 6)
#define SNIFF_SYNC_AT_WIFI_WAKEUP_ADVANCE_ENABLE      (1 << 7)

typedef struct wlan_ext_sniff_sync_param_set
{
    uint8_t   Enable;
    uint8_t   ChannelNum;
    uint16_t  Reserved0;
    uint32_t  SyncFrameType;
    uint32_t  u32SniffSyncCfg;
    union{
        struct {
            uint32_t  WakeupPeriod_ms;       //unit:millisecond
            uint32_t  KeepActivePeriod_ms;   //unit:millisecond
            uint8_t   Flag;
            uint8_t   Reserved1;
            uint16_t  Reserved2;
            uint32_t  StartTime;            //unit:millisecond
        } __packed time_sync_at_host;
        struct {
            uint8_t   Flag;
            uint8_t   SyncDTIM;
            uint8_t   MaxLostSyncPacket;
            uint8_t   TSFOffset;                  //unit:byte
            uint8_t   AdaptiveExpansion;          //unit:millisecond
            uint8_t   KeepActiveNumAfterLostSync; //unit:millisecond
            uint16_t  ActiveTime;                 //unit:millisecond
            uint8_t   MaxAdaptiveExpansionLimit;  //unit:millisecond
            uint8_t   WakeupAdvanceTime;          //unit:millisecond
            uint16_t  MaxKeepAliveTime;           //unit:millisecond
        } __packed time_sync_at_wifi;
    } __packed;
} wlan_ext_sniff_sync_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SNIFF_KP_ACTIVE
 */
//u32Config
#define SNIFF_FRM_ALLOCATE_SEQ_NUM       (1 << 0)
typedef struct wlan_ext_sniff_kp_active_set
{
    uint32_t  Enable;
	uint32_t  u32Config;
} wlan_ext_sniff_kp_active_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_TEMP_FRM
 */
//u32Config
#define MAX_FRM_LEN       (694)
typedef struct wlan_ext_temp_frm_set
{
    uint16_t  FrmLength;
	uint8_t   Frame[MAX_FRM_LEN];
} wlan_ext_temp_frm_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SYNC_FRM_SEND
 */
#define SEND_SYNC_FRM_ADVANCE_ENABLE		(1 << 0)
typedef struct wlan_ext_send_sync_frm_set
{
    uint8_t   Enable;
	uint8_t   SendSyncFrmAdvanceTime;//unit:ms
	uint16_t  Flag;
	uint32_t  BcnInterval;
} wlan_ext_send_sync_frm_set_t;//units:1024us

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_BCN_WIN_CFG
 */
typedef struct wlan_ext_bcn_win_param_set
{
    uint32_t  BeaconWindowAdjAmpUs;
    uint8_t   BeaconWindowAdjStartNum;
    uint8_t   BeaconWindowAdjStopNum;
    uint8_t   BeaconWindowMaxStartNum;
	uint8_t   Reserved;
} wlan_ext_bcn_win_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_FORCE_B_RATE
 */
typedef struct wlan_ext_force_b_rate_set
{
    uint8_t   ForceBRateEnable;
    uint8_t   Force2mThreshold;
    uint8_t   Force1mThreshold;
	uint8_t   Reserved;
} wlan_ext_force_b_rate_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_POWER_LEVEL_TAB
 */
#define POWER_LEVEL_TAB_USE_LENGTH     11
typedef struct wlan_ext_power_level_tab_set
{
    uint16_t   PowerTab[POWER_LEVEL_TAB_USE_LENGTH];
} wlan_ext_power_level_tab_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_GET_POWER_LEVEL_TAB
 */
#define POWER_LEVEL_TAB_TYPE_MAX       0
#define POWER_LEVEL_TAB_TYPE_CUR       1
#define POWER_LEVEL_TAB_TYPE_USER      2
typedef struct wlan_ext_power_level_tab_get
{
    uint16_t   PowerTabType;
    uint16_t   PowerTab[POWER_LEVEL_TAB_USE_LENGTH];
} wlan_ext_power_level_tab_get_t;


int wlan_ext_request(struct netif *nif, wlan_ext_cmd_t cmd, uint32_t param);

#ifdef __cplusplus
}
#endif

#endif /* _NET_WLAN_WLAN_EXT_REQ_H_ */
