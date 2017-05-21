/*
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "attr_names.h"

#include <stddef.h>

#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>

#define TO_ATTR_NAME(a) {.attr = a, .name = #a}

static const struct attr_name unipro_l1_attrs[] = {
    TO_ATTR_NAME(TX_HSMODE_CAPABILITY),
    TO_ATTR_NAME(TX_HSGEAR_CAPABILITY),
    TO_ATTR_NAME(TX_PWMG0_CAPABILITY),
    TO_ATTR_NAME(TX_PWMGEAR_CAPABILITY),
    TO_ATTR_NAME(TX_AMPLITUDE_CAPABILITY),
    TO_ATTR_NAME(TX_EXTERNALSYNC_CAPABILITY),
    TO_ATTR_NAME(TX_HS_UNTERMINATED_LINE_DRIVE_CAPABILITY),
    TO_ATTR_NAME(TX_LS_TERMINATED_LINE_DRIVE_CAPABILITY),
    TO_ATTR_NAME(TX_MIN_SLEEP_NOCONFIG_TIME_CAPABILITY),
    TO_ATTR_NAME(TX_MIN_STALL_NOCONFIG_TIME_CAPABILITY),
    TO_ATTR_NAME(TX_MIN_SAVE_CONFIG_TIME_CAPABILITY),
    TO_ATTR_NAME(TX_REF_CLOCK_SHARED_CAPABILITY),
    TO_ATTR_NAME(TX_PHY_MAJORMINOR_RELEASE_CAPABILITY),
    TO_ATTR_NAME(TX_PHY_EDITORIAL_RELEASE_CAPABILITY),
    TO_ATTR_NAME(TX_HIBERN8TIME_CAPABILITY),
    TO_ATTR_NAME(TX_ADVANCED_GRANULARITY_CAPABILITY),
    TO_ATTR_NAME(TX_ADVANCED_HIBERN8TIME_CAPABILITY),
    TO_ATTR_NAME(TX_HS_EQUALIZER_SETTING_CAPABILITY),
    TO_ATTR_NAME(TX_MODE),
    TO_ATTR_NAME(TX_HSRATE_SERIES),
    TO_ATTR_NAME(TX_HSGEAR),
    TO_ATTR_NAME(TX_PWMGEAR),
    TO_ATTR_NAME(TX_AMPLITUDE),
    TO_ATTR_NAME(TX_HS_SLEWRATE),
    TO_ATTR_NAME(TX_SYNC_SOURCE),
    TO_ATTR_NAME(TX_HS_SYNC_LENGTH),
    TO_ATTR_NAME(TX_HS_PREPARE_LENGTH),
    TO_ATTR_NAME(TX_LS_PREPARE_LENGTH),
    TO_ATTR_NAME(TX_HIBERN8_CONTROL),
    TO_ATTR_NAME(TX_LCC_ENABLE),
    TO_ATTR_NAME(TX_PWM_BURST_CLOSURE_EXTENSION),
    TO_ATTR_NAME(TX_BYPASS_8B10B_ENABLE),
    TO_ATTR_NAME(TX_DRIVER_POLARITY),
    TO_ATTR_NAME(TX_HS_UNTERMINATED_LINE_DRIVE_ENABLE),
    TO_ATTR_NAME(TX_LS_TERMINATED_LINE_DRIVE_ENABLE),
    TO_ATTR_NAME(TX_LCC_SEQUENCER),
    TO_ATTR_NAME(TX_MIN_ACTIVATETIME),
    TO_ATTR_NAME(TX_ADVANCED_GRANULARITY_SETTING),
    TO_ATTR_NAME(TX_ADVANCED_GRANULARITY),
    TO_ATTR_NAME(TX_HS_EQUALIZER_SETTING),
    TO_ATTR_NAME(TX_FSM_STATE),
    TO_ATTR_NAME(MC_OUTPUT_AMPLITUDE),
    TO_ATTR_NAME(MC_HS_UNTERMINATED_ENABLE),
    TO_ATTR_NAME(MC_LS_TERMINATED_ENABLE),
    TO_ATTR_NAME(MC_HS_UNTERMINATED_LINE_DRIVE_ENABLE),
    TO_ATTR_NAME(MC_LS_TERMINATED_LINE_DRIVE_ENABLE),
    TO_ATTR_NAME(RX_HSMODE_CAPABILITY),
    TO_ATTR_NAME(RX_HSGEAR_CAPABILITY),
    TO_ATTR_NAME(RX_PWMG0_CAPABILITY),
    TO_ATTR_NAME(RX_PWMGEAR_CAPABILITY),
    TO_ATTR_NAME(RX_HS_UNTERMINATED_CAPABILITY),
    TO_ATTR_NAME(RX_LS_TERMINATED_CAPABILITY),
    TO_ATTR_NAME(RX_MIN_SLEEP_NOCONFIG_TIME_CAPABILITY),
    TO_ATTR_NAME(RX_MIN_STALL_NOCONFIG_TIME_CAPABILITY),
    TO_ATTR_NAME(RX_MIN_SAVE_CONFIG_TIME_CAPABILITY),
    TO_ATTR_NAME(RX_REF_CLOCK_SHARED_CAPABILITY),
    TO_ATTR_NAME(RX_HS_G1_SYNC_LENGTH_CAPABILITY),
    TO_ATTR_NAME(RX_HS_G1_PREPARE_LENGTH_CAPABILITY),
    TO_ATTR_NAME(RX_LS_PREPARE_LENGTH_CAPABILITY),
    TO_ATTR_NAME(RX_PWM_BURST_CLOSURE_LENGTH_CAPABILITY),
    TO_ATTR_NAME(RX_MIN_ACTIVATETIME_CAPABILITY),
    TO_ATTR_NAME(RX_PHY_MAJORMINOR_RELEASE_CAPABILITY),
    TO_ATTR_NAME(RX_PHY_EDITORIAL_RELEASE_CAPABILITY),
    TO_ATTR_NAME(RX_HIBERN8TIME_CAPABILITY),
    TO_ATTR_NAME(RX_HS_G2_SYNC_LENGTH_CAPABILITY),
    TO_ATTR_NAME(RX_HS_G3_SYNC_LENGTH_CAPABILITY),
    TO_ATTR_NAME(RX_HS_G2_PREPARE_LENGTH_CAPABILITY),
    TO_ATTR_NAME(RX_HS_G3_PREPARE_LENGTH_CAPABILITY),
    TO_ATTR_NAME(RX_ADVANCED_GRANULARITY_CAPABILITY),
    TO_ATTR_NAME(RX_ADVANCED_HIBERN8TIME_CAPABILITY),
    TO_ATTR_NAME(RX_ADVANCED_MIN_ACTIVATETIME_CAPABILITY),
    TO_ATTR_NAME(RX_MODE),
    TO_ATTR_NAME(RX_HSRATE_SERIES),
    TO_ATTR_NAME(RX_HSGEAR),
    TO_ATTR_NAME(RX_PWMGEAR),
    TO_ATTR_NAME(RX_LS_TERMINATED_ENABLE),
    TO_ATTR_NAME(RX_HS_UNTERMINATED_ENABLE),
    TO_ATTR_NAME(RX_ENTER_HIBERN8),
    TO_ATTR_NAME(RX_BYPASS_8B10B_ENABLE),
    TO_ATTR_NAME(RX_TERMINATION_FORCE_ENABLE),
    TO_ATTR_NAME(RX_FSM_STATE),
    TO_ATTR_NAME(OMC_TYPE_CAPABILITY),
    TO_ATTR_NAME(MC_HSMODE_CAPABILITY),
    TO_ATTR_NAME(MC_HSGEAR_CAPABILITY),
    TO_ATTR_NAME(MC_HS_START_TIME_VAR_CAPABILITY),
    TO_ATTR_NAME(MC_HS_START_TIME_RANGE_CAPABILITY),
    TO_ATTR_NAME(MC_RX_SA_CAPABILITY),
    TO_ATTR_NAME(MC_RX_LA_CAPABILITY),
    TO_ATTR_NAME(MC_LS_PREPARE_LENGTH),
    TO_ATTR_NAME(MC_PWMG0_CAPABILITY),
    TO_ATTR_NAME(MC_PWMGEAR_CAPABILITY),
    TO_ATTR_NAME(MC_LS_TERMINATED_CAPABILITY),
    TO_ATTR_NAME(MC_HS_UNTERMINATED_CAPABILITY),
    TO_ATTR_NAME(MC_LS_TERMINATED_LINE_DRIVE_CAPABILITY),
    TO_ATTR_NAME(MC_HS_UNTERMINATED_LINE_DRIVE_CAPABILIT),
    TO_ATTR_NAME(MC_MFG_ID_PART1),
    TO_ATTR_NAME(MC_MFG_ID_PART2),
    TO_ATTR_NAME(MC_PHY_MAJORMINOR_RELEASE_CAPABILITY),
    TO_ATTR_NAME(MC_PHY_EDITORIAL_RELEASE_CAPABILITY),
    TO_ATTR_NAME(MC_VENDOR_INFO_PART1),
    TO_ATTR_NAME(MC_VENDOR_INFO_PART2),
    TO_ATTR_NAME(MC_VENDOR_INFO_PART3),
    TO_ATTR_NAME(MC_VENDOR_INFO_PART4),
    {.attr = 0, .name = NULL},
};

static const struct attr_name unipro_l1_5_attrs[] = {
    TO_ATTR_NAME(PA_PHYTYPE),
    TO_ATTR_NAME(PA_AVAILTXDATALANES),
    TO_ATTR_NAME(PA_AVAILRXDATALANES),
    TO_ATTR_NAME(PA_MINRXTRAILINGCLOCKS),
    TO_ATTR_NAME(PA_TXHSG1SYNCLENGTH),
    TO_ATTR_NAME(PA_TXHSG1PREPARELENGTH),
    TO_ATTR_NAME(PA_TXHSG2SYNCLENGTH),
    TO_ATTR_NAME(PA_TXHSG2PREPARELENGTH),
    TO_ATTR_NAME(PA_TXHSG3SYNCLENGTH),
    TO_ATTR_NAME(PA_TXHSG3PREPARELENGTH),
    TO_ATTR_NAME(PA_TXMK2EXTENSION),
    TO_ATTR_NAME(PA_PEERSCRAMBLING),
    TO_ATTR_NAME(PA_TXSKIP),
    TO_ATTR_NAME(PA_SAVECONFIGEXTENSIONENABLE),
    TO_ATTR_NAME(PA_LOCALTXLCCENABLE),
    TO_ATTR_NAME(PA_PEERTXLCCENABLE),
    TO_ATTR_NAME(PA_ACTIVETXDATALANES),
    TO_ATTR_NAME(PA_CONNECTEDTXDATALANES),
    TO_ATTR_NAME(PA_TXTRAILINGCLOCKS),
    TO_ATTR_NAME(PA_TXPWRSTATUS),
    TO_ATTR_NAME(PA_TXGEAR),
    TO_ATTR_NAME(PA_TXTERMINATION),
    TO_ATTR_NAME(PA_HSSERIES),
    TO_ATTR_NAME(PA_PWRMODE),
    TO_ATTR_NAME(PA_ACTIVERXDATALANES),
    TO_ATTR_NAME(PA_CONNECTEDRXDATALANES),
    TO_ATTR_NAME(PA_RXPWRSTATUS),
    TO_ATTR_NAME(PA_RXGEAR),
    TO_ATTR_NAME(PA_RXTERMINATION),
    TO_ATTR_NAME(PA_SCRAMBLING),
    TO_ATTR_NAME(PA_MAXRXPWMGEAR),
    TO_ATTR_NAME(PA_MAXRXHSGEAR),
    TO_ATTR_NAME(PA_PACPREQTIMEOUT),
    TO_ATTR_NAME(PA_PACPREQEOBTIMEOUT),
    TO_ATTR_NAME(PA_REMOTEVERINFO),
    TO_ATTR_NAME(PA_LOGICALLANEMAP),
    TO_ATTR_NAME(PA_SLEEPNOCONFIGTIME),
    TO_ATTR_NAME(PA_STALLNOCONFIGTIME),
    TO_ATTR_NAME(PA_SAVECONFIGTIME),
    TO_ATTR_NAME(PA_RXHSUNTERMINATIONCAPABILITY),
    TO_ATTR_NAME(PA_RXLSTERMINATIONCAPABILITY),
    TO_ATTR_NAME(PA_HIBERN8TIME),
    TO_ATTR_NAME(PA_TACTIVATE),
    TO_ATTR_NAME(PA_LOCALVERINFO),
    TO_ATTR_NAME(PA_GRANULARITY),
    TO_ATTR_NAME(PA_MK2EXTENSIONGUARDBAND),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA0),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA1),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA2),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA3),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA4),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA5),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA6),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA7),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA8),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA9),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA10),
    TO_ATTR_NAME(PA_PWRMODEUSERDATA11),
    TO_ATTR_NAME(PA_PACPFRAMECOUNT),
    TO_ATTR_NAME(PA_PACPERRORCOUNT),
    TO_ATTR_NAME(PA_PHYTESTCONTROL),
    {.attr = 0, .name = NULL},
};

static const struct attr_name unipro_l2_attrs[] = {
    TO_ATTR_NAME(DL_TXPREEMPTIONCAP),
    TO_ATTR_NAME(DL_TC0TXMAXSDUSIZE),
    TO_ATTR_NAME(DL_TC0RXINITCREDITVAL),
    TO_ATTR_NAME(DL_TC1TXMAXSDUSIZE),
    TO_ATTR_NAME(DL_TC1RXINITCREDITVAL),
    TO_ATTR_NAME(DL_TC0TXBUFFERSIZE),
    TO_ATTR_NAME(DL_TC1TXBUFFERSIZE),
    TO_ATTR_NAME(DL_TC0TXFCTHRESHOLD),
    TO_ATTR_NAME(DL_FC0PROTECTIONTIMEOUTVAL),
    TO_ATTR_NAME(DL_TC0REPLAYTIMEOUTVAL),
    TO_ATTR_NAME(DL_AFC0REQTIMEOUTVAL),
    TO_ATTR_NAME(DL_AFC0CREDITTHRESHOLD),
    TO_ATTR_NAME(DL_TC0OUTACKTHRESHOLD),
    TO_ATTR_NAME(DL_PEERTC0PRESENT),
    TO_ATTR_NAME(DL_PEERTC0RXINITCREDITVAL),
    TO_ATTR_NAME(DL_TC1TXFCTHRESHOLD),
    TO_ATTR_NAME(DL_FC1PROTECTIONTIMEOUTVAL),
    TO_ATTR_NAME(DL_TC1REPLAYTIMEOUTVAL),
    TO_ATTR_NAME(DL_AFC1REQTIMEOUTVAL),
    TO_ATTR_NAME(DL_AFC1CREDITTHRESHOLD),
    TO_ATTR_NAME(DL_TC1OUTACKTHRESHOLD),
    TO_ATTR_NAME(DL_PEERTC1PRESENT),
    TO_ATTR_NAME(DL_PEERTC1RXINITCREDITVAL),
    {.attr = 0, .name = NULL},
};

static const struct attr_name unipro_l3_attrs[] = {
    TO_ATTR_NAME(N_DEVICEID),
    TO_ATTR_NAME(N_DEVICEID_VALID),
    TO_ATTR_NAME(N_TC0TXMAXSDUSIZE),
    TO_ATTR_NAME(N_TC1TXMAXSDUSIZE),
    {.attr = 0, .name = NULL},
};

static const struct attr_name unipro_l4_attrs[] = {
    TO_ATTR_NAME(T_NUMCPORTS),
    TO_ATTR_NAME(T_NUMTESTFEATURES),
    TO_ATTR_NAME(T_TC0TXMAXSDUSIZE),
    TO_ATTR_NAME(T_TC1TXMAXSDUSIZE),
    TO_ATTR_NAME(T_TSTCPORTID),
    TO_ATTR_NAME(T_TSTSRCON),
    TO_ATTR_NAME(T_TSTSRCPATTERN),
    TO_ATTR_NAME(T_TSTSRCINCREMENT),
    TO_ATTR_NAME(T_TSTSRCMESSAGESIZE),
    TO_ATTR_NAME(T_TSTSRCMESSAGECOUNT),
    TO_ATTR_NAME(T_TSTSRCINTERMESSAGEGAP),
    TO_ATTR_NAME(T_TSTDSTON),
    TO_ATTR_NAME(T_TSTDSTERRORDETECTIONENABLE),
    TO_ATTR_NAME(T_TSTDSTPATTERN),
    TO_ATTR_NAME(T_TSTDSTINCREMENT),
    TO_ATTR_NAME(T_TSTDSTMESSAGECOUNT),
    TO_ATTR_NAME(T_TSTDSTMESSAGEOFFSET),
    TO_ATTR_NAME(T_TSTDSTMESSAGESIZE),
    TO_ATTR_NAME(T_TSTDSTFCCREDITS),
    TO_ATTR_NAME(T_TSTDSTINTERFCTOKENGAP),
    TO_ATTR_NAME(T_TSTDSTINITIALFCCREDITS),
    TO_ATTR_NAME(T_TSTDSTERRORCODE),
    TO_ATTR_NAME(T_PEERDEVICEID),
    TO_ATTR_NAME(T_PEERCPORTID),
    TO_ATTR_NAME(T_CONNECTIONSTATE),
    TO_ATTR_NAME(T_TRAFFICCLASS),
    TO_ATTR_NAME(T_PROTOCOLID),
    TO_ATTR_NAME(T_CPORTFLAGS),
    TO_ATTR_NAME(T_TXTOKENVALUE),
    TO_ATTR_NAME(T_RXTOKENVALUE),
    TO_ATTR_NAME(T_LOCALBUFFERSPACE),
    TO_ATTR_NAME(T_PEERBUFFERSPACE),
    TO_ATTR_NAME(T_CREDITSTOSEND),
    TO_ATTR_NAME(T_CPORTMODE),
    {.attr = 0, .name = NULL},
};

static const struct attr_name unipro_dme_attrs[] = {
    TO_ATTR_NAME(DME_DDBL1_REVISION),
    TO_ATTR_NAME(DME_DDBL1_LEVEL),
    TO_ATTR_NAME(DME_DDBL1_DEVICECLASS),
    TO_ATTR_NAME(DME_DDBL1_MANUFACTURERID),
    TO_ATTR_NAME(DME_DDBL1_PRODUCTID),
    TO_ATTR_NAME(DME_DDBL1_LENGTH),
    TO_ATTR_NAME(DME_FC0PROTECTIONTIMEOUTVAL),
    TO_ATTR_NAME(DME_TC0REPLAYTIMEOUTVAL),
    TO_ATTR_NAME(DME_AFC0REQTIMEOUTVAL),
    TO_ATTR_NAME(DME_FC1PROTECTIONTIMEOUTVAL),
    TO_ATTR_NAME(DME_TC1REPLAYTIMEOUTVAL),
    TO_ATTR_NAME(DME_AFC1REQTIMEOUTVAL),
    {.attr = 0, .name = NULL},
};

static const struct attr_name unipro_tsb_attrs[] = {
    TO_ATTR_NAME(TSB_DME_DDBL2_A),
    TO_ATTR_NAME(TSB_DME_DDBL2_B),
    TO_ATTR_NAME(TSB_MAILBOX),
    TO_ATTR_NAME(TSB_DME_LAYERENABLEREQ),
    TO_ATTR_NAME(TSB_DME_LAYERENABLECNF),
    TO_ATTR_NAME(TSB_DME_RESETREQ),
    TO_ATTR_NAME(TSB_DME_RESETCNF),
    TO_ATTR_NAME(TSB_DME_ENDPOINTRESETREQ),
    TO_ATTR_NAME(TSB_DME_ENDPOINTRESETCNF),
    TO_ATTR_NAME(TSB_DME_ENDPOINTRESETIND),
    TO_ATTR_NAME(TSB_DME_LINKSTARTUPREQ),
    TO_ATTR_NAME(TSB_DME_LINKSTARTUPCNF),
    TO_ATTR_NAME(TSB_DME_LINKSTARTUPIND),
    TO_ATTR_NAME(TSB_DME_LINKLOSTIND),
    TO_ATTR_NAME(TSB_DME_HIBERNATEENTERREQ),
    TO_ATTR_NAME(TSB_DME_HIBERNATEENTERCNF),
    TO_ATTR_NAME(TSB_DME_HIBERNATEENTERIND),
    TO_ATTR_NAME(TSB_DME_HIBERNATEEXITREQ),
    TO_ATTR_NAME(TSB_DME_HIBERNATEEXITCNF),
    TO_ATTR_NAME(TSB_DME_HIBERNATEEXITIND),
    TO_ATTR_NAME(TSB_DME_POWERMODEIND),
    TO_ATTR_NAME(TSB_DME_TESTMODEREQ),
    TO_ATTR_NAME(TSB_DME_TESTMODECNF),
    TO_ATTR_NAME(TSB_DME_TESTMODEIND),
    TO_ATTR_NAME(TSB_DME_ERRORPHYIND),
    TO_ATTR_NAME(TSB_DME_ERRORPAIND),
    TO_ATTR_NAME(TSB_DME_ERRORDIND),
    TO_ATTR_NAME(TSB_DME_ERRORNIND),
    TO_ATTR_NAME(TSB_DME_ERRORTIND),
    TO_ATTR_NAME(TSB_INTERRUPTENABLE),
    TO_ATTR_NAME(TSB_INTERRUPTSTATUS),
    TO_ATTR_NAME(TSB_L2STATUS),
    TO_ATTR_NAME(TSB_POWERSTATE),
    TO_ATTR_NAME(TSB_TXBURSTCLOSUREDELAY),
    TO_ATTR_NAME(TSB_MPHYCFGUPDT),
    TO_ATTR_NAME(TSB_ADJUSTTRAILINGCLOCKS),
    TO_ATTR_NAME(TSB_SUPPRESSRREQ),
    TO_ATTR_NAME(TSB_L2TIMEOUT),
    TO_ATTR_NAME(TSB_MAXSEGMENTCONFIG),
    TO_ATTR_NAME(TSB_TBD),
    TO_ATTR_NAME(TSB_RBD),
    TO_ATTR_NAME(TSB_DEBUGTXBYTECOUNT),
    TO_ATTR_NAME(TSB_DEBUGRXBYTECOUNT),
    TO_ATTR_NAME(TSB_DEBUGINVALIDBYTEENABLE),
    TO_ATTR_NAME(TSB_DEBUGLINKSTARTUP),
    TO_ATTR_NAME(TSB_DEBUGPWRCHANGE),
    TO_ATTR_NAME(TSB_DEBUGSTATES),
    TO_ATTR_NAME(TSB_DEBUGCOUNTER0),
    TO_ATTR_NAME(TSB_DEBUGCOUNTER1),
    TO_ATTR_NAME(TSB_DEBUGCOUNTER0MASK),
    TO_ATTR_NAME(TSB_DEBUGCOUNTER1MASK),
    TO_ATTR_NAME(TSB_DEBUGCOUNTERCONTROL),
    TO_ATTR_NAME(TSB_DEBUGCOUNTEROVERFLOW),
    TO_ATTR_NAME(TSB_DEBUGOMC),
    TO_ATTR_NAME(TSB_DEBUGCOUNTERBMASK),
    TO_ATTR_NAME(TSB_DEBUGSAVECONFIGTIME),
    TO_ATTR_NAME(TSB_DEBUGCLOCKENABLE),
    TO_ATTR_NAME(TSB_DEEPSTALLCFG),
    TO_ATTR_NAME(TSB_DEEPSTALLSTATUS),
    {.attr = 0, .name = NULL},
};

const struct attr_name_group unipro_l1_attr_group = {
    .attr_names = unipro_l1_attrs,
    .group_name = "PHY layer (L1)",
};

const struct attr_name_group unipro_l1_5_attr_group = {
    .attr_names = unipro_l1_5_attrs,
    .group_name = "PHY adapter layer (L1.5)",
};

const struct attr_name_group unipro_l2_attr_group = {
    .attr_names = unipro_l2_attrs,
    .group_name = "Link layer (L2)",
};

const struct attr_name_group unipro_l3_attr_group = {
    .attr_names = unipro_l3_attrs,
    .group_name = "Network layer (L3)",
};

const struct attr_name_group unipro_l4_attr_group = {
    .attr_names = unipro_l4_attrs,
    .group_name = "Transport layer (L4)",
};

const struct attr_name_group unipro_dme_attr_group = {
    .attr_names = unipro_dme_attrs,
    .group_name = "DME",
};

const struct attr_name_group unipro_tsb_attr_group = {
    .attr_names = unipro_tsb_attrs,
    .group_name = "TSB",
};

static const char* attr_group_get_name(uint16_t attr,
                                       const struct attr_name_group *group)
{
    const struct attr_name *an = group->attr_names;
    while (an->name) {
        if (attr == an->attr) {
            return an->name;
        }
        an++;
    }
    return NULL;
}

/**
 * @brief Find an attribute's name
 * @param attr Attribute whose string name to find
 * @return Printable name of the attribute, or NULL.
 */
const char* attr_get_name(uint16_t attr)
{
    const char *ret;

    ret = attr_group_get_name(attr, &unipro_l1_attr_group);
    if (ret) {
        return ret;
    }
    ret = attr_group_get_name(attr, &unipro_l1_5_attr_group);
    if (ret) {
        return ret;
    }
    ret = attr_group_get_name(attr, &unipro_l2_attr_group);
    if (ret) {
        return ret;
    }
    ret = attr_group_get_name(attr, &unipro_l3_attr_group);
    if (ret) {
        return ret;
    }
    ret = attr_group_get_name(attr, &unipro_l4_attr_group);
    if (ret) {
        return ret;
    }
    ret = attr_group_get_name(attr, &unipro_dme_attr_group);
    if (ret) {
        return ret;
    }

    return attr_group_get_name(attr, &unipro_tsb_attr_group);
}
