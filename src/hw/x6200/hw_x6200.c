/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  OpenX6X00 - the X6200 behind hw.h: Aether Radio's X6200 control library (the base over its
 *  serial link and I2C, the GPIO pins), the base's register layout and its packet. The X6200's
 *  base computes the spectrum itself and sends 512 dB bins instead of IQ samples (HW_CAP_PSD),
 *  and it reports the POWER key in the packet. Built from hw/x6100/hw_x6100.c.
 *
 *  Copyright (c) 2022-2023 Belousov Oleg aka R1CBU (the flow decoding, the init sequence)
 *  Copyright (c) 2026 RF.Guru
 */

#define _GNU_SOURCE
#include "../hw.h"
#include "../../base_guard.h"
#include "../../cfg/cfg_api.h"

#include <aether_radio/x6200_control/control.h>
#include <aether_radio/x6200_control/low/flow.h>
#include <aether_radio/x6200_control/low/gpio.h>

#include <ctype.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

/* The GUI's enums (radio_types.h) are the library's numbers, except the modes: the X6200 has
 * SAM between AM and NFM (lib_mode() below) */
#define SAME(a, b) _Static_assert((int)(a) == (int)(b), #a " is not " #b)
SAME(RADIO_VFO_A, X6200_VFO_A);          SAME(RADIO_VFO_B, X6200_VFO_B);
SAME(radio_mode_lsb, x6200_mode_lsb);    SAME(radio_mode_lsb_dig, x6200_mode_lsb_dig);
SAME(radio_mode_usb, x6200_mode_usb);    SAME(radio_mode_usb_dig, x6200_mode_usb_dig);
SAME(radio_mode_cw, x6200_mode_cw);      SAME(radio_mode_cwr, x6200_mode_cwr);
SAME(radio_mode_am, x6200_mode_am);
SAME(radio_agc_off, x6200_agc_off);      SAME(radio_agc_slow, x6200_agc_slow);
SAME(radio_agc_fast, x6200_agc_fast);    SAME(radio_agc_auto, x6200_agc_auto);
SAME(radio_att_off, x6200_att_off);      SAME(radio_att_on, x6200_att_on);
SAME(radio_pre_off, x6200_pre_off);      SAME(radio_pre_on, x6200_pre_on);
SAME(radio_key_manual, x6200_key_manual); SAME(radio_key_auto_left, x6200_key_auto_left);
SAME(radio_key_auto_right, x6200_key_auto_right);
SAME(radio_iambic_a, x6200_iambic_a);    SAME(radio_iambic_b, x6200_iambic_b);
SAME(radio_mic_builtin, x6200_mic_builtin); SAME(radio_mic_handle, x6200_mic_handle);
SAME(radio_mic_auto, x6200_mic_auto);
SAME(RADIO_HKEY_SPCH, X6200_HKEY_SPCH);  SAME(RADIO_HKEY_TUNER, X6200_HKEY_TUNER);
SAME(RADIO_HKEY_XFC, X6200_HKEY_XFC);    SAME(RADIO_HKEY_UP, X6200_HKEY_UP);
SAME(RADIO_HKEY_DOWN, X6200_HKEY_DOWN);  SAME(RADIO_HKEY_VM, X6200_HKEY_VM);
SAME(RADIO_HKEY_NW, X6200_HKEY_NW);      SAME(RADIO_HKEY_F1, X6200_HKEY_F1);
SAME(RADIO_HKEY_F2, X6200_HKEY_F2);      SAME(RADIO_HKEY_1, X6200_HKEY_1);
SAME(RADIO_HKEY_2, X6200_HKEY_2);        SAME(RADIO_HKEY_3, X6200_HKEY_3);
SAME(RADIO_HKEY_4, X6200_HKEY_4);        SAME(RADIO_HKEY_5, X6200_HKEY_5);
SAME(RADIO_HKEY_6, X6200_HKEY_6);        SAME(RADIO_HKEY_7, X6200_HKEY_7);
SAME(RADIO_HKEY_8, X6200_HKEY_8);        SAME(RADIO_HKEY_9, X6200_HKEY_9);
SAME(RADIO_HKEY_DOT, X6200_HKEY_DOT);    SAME(RADIO_HKEY_0, X6200_HKEY_0);
SAME(RADIO_HKEY_CE, X6200_HKEY_CE);      SAME(RADIO_HKEY_MODE, X6200_HKEY_MODE);
SAME(RADIO_HKEY_FIL, X6200_HKEY_FIL);    SAME(RADIO_HKEY_GENE, X6200_HKEY_GENE);
SAME(RADIO_HKEY_FINP, X6200_HKEY_FINP);
#undef SAME

/* The GUI numbers SAM and WFM after NFM, the base has SAM before it */
static x6200_mode_t lib_mode(radio_mode_t m) {
    switch (m) {
        case radio_mode_nfm:    return x6200_mode_nfm;
        case radio_mode_sam:    return x6200_mode_sam;
        case radio_mode_wfm:    return x6200_mode_wfm;
        default:                return (x6200_mode_t)m;
    }
}

/* Bits of x6200_sple_atue_trx that key the radio from the GUI */
#define TX_BITS (x6200_iptt | x6200_modem_trx | x6200_atu_tune | x6200_swrscan_trx | x6200_calibration_trx)

#define PSD_BINS    512     /* dB bins in a packet, the lowest frequency first */
#define PSD_OFFSET  126.0f  /* bin value 126 is 0 dBm: the stock X6200 GUI's constant */
#define PSD_SPAN_HZ 384000  /* the bins at decimation 0; halved per step (the stock GUI's FFT_FULL_WIDTH) */
#define PSD_MAX_DEC 3       /* the stock GUI goes no further (48 kHz); what the base does beyond is unknown */
#define SMETER_CORR 4       /* dB the stock GUI adds to the base's S-meter reading */

static x6200_flow_t *pack;
static float         psd_db[PSD_BINS];
static uint8_t       fft_dec;       /* the base's spectrum decimation as last set */
static char          ver_str[32] = "?";
static radio_base_ver_t ver;

/* The notch: the GUI sets on/off and auto apart, the base takes one mode */
static bool dnf_on, dnf_auto;

/* ---- The library's writes, watched (base_guard.h) ---------------------------------------- */

#define TRIES       3
#define RETRY_US    2000

static bool (*lib_cmd)(x6200_cmd_enum_t cmd, uint32_t arg);

__attribute__((constructor)) static void find_lib_cmd(void) {
    lib_cmd = (bool (*)(x6200_cmd_enum_t, uint32_t))dlsym(RTLD_NEXT, "x6200_control_cmd");
    if (!lib_cmd) {
        fprintf(stderr, "hw_x6200: x6200_control_cmd not found in the control library\n");
        abort();
    }
}

/* The library's own name: its setters call it through the PLT, so every write lands here (see
 * hw_x6100.c and base_guard.h for the why) */
bool x6200_control_cmd(x6200_cmd_enum_t cmd, uint32_t arg) {
    uint32_t prev = x6200_control_get(cmd);
    int      fails = 0;
    bool     ok;

    while (!(ok = lib_cmd(cmd, arg)) && ++fails < TRIES) {
        usleep(RETRY_US);
    }
    if (fails) {
        base_guard_write_failed(cmd, ok);
    }
    if (cmd == x6200_sple_atue_trx && (prev & TX_BITS) && !(arg & TX_BITS)) {
        base_guard_unkeyed();
    }
    return ok;
}

static void wr12_start(void);
static void wr12_vfo_switched(radio_vfo_t vfo);
static void wr12_split_set(bool on);

/* ---- Init ----------------------------------------------------------------------------- */

bool hw_init(void) {
    if (!x6200_gpio_init()) {
        return false;
    }
    x6200_gpio_set(x6200_pin_morse_key, 1);     /* Morse key off */
    if (!x6200_flow_init()) {
        return false;
    }
    wr12_start();
    pack = malloc(sizeof(x6200_flow_t));
    return pack != NULL;
}

bool hw_control_init(void) {
    return x6200_control_init();
}

void hw_bb_reset(void) {
    x6200_gpio_set(x6200_pin_bb_reset, 1);
    usleep(100000);
    x6200_gpio_set(x6200_pin_bb_reset, 0);
}

/* The base's info block: the library hands back its first 128 bytes; the firmware version is
 * the string at 0x41 (its own init prints it as "BASE fw version"). Kept as major.minor.patch
 * for the GUI, revision 0: the X6200's capabilities do not depend on it */
void hw_start(void) {
    const char *info = x6200_control_get_fw_version();
    const char *s    = info ? info + 0x41 : NULL;

    memset(&ver, 0, sizeof(ver));
    if (s && *s) {
        snprintf(ver_str, sizeof(ver_str), "%s", s);
        while (*s && !isdigit((unsigned char)*s)) {
            s++;
        }
        unsigned a = 0, b = 0, c = 0;
        if (sscanf(s, "%u.%u.%u", &a, &b, &c) >= 2) {
            ver = (radio_base_ver_t){a, b, c, 0};
        }
    }
}

radio_base_ver_t hw_base_ver(void) {
    return ver;
}

const char *hw_base_ver_str(void) {
    return ver_str;
}

/* The base narrows its spectrum for the zoom, finds the notch's carrier itself, sends the
 * spectrum ready-made, and has a compressor with a ratio. No IF shift, no TX calibration of
 * the X6100's kind, no CESSB, no TX filter. In CW and CW-R its spectrum stays centred on the
 * dial, and the tone rises as the dial goes up in both: a broadcast carrier on the dial sounded
 * at the key tone and sat in the middle of the base's spectrum, base 1.0.8 (#115) */
uint32_t hw_caps(void) {
    return HW_CAP_ZOOM | HW_CAP_DNF_AUTO | HW_CAP_PSD | HW_CAP_COMP | HW_CAP_TX_LEVEL | HW_CAP_SAM_WFM | HW_CAP_KEEPALIVE
         | HW_CAP_TX_NO_ALC | HW_CAP_CW_SPECTRUM_ON_DIAL | HW_CAP_CW_TONE_RISES;
}

/* ---- Settings ------------------------------------------------------------------------- */

/* The settings with a register of their own: compared before the write, read back by hw_get_i() */
static int reg_of(hw_param_t p) {
    switch (p) {
        case HW_RIT:            return x6200_rit;
        case HW_XIT:            return x6200_xit;
        case HW_AGC_TIME:       return x6200_agctime;
        case HW_ATU_NETWORK:    return x6200_atu_network;
        default:                return -1;
    }
}

int32_t hw_get_i(hw_param_t p) {
    int      reg = reg_of(p);
    uint32_t f;

    switch (p) {
        case HW_FILTER1_LOW:
            f = x6200_control_get(x6200_rxfilter);
            return (int16_t)(f >> 16);
        case HW_FILTER1_HIGH:
            f = x6200_control_get(x6200_rxfilter);
            return (int16_t)(f & 0xFFFF);
        default:
            return reg < 0 ? 0 : (int32_t)x6200_control_get((x6200_cmd_enum_t)reg);
    }
}

static void dnf_apply(void) {
    x6200_control_dnf_set(!dnf_on ? x6200_dnf_off : dnf_auto ? x6200_dnf_auto : x6200_dnf_manual);
}

bool hw_set_i(hw_param_t p, int32_t v) {
    int reg = reg_of(p);

    if (reg >= 0) {
        if (x6200_control_get((x6200_cmd_enum_t)reg) == (uint32_t)v) {
            return false;
        }
        x6200_control_cmd((x6200_cmd_enum_t)reg, (uint32_t)v);
        return true;
    }
    switch (p) {
        case HW_VFO_CUR:        x6200_control_vfo_set((x6200_vfo_t)v); wr12_vfo_switched((radio_vfo_t)v); break;
        case HW_SPLIT:          wr12_split_set(v); break;
        case HW_RFG:            x6200_control_rfg_set(v); break;
        /* One squelch setting in the GUI, two on the base (voice and FM): both follow it */
        case HW_SQL:            x6200_control_sql_set(v); x6200_control_sql_fm_set(v); break;
        case HW_RXVOL:          x6200_control_rxvol_set(v); break;
        case HW_ATU_ENABLED:    x6200_control_atu_set(v); break;
        case HW_ATU_TUNE:       x6200_control_atu_tune(v); break;
        case HW_SWRSCAN:        x6200_control_swrscan_set(v); break;
        case HW_PTT:            x6200_control_ptt_set(v); break;
        case HW_MODEM:          x6200_control_modem_set(v); break;
        case HW_RECORD:         x6200_control_record_set(v); break;
        case HW_MIC:            x6200_control_mic_set((x6200_mic_sel_t)v); break;
        case HW_HMIC:           x6200_control_hmic_set(v); break;
        case HW_IMIC:           x6200_control_imic_set(v); break;
        case HW_MONI:           x6200_control_monitor_level_set(v); break;
        case HW_LINE_IN:        x6200_control_linein_set(v); break;
        case HW_LINE_OUT:       x6200_control_lineout_set(v); break;
        case HW_SPMODE:         x6200_control_spmode_set(v); break;
        case HW_CHARGER:        x6200_control_charger_set(v); break;
        case HW_BIAS_DRIVE:     x6200_control_bias_drive_set(v); break;
        case HW_BIAS_FINAL:     x6200_control_bias_final_set(v); break;
        /* The GUI asks for 100 kHz over 2^v; the base's steps are 384 kHz over 2^dec: one step
         * further than asked keeps the stream wider than the screen (192 kHz over 2^v), the GUI
         * takes the middle of it (hw_pack_t.psd_span) */
        case HW_FFT_DEC:
            fft_dec = v + 1 > PSD_MAX_DEC ? PSD_MAX_DEC : v + 1;
            x6200_control_fft_dec_set(fft_dec);
            break;
        /* One RX filter register: low and high halves. The second filter (the X6100's wider one
         * around the first) does not exist here */
        case HW_FILTER1_LOW:
            if (hw_get_i(HW_FILTER1_LOW) == v) return false;
            x6200_control_rx_filter_set_low(v);
            break;
        case HW_FILTER1_HIGH:
            if (hw_get_i(HW_FILTER1_HIGH) == v) return false;
            x6200_control_rx_filter_set_high(v);
            break;
        /* radio.c speaks the X6100's compressor: COMP_ON true is OFF there, and COMP_LEVEL is the ratio minus 2
         * (0-6). The X6200's bit is a plain on/off and its levels are off, 1:2, 1:4, 1:8 (x6200_comp_level_t;
         * gdyuldin's GUI sends the bit as it is): "Comp off" turned it on, and ratios over 1:5 went past the end */
        case HW_COMP_ON:        x6200_control_comp_set(!v); break;
        case HW_COMP_LEVEL:     x6200_control_comp_level_set(v + 2 <= 3 ? x6200_comp_1_2 : v + 2 <= 7 ? x6200_comp_1_4
                                                                                                  : x6200_comp_1_8);
                                break;
        case HW_VOX:            x6200_control_vox_set(v); break;
        case HW_VOX_GAIN:       x6200_control_vox_gain_set(v); break;
        case HW_VOX_AG:         x6200_control_vox_ag_set(v); break;
        case HW_VOX_DELAY:      x6200_control_vox_delay_set(v); break;
        case HW_KEY_TONE:       x6200_control_key_tone_set(v); break;
        case HW_KEY_SPEED:      x6200_control_key_speed_set(v); break;
        case HW_KEY_MODE:       x6200_control_key_mode_set((x6200_key_mode_t)v); break;
        case HW_IAMBIC_MODE:    x6200_control_iambic_mode_set((x6200_iambic_mode_t)v); break;
        case HW_KEY_VOL:        x6200_control_key_vol_set(v); break;
        case HW_KEY_TRAIN:      x6200_control_key_train_set(v); break;
        case HW_QSK_TIME:       x6200_control_qsk_time_set(v); break;
        case HW_AGC_HANG:       x6200_control_agc_hang_set(v); break;
        case HW_AGC_KNEE:       x6200_control_agc_knee_set(v); break;
        case HW_AGC_SLOPE:      x6200_control_agc_slope_set(v); break;
        case HW_DNF:            dnf_on = v; dnf_apply(); break;
        case HW_DNF_AUTO:       dnf_auto = v; dnf_apply(); break;
        case HW_DNF_CENTER:     x6200_control_dnf_center_set(v); break;
        case HW_DNF_WIDTH:      x6200_control_dnf_width_set(v); break;
        case HW_NB:             x6200_control_nb_set(v); break;
        case HW_NB_LEVEL:       x6200_control_nb_level_set(v); break;
        case HW_NB_WIDTH:       x6200_control_nb_width_set(v); break;
        case HW_NR:             x6200_control_nr_set(v); break;
        case HW_NR_LEVEL:       x6200_control_nr_level_set(v); break;
        /* The X6100 patched firmware's extras: this base has no register for them */
        case HW_TX_I_OFFSET:
        case HW_TX_Q_OFFSET:
        case HW_FILTER2_LOW:
        case HW_FILTER2_HIGH:
        case HW_IF_SHIFT_ON:
        case HW_IF_SHIFT_HZ:
        case HW_FM_EMP:
        case HW_TX_FILTER_LOW:
        case HW_TX_FILTER_HIGH:
        case HW_CESSB:
        case HW_CW_PEAK:
        case HW_CW_PEAK_Q:
            return false;
        default:
            fprintf(stderr, "hw_x6200: hw_set_i(%d) is not an integer setting\n", p);
            return false;
    }
    return true;
}

bool hw_set_f(hw_param_t p, float v) {
    switch (p) {
        case HW_TXPWR_F:            x6200_control_txpwr_set(v); break;
        case HW_KEY_RATIO_F:        x6200_control_key_ratio_set(v); break;
        /* Not on this base */
        case HW_ADC_DAC_GAIN_F:
        case HW_DAC_GAIN_F:
        case HW_COMP_THRESHOLD_F:
        case HW_COMP_MAKEUP_F:
        case HW_CESSB_POWER_UP_F:
            return false;
        default:
            fprintf(stderr, "hw_x6200: hw_set_f(%d) is not a float setting\n", p);
            return false;
    }
    return true;
}

/* ---- The WR12, Xiegu's 54 MHz-1 GHz receiver on HOST ------------------------------------ */

/* As the stock app does it (XWBReceiverComm::start, MainWidget::updateVfoFrequency):
 * - found on USB by itself (a thread looks each second): taken from gpsd, which opens any ttyACM as a GPS, asked
 *   AT+AYT, and set up as the stock app does (AT+INIT, the LNA at 15, IF and mixer gains at 0, both AGCs on, bypass
 *   on). hw_wr12_state() tells the GUI, which shows WR12 and switches Settings > General > WR12 on HOST with it.
 * - with it there, on and the current VFO at 54 MHz-1 GHz: the WR12 tuned over its port, its bypass off, bit 23 of
 *   register 12 (the stock app's "iqswap") set, and the base tuned to the WR12's output, 5.06 MHz (the stock
 *   app's BaseCommIIC::parseParamItem sends any VFO above 54 MHz as 5060000 while a WR12 is there). Below 54 MHz:
 *   iqswap off, the WR12's bypass on, the base tuned as ever.
 * - while it is there and on, the base's TX outside the ham bands is shut (base_open_tx_set), as the stock app
 *   always has it: the hand mic's PTT keys the base by itself, and with it open the base went out on 5.06 MHz.
 *   Its split is held off while listening through it, and the bypass goes on before the base is on HF again.
 * The port stays open and each command waits for its OK, as the stock app's. The frequency goes from a thread of
 * its own, the latest one only: a knob turned fast held the radio's lock through one write per step, and the
 * spectrum lagged. */
#define WR12_VID        "0483"      /* STMicroelectronics */
#define WR12_PID        "5740"      /* Virtual COM Port */
#define WR12_MIN_HZ     54000000
#define WR12_MAX_HZ     1000000000
#define WR12_IF_HZ      5060000     /* where the WR12 puts what it receives: the base listens there */
#define X6200_IQSWAP    0x800000    /* register 12: the stock app's iqswap, set while listening through the WR12 */

static pthread_mutex_t wr12_io = PTHREAD_MUTEX_INITIALIZER;    /* the port */
static pthread_mutex_t wr12_mx = PTHREAD_MUTEX_INITIALIZER;    /* the frequency waiting: never held over a write */
static pthread_cond_t  wr12_cv = PTHREAD_COND_INITIALIZER;
static char            wr12_path[24];   /* /dev/ttyACMn it was found on */
static int             wr12_fd = -1;
/* Set by wr12_watch, read under the radio's lock */
static volatile int    wr12_state = -1; /* hw_wr12_state() */
static volatile bool   wr12_ready;      /* set up since it was last found (wr12_init) */
static volatile bool   wr12_bypass_owed; /* AT+BYPASS=1 not answered: asked again each second */
static volatile int32_t wr12_hz;        /* the frequency last asked of it, 0 to send it again (wr12_init) */
/* Under the radio's lock (hw_vfo_set_freq, hw_set_i) */
static bool            wr12_active;     /* listening through it: iqswap set, bypass off */
static bool            wr12_in_use;     /* there and on: the base's TX kept to the ham bands */
static bool            wr12_split;      /* split as the GUI asked it, held off in the base while listening through it */
static int32_t         wr12_vfo_hz[2];  /* what each VFO was last tuned to (hw_vfo_set_freq), 0 not yet */
/* Under wr12_mx */
static int32_t         wr12_want;       /* a frequency for wr12_thread to send, 0 none */

typedef enum { WR12_NOT_SENT, WR12_NO_OK, WR12_OK } wr12_res_t;

/* Under wr12_io */
static void wr12_close(void) {
    if (wr12_fd >= 0) {
        close(wr12_fd);
        wr12_fd = -1;
    }
}

/* Under wr12_io: OK (true) or ERROR back within `ms` */
static bool wr12_reply(int ms) {
    char   buf[96];
    size_t n = 0;

    for (int i = 0; i < ms / 10 && n < sizeof(buf) - 1; i++) {
        struct pollfd p = {wr12_fd, POLLIN, 0};
        ssize_t       r;

        if (poll(&p, 1, 10) > 0 && (r = read(wr12_fd, buf + n, sizeof(buf) - 1 - n)) > 0) {
            n += r;
            buf[n] = '\0';
            if (strstr(buf, "OK")) return true;
            if (strstr(buf, "ERROR")) return false;
        }
    }
    return false;
}

/* Under wr12_io. Each command waits for its answer, half a second, three tries, as the stock app's
 * (XWBReceiverComm::writeAtCommand(..., 500, 3)): two sent together, the WR12 dropped one (plugged in again on
 * 96.9 MHz: BYPASS=0 kept, the frequency lost, 144 MHz left); right after AT+INIT it took over 0.3 s */
static wr12_res_t wr12_write(const char *cmd, bool log) {
    char buf[48];
    int  n = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    bool ok = false, answered = false;

    for (int tries = 0; tries < 3 && !answered; tries++) {
        if (wr12_fd < 0 && wr12_path[0]) {
            struct termios t;

            wr12_fd = open(wr12_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (wr12_fd >= 0 && tcgetattr(wr12_fd, &t) == 0) {
                cfmakeraw(&t);
                tcsetattr(wr12_fd, TCSANOW, &t);
            }
        }
        if (wr12_fd >= 0) {
            tcflush(wr12_fd, TCIFLUSH);
        }
        ok = wr12_fd >= 0 && write(wr12_fd, buf, n) == n;
        if (!ok) {
            wr12_close();       /* unplugged: no use trying again */
            break;
        }
        answered = wr12_reply(500);
    }
    if (log || !answered) {
        fprintf(stderr, "hw_x6200: WR12 %s%s\n", cmd, !ok ? " (not sent)" : answered ? "" : " (no OK)");
    }
    return !ok ? WR12_NOT_SENT : answered ? WR12_OK : WR12_NO_OK;
}

/* wr12_write with the port taken */
static wr12_res_t wr12_send(const char *cmd) {
    wr12_res_t res;

    pthread_mutex_lock(&wr12_io);
    res = wr12_write(cmd, true);
    pthread_mutex_unlock(&wr12_io);
    return res;
}

/* Set up as the stock app does; stops only when the port is gone (a setting not answered is sent on) */
static void wr12_init(void) {
    static const char *const cmds[] = {
        "AT+INIT", "AT+LNAGAIN=15", "AT+VGAGAIN=0", "AT+MIXERGAIN=0", "AT+LNAAGCENA=1", "AT+MIXERAGCENA=1"
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        if (wr12_send(cmds[i]) == WR12_NOT_SENT) {
            return;
        }
        usleep(50000);
    }
    wr12_hz = 0;        /* the frequency again on the next tune */
    wr12_ready = true;
}

/* The ttyACM whose USB device is the WR12's, in `path` */
static bool wr12_find(char *path, size_t size) {
    for (int i = 0; i < 4; i++) {
        char  f[64], vid[8] = "", pid[8] = "";
        FILE *fp;

        snprintf(f, sizeof(f), "/sys/class/tty/ttyACM%d/device/../idVendor", i);
        if ((fp = fopen(f, "r"))) {
            if (!fgets(vid, sizeof(vid), fp)) vid[0] = '\0';
            fclose(fp);
        }
        snprintf(f, sizeof(f), "/sys/class/tty/ttyACM%d/device/../idProduct", i);
        if ((fp = fopen(f, "r"))) {
            if (!fgets(pid, sizeof(pid), fp)) pid[0] = '\0';
            fclose(fp);
        }
        if (!strncmp(vid, WR12_VID, 4) && !strncmp(pid, WR12_PID, 4)) {
            snprintf(path, size, "/dev/ttyACM%d", i);
            return true;
        }
    }
    return false;
}

/* gpsd told to leave `path`, or to take it again */
static void wr12_gpsd(const char *how, const char *path) {
    char cmd[64];

    snprintf(cmd, sizeof(cmd), "gpsdctl %s %s > /dev/null 2>&1", how, path);
    if (system(cmd) != 0) {
        /* no gpsd running, or it never had it: nothing to give or take */
    }
}

/* Each second: a WR12 plugged in or out. Another device on the same chip (a GPS on an STM serial port) does not
 * answer AT+AYT: it goes back to gpsd and is not asked again until it is unplugged */
static void *wr12_watch(void *arg) {
    char not_wr12[24] = "";

    (void)arg;
    for (;;) {
        char path[24];
        bool found = wr12_find(path, sizeof(path));

        if (found && wr12_state != 1 && strcmp(path, not_wr12)) {
            bool ok;

            wr12_gpsd("remove", path);
            pthread_mutex_lock(&wr12_io);
            wr12_close();
            snprintf(wr12_path, sizeof(wr12_path), "%s", path);
            ok = wr12_write("AT+AYT", true) == WR12_OK;
            if (!ok) {
                wr12_close();
                wr12_path[0] = '\0';
            }
            pthread_mutex_unlock(&wr12_io);
            if (ok) {
                wr12_init();
                /* bypass on whatever: off again with the next tuning above 54 MHz (wr12_hz 0). Sent by this thread
                 * as listening through it was, a crossing to HF meanwhile left it out of bypass on HF */
                wr12_bypass_owed = wr12_send("AT+BYPASS=1") != WR12_OK;
                fprintf(stderr, "hw_x6200: WR12 found on %s\n", path);
            } else {
                wr12_gpsd("add", path);
                snprintf(not_wr12, sizeof(not_wr12), "%s", path);
                fprintf(stderr, "hw_x6200: %s does not answer as a WR12: left to gpsd\n", path);
            }
            wr12_state = ok;
        } else if (!found) {
            not_wr12[0] = '\0';
        }
        if (!found && wr12_state != 0) {
            pthread_mutex_lock(&wr12_io);
            wr12_close();
            wr12_path[0] = '\0';
            pthread_mutex_unlock(&wr12_io);
            wr12_ready = false;
            if (wr12_state == 1) {
                fprintf(stderr, "hw_x6200: WR12 gone\n");
            }
            wr12_state = 0;
        }
        /* wr12_active read without the radio's lock: at worst a bypass sent just as it went off again, the WR12 deaf
         * (never HF into it) until the next tune */
        if (wr12_bypass_owed && wr12_state == 1 && !wr12_active && wr12_send("AT+BYPASS=1") == WR12_OK) {
            wr12_bypass_owed = false;
        }
        sleep(1);
    }
    return NULL;
}

int hw_wr12_state(void) {
    return wr12_state;
}

/* Sends the frequency waiting, its OK waited for; one asked meanwhile goes next, the ones between it are skipped */
static void *wr12_thread(void *arg) {
    (void)arg;
    for (;;) {
        char    cmd[32];
        int32_t hz;

        pthread_mutex_lock(&wr12_mx);
        while (!wr12_want) {
            pthread_cond_wait(&wr12_cv, &wr12_mx);
        }
        hz = wr12_want;
        wr12_want = 0;
        pthread_mutex_unlock(&wr12_mx);

        snprintf(cmd, sizeof(cmd), "AT+FREQ=%d", (int)hz);
        pthread_mutex_lock(&wr12_io);
        wr12_write(cmd, false);
        pthread_mutex_unlock(&wr12_io);
    }
    return NULL;
}

static void wr12_start(void) {
    pthread_t t;

    if (pthread_create(&t, NULL, wr12_watch, NULL) == 0) {
        pthread_detach(t);
    }
    if (pthread_create(&t, NULL, wr12_thread, NULL) == 0) {
        pthread_detach(t);
    }
}

/* Under the radio's lock: the frequency to wr12_thread, never waited for (the bypass is: wr12_enter, wr12_leave) */
static void wr12_tune(int32_t hz) {
    pthread_mutex_lock(&wr12_mx);
    wr12_want = hz;
    pthread_cond_signal(&wr12_cv);
    pthread_mutex_unlock(&wr12_mx);
    wr12_hz = hz;
}

static void iqswap_set(bool on) {
    uint32_t r = x6200_control_get(x6200_sple_atue_trx);
    uint32_t n = on ? (r | X6200_IQSWAP) : (r & ~(uint32_t)X6200_IQSWAP);

    if (n != r) {
        x6200_control_cmd(x6200_sple_atue_trx, n);
        fprintf(stderr, "hw_x6200: iqswap %s\n", on ? "on" : "off");
    }
}

/* The base's TX outside the ham bands: byte 280 of its calibration block, 1 open. X6200Control opens it at start
 * (x6200_control_init); the stock app leaves it shut, and the base then keys only inside the band table of the same
 * block (160 to 6 m; 60 m 5.332-5.405 MHz). Shut while listening through the WR12: the hand mic's PTT keys the base
 * by itself, without a word to the GUI, and with it open the base went out on the WR12's 5.06 MHz at the 2 W set
 * (4-nov, dummy load on the radio). The block read back from the base and sent whole, as the library does. */
#define BASE_I2C_ADDR       0x72
#define BASE_CAL_READ       0x0100
#define BASE_CAL_WRITE      0xf000
#define BASE_CAL_LEN        507
#define BASE_CAL_OPEN_TX    280

static bool base_open_tx_set(bool on) {
    uint8_t                    block[2 + BASE_CAL_LEN];
    uint8_t                    reg[2] = { BASE_CAL_READ >> 8, BASE_CAL_READ & 0xff };
    struct i2c_msg             rd_msgs[] = {
        { .addr = BASE_I2C_ADDR, .flags = 0, .len = sizeof(reg), .buf = reg },
        { .addr = BASE_I2C_ADDR, .flags = I2C_M_RD, .len = BASE_CAL_LEN, .buf = block + 2 },
    };
    struct i2c_msg             wr_msg = { .addr = BASE_I2C_ADDR, .flags = 0, .len = sizeof(block), .buf = block };
    struct i2c_rdwr_ioctl_data rd = { .msgs = rd_msgs, .nmsgs = 2 };
    struct i2c_rdwr_ioctl_data wr = { .msgs = &wr_msg, .nmsgs = 1 };
    int                        fd = open("/dev/i2c-0", O_RDWR);
    bool                       ok;

    if (fd < 0) {
        fprintf(stderr, "hw_x6200: TX outside the ham bands not %s: /dev/i2c-0: %s\n", on ? "opened" : "shut",
                strerror(errno));
        return false;
    }
    ok = ioctl(fd, I2C_RDWR, &rd) >= 0;
    if (ok && block[2 + BASE_CAL_OPEN_TX] > 1) {     /* as the library: not the layout it knows, left alone */
        fprintf(stderr, "hw_x6200: calibration byte %d is %u, TX outside the ham bands left as it is\n",
                BASE_CAL_OPEN_TX, block[2 + BASE_CAL_OPEN_TX]);
        close(fd);
        return false;
    }
    if (ok) {
        block[0] = BASE_CAL_WRITE >> 8;
        block[1] = BASE_CAL_WRITE & 0xff;
        block[2 + BASE_CAL_OPEN_TX] = on;
        ok = ioctl(fd, I2C_RDWR, &wr) >= 0;
    }
    close(fd);
    fprintf(stderr, "hw_x6200: TX outside the ham bands %s%s\n", on ? "open" : "shut", ok ? "" : ": failed");
    return ok;
}

/* The base's VFO register for `vfo` on `hz`: false when it was there already */
static bool base_freq_set(radio_vfo_t vfo, int32_t hz) {
    if (x6200_control_get(vfo == RADIO_VFO_A ? x6200_vfoa_freq : x6200_vfob_freq) == (uint32_t)hz) {
        return false;
    }
    x6200_control_vfo_freq_set((x6200_vfo_t)vfo, hz);
    return true;
}

/* Bypass off: the base already on the WR12's output with its TX shut (in use), never on HF while the WR12 takes
 * the antenna. The base's split held off meanwhile: with the TX VFO on HF, the hand mic's PTT went out there, into
 * the WR12 */
static void wr12_enter(void) {
    if (wr12_split) {
        x6200_control_split_set(false);
    }
    iqswap_set(true);
    wr12_send("AT+BYPASS=0");
    wr12_active = true;
}

/* Bypass on before the base is back on HF. Not answered with the WR12 there: asked again each second (wr12_watch),
 * as a WR12 out of bypass on HF would take the transmission */
static void wr12_leave(void) {
    wr12_bypass_owed = wr12_send("AT+BYPASS=1") != WR12_OK && wr12_state == 1;
    iqswap_set(false);
    wr12_active = false;
    if (wr12_split) {
        x6200_control_split_set(true);
    }
}

/* In use: there and on (Settings > General > WR12 on HOST). Then the base's TX stays in the ham bands, as the
 * stock app always has it: every VFO above 54 MHz sits on 5.06 MHz in the base, and the hand mic's PTT keys the
 * base by itself on whichever VFO transmits. Out of use: through the WR12 no more, each VFO left on its output back
 * on its own frequency, and only then TX outside the ham bands open again */
static void wr12_use_check(void) {
    bool use = wr12_state == 1 && param_i_get(cfg_wr12);

    if (use == wr12_in_use) {
        return;
    }
    wr12_in_use = use;
    if (use) {
        base_open_tx_set(false);
        return;
    }
    if (wr12_active) {
        wr12_leave();
    }
    for (int v = RADIO_VFO_A; v <= RADIO_VFO_B; v++) {
        int32_t hz = wr12_vfo_hz[v == RADIO_VFO_B];

        if (hz && x6200_control_get(v == RADIO_VFO_A ? x6200_vfoa_freq : x6200_vfob_freq) == WR12_IF_HZ) {
            base_freq_set((radio_vfo_t)v, hz);
        }
    }
    base_open_tx_set(true);
}

/* The GUI's split, held off in the base while listening through the WR12 (wr12_enter) */
static void wr12_split_set(bool on) {
    wr12_split = on;
    x6200_control_split_set(on && !wr12_active);
}

/* A/B: the VFO now current tuned again, through the WR12 or not. Only the current VFO drives it, and nothing else
 * looked again: A on 2 m and B on 40 m, B kept the WR12 out of bypass and iqswap on (HF through its VHF path, a
 * transmission into it), and back on A the base stayed on 145.5 MHz itself, deaf (4-nov) */
static void wr12_vfo_switched(radio_vfo_t vfo) {
    int32_t hz = wr12_vfo_hz[vfo == RADIO_VFO_B];

    if (hz) {
        hw_vfo_set_freq(vfo, hz);
    }
}

/* Above 54 MHz with the WR12 in use, the base is on the WR12's output (the stock app's BaseCommIIC::parseParamItem
 * sends any VFO there as 5060000), and the current VFO drives the WR12. The order keeps HF power off the WR12: into
 * it, the base leaves HF before the bypass goes off; out of it, the bypass is on before the base is back on HF */
bool hw_vfo_set_freq(radio_vfo_t vfo, int32_t hz) {
    /* on A/B the setting is already the new VFO: HW_VFO_CUR comes from its change (radio.c) */
    bool cur = vfo == (radio_vfo_t)param_i_get(cfg_band_current_vfo);

    wr12_vfo_hz[vfo == RADIO_VFO_B] = hz;
    wr12_use_check();
    if (!wr12_in_use || hz < WR12_MIN_HZ || hz > WR12_MAX_HZ) {
        if (cur && wr12_active) {
            wr12_leave();
        }
        return base_freq_set(vfo, hz);
    }
    base_freq_set(vfo, WR12_IF_HZ);
    if (cur) {
        if (!wr12_ready) {
            wr12_init();
        }
        if (!wr12_active || !wr12_hz) {     /* found again: its bypass back off */
            wr12_enter();
        }
        if (hz != wr12_hz) {
            wr12_tune(hz);
        }
    }
    return true;
}

bool hw_vfo_set_mode(radio_vfo_t vfo, radio_mode_t mode) {
    x6200_mode_t m = lib_mode(mode);

    if (x6200_control_get(vfo == RADIO_VFO_A ? x6200_vfoa_mode : x6200_vfob_mode) == (uint32_t)m) {
        return false;
    }
    x6200_control_vfo_mode_set((x6200_vfo_t)vfo, m);
    return true;
}

void hw_vfo_set_agc(radio_vfo_t vfo, radio_agc_t agc) {
    x6200_control_vfo_agc_set((x6200_vfo_t)vfo, (x6200_agc_t)agc);
}

void hw_vfo_set_att(radio_vfo_t vfo, bool on) {
    x6200_control_vfo_att_set((x6200_vfo_t)vfo, on ? x6200_att_on : x6200_att_off);
}

void hw_vfo_set_pre(radio_vfo_t vfo, bool on) {
    x6200_control_vfo_pre_set((x6200_vfo_t)vfo, on ? x6200_pre_on : x6200_pre_off);
}

void hw_poweroff(void) {
    x6200_control_poweroff();
}

/* The library reopens the bus and tries once more on its own, and reports nothing but errno */
bool hw_resend_all(void) {
    errno = 0;
    x6200_control_idle();
    return errno == 0;
}

bool hw_tx_keyed_by_gui(void) {
    return (x6200_control_get(x6200_sple_atue_trx) & TX_BITS) != 0;
}

void hw_pin_set(hw_pin_t pin, bool on) {
    switch (pin) {
        case HW_PIN_WIFI:       x6200_gpio_set(x6200_pin_wifi, on ? 1 : 0); break;
        case HW_PIN_LIGHT:      x6200_gpio_set(x6200_pin_light, on ? 1 : 0); break;
        case HW_PIN_USB_CAT:    x6200_gpio_set(x6200_pin_usb, on ? 1 : 0); break;
        case HW_PIN_MORSE_KEY:  x6200_gpio_set(x6200_pin_morse_key, on ? 0 : 1); break;  /* active low */
        case HW_PIN_BB_RESET:   x6200_gpio_set(x6200_pin_bb_reset, on ? 1 : 0); break;
    }
}

/* ---- The packet stream -------------------------------------------------------------------- */

/* 552 bytes every ~35 ms: 512 spectrum bins as unsigned dB (126 = 0 dBm), the flags, the
 * meters (dbm is the S-meter, negated), the hand mic's key */
bool hw_flow_read(hw_pack_t *p) {
    if (!x6200_flow_read(pack)) {
        return false;
    }
    for (size_t i = 0; i < PSD_BINS; i++) {
        psd_db[i] = (float)pack->samples[i] - PSD_OFFSET;
    }
    p->samples    = NULL;
    p->n_samples  = 0;
    p->psd        = psd_db;
    p->n_psd      = PSD_BINS;
    p->psd_span   = PSD_SPAN_HZ >> fft_dec;
    p->smeter_dbm = -(int16_t)pack->dbm + SMETER_CORR;
    p->lo_freq    = 0;
    p->fft_dec    = 0;
    p->vary_freq  = false;
    p->tx         = pack->flag.tx;
    p->atu_status = pack->flag.atu_status;
    p->charging   = pack->flag.charging;
    p->vext_flag  = pack->flag.vext;
    p->bat_hot    = pack->flag.battery_high_temp;
    p->bat_cold   = pack->flag.battery_low_temp;
    p->pa_hot     = pack->flag.pa_high_temp;
    p->dcin_low   = pack->flag.dc_in_too_low;
    p->power_key  = pack->flag.power_key;
    p->bat_missing = pack->flag.battery_not_found;
    p->tx_power   = pack->tx_power;
    p->vswr       = pack->vswr;
    p->alc_level  = pack->alc_level;
    p->vext       = pack->vext;
    p->vbat       = pack->vbat;
    p->batcap     = pack->batcap;
    p->atu_params = pack->atu_params;
    p->hkey       = pack->hkey;
    return true;
}

void hw_flow_restart(void) {
    x6200_flow_restart();
}
