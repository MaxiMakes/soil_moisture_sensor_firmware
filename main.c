/*
 * Soil moisture sensor node - ATtiny402 firmware
 * ------------------------------------------------
 *
 * Up to 254 nodes share a two-wire TTL bus with one controller:
 *
 *   controller TX ---> RX of every node          (PA2)
 *   controller RX <--- TX of every node          (PA1, open drain, 5K Pull up)
 *
 * The UART runs in Multi-Processor Communication Mode (MPCM) with 9-bit frames:
 * a frame whose 9th bit is 1 is an address frame, all other frames are data frames.
 * A sleeping node ignores every data frame in hardware and only wakes up when an
 * address frame carrying its own address (or the broadcast address 0x00) arrives.
 *
 * Request  (controller -> node):  [addr*] [cmd] [len] [payload...] [crc8]
 * Response (node -> controller):  [addr]  [status] [len] [payload...] [crc8]
 *   * = 9th bit set. Everything else has the 9th bit cleared.
 *   crc8, polynomial 0x07, init 0x00, over all preceding bytes of the frame.
 *   Multi-byte values are big endian. Broadcast requests are never answered.
 *
 * Commands (see README.md for the full description):
 *   0x01 SET_ADDR    [new_addr]        -> []                       stored in EEPROM
 *   0x02 QUERY       [count]           -> [n][val_hi val_lo]*n [age_min_hi age_min_lo]
 *   0x03 SET_RATE    [min_hi min_lo]   -> []                       0 = periodic off, EEPROM
 *   0x04 MEASURE     []                -> [val_hi val_lo]
 *   0x05 SET_FREQ    [khz_hi khz_lo]   -> [actual_khz_hi actual_khz_lo]  EEPROM
 *   0x06 GET_CONFIG  []                -> [fw][addr][min_hi min_lo][khz_hi khz_lo][act_hi act_lo]
 *
 * Pinout (SOIC-8):
 *   PA0  UPDI
 *   PA1  TX   (USART0 alternate TxD, open drain)
 *   PA2  RX   (USART0 alternate RxD)
 *   PA3  Not connected
 *   PA6  SENSOR-IN  square wave drive for the capacitive probe (TCB0 WO0)
 *   PA7  SENSOR-OUT rectified/filtered probe voltage (ADC AIN7)
 *
 * Bus: 1200 baud (long cable, many nodes).
 *
 * Power: the node sits in Standby sleep (RTC + USART start-of-frame detector running).
 * It wakes once per minute to count down the measurement interval and whenever an
 * address frame is received.
 *
 * Fuses: OSCCFG = 20 MHz (default). Set EESAVE in SYSCFG0 so the address survives reflashing.
 *
 */

/*
* you might need to download the support pack for avr-gcc from http://packs.download.atmel.com/
* add include/avr/iotn402.h to /usr/avr/include/avr
* add gcc/dev/attiny402/device-specs/specs-attiny402 to /usr/lib/gcc/avr/<VERSION>/device-specs/
* add gcc/dev/attiny402/avrxmega3/short-calls/ to /usr/lib/avr/lib/
*/
//avrdude -c jtag2updi -p t402 -P /dev/ttyUSB0 -Uflash:w:soil-moisture-code.hex

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 * Build-time configuration
 * ------------------------------------------------------------------------- */
#define FW_VERSION              2

#define BUS_BAUD                1200UL          // low: long wires, many nodes 
#define F_OSC                   20000000UL      // F_OSC = oscillator frequency; OSCCFG fuse: 20 MHz 

/* Idle CLK_PER: OSC20M / 6 (reset default). */
#define IDLE_MCLKCTRLB          (CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm)   /* MCLKCTRLB = main clock control B */
#if !defined(F_CPU) || (F_CPU) != (F_OSC / 6)
#error "F_CPU must be 3333333 (20 MHz / 6)"
#endif

/* CLK_PER while measuring. The probe drive frequency is derived from it.
 * 10 MHz is the maximum for VDD = 3.3 V  
 */
#define MEAS_MCLKCTRLB          (CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm)   /* MEAS = measurement; 10 MHz */
#define F_CLK_FAST              (F_OSC / 2)     /* CLK = clock, while measuring */

#define VDD_IS_5V               0               /* 0: 3.3 V board, picks the factory oscillator error row */
/* 0: wake from Standby when a complete address frame for us has arrived (RXC).
 * 1: additionally wake on every start bit (RXS). Try this if nodes miss the
 *    first request after sleeping; costs a short wake-up per byte on the bus. */
#define WAKE_ON_START_BIT       0
#define ADC_USE_VDD_REF         1               /* 1: ratiometric to VDD, 0: internal 4.34 V */
#define ADC_SAMPLEN             10              /* SAMPLEN = sample length: extra sample cycles for the high-Z probe output */
#define SENSOR_SETTLE_MS        50              /* PWM on -> ADC sample delay; the RC output has
                                                   tau ~5-9 ms, 20 ms was not settled (measured) */

#define HIST_LEN                8               /* HIST_LEN = history length: stored measurements (power of two) */
#define DEFAULT_INTERVAL_MIN    20              /* MIN = minutes between measurements */
#define DEFAULT_FREQ_KHZ        1000            /* FREQ = probe drive frequency in kHz; chosen by the sponge sweep, see docs/freq_experiment.md */

#define ADDR_BROADCAST          0x00            /* ADDR = address */
#define ADDR_UNCONFIGURED       0xFF
#define BYTE_TIMEOUT_MS         50              /* max gap between bytes of one request (~6 byte times) */
#define TURNAROUND_MS           5               /* silence before a node answers */

#define MAX_REQ_PAYLOAD         4               /* REQ = request */
#define MAX_RESP_PAYLOAD        (1 + 2 * HIST_LEN + 2)   /* RESP = response */

/* Protocol: CMD = command codes */
enum {
    CMD_SET_ADDR   = 0x01,
    CMD_QUERY      = 0x02,
    CMD_SET_RATE   = 0x03,
    CMD_MEASURE    = 0x04,
    CMD_SET_FREQ   = 0x05,
    CMD_GET_CONFIG = 0x06,
};

/* ST = status codes in the response */
enum {
    ST_OK        = 0x00,
    ST_BAD_CMD   = 0x01,
    ST_BAD_LEN   = 0x02,
    ST_BAD_PARAM = 0x03,
};

/* EEPROM layout (all in page 0); EE = EEPROM */
#define EE_MAGIC_VAL    0xA5
#define EE_MAGIC        0
#define EE_ADDR         1
#define EE_INTERVAL_H   2
#define EE_INTERVAL_L   3
#define EE_FREQ_H       4
#define EE_FREQ_L       5
#define EE_SIZE_USED    6

/* ---------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */
static volatile uint8_t my_addr = ADDR_UNCONFIGURED;      /* addr = bus address of this node */
static uint16_t cfg_interval_min = DEFAULT_INTERVAL_MIN;  /* cfg = configuration, min = minutes */
static uint16_t cfg_freq_khz     = DEFAULT_FREQ_KHZ;      /* freq = probe drive frequency, kHz */

static uint16_t baud_slow, baud_fast;   /* USART BAUD register values for idle / measuring clock */

static uint8_t  pwm_top;            /* pwm = pulse-width modulation; TCB period - 1 */
static uint8_t  pwm_div2;           /* div2 = divide by two: 1 = TCB clocked with CLK_PER/2 */
static uint16_t pwm_actual_khz;     /* frequency really produced, kHz */

static uint16_t hist[HIST_LEN];     /* hist = history: ring buffer of the last measurements */
static uint8_t  hist_head;          /* history head = next write position */
static uint8_t  hist_count;         /* history count = filled entries (max HIST_LEN) */
static uint16_t age_min;            /* age in minutes since the newest measurement (saturating) */
static uint16_t minutes_elapsed;    /* minutes since last periodic measurement */

static volatile uint8_t rtc_minutes;    /* rtc = real-time counter; set by its ISR, consumed by main */

/* RX queue filled by the USART ISR: data byte + RXDATAH flags */
#define RXQ_LEN 8                       /* RXQ = receive queue */
static volatile uint8_t rxq_d[RXQ_LEN];     /* d = data byte */
static volatile uint8_t rxq_f[RXQ_LEN];     /* f = flags (RXDATAH: 9th bit, errors) */
static volatile uint8_t rxq_head, rxq_tail; /* ring buffer write / read positions */

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */
/* crc8 = 8-bit cyclic redundancy check, polynomial 0x07; d = data byte */
static uint8_t crc8(uint8_t crc, uint8_t d)
{
    crc ^= d;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

// BAUD register for the given CLK_PER, corrected with the factory oscillator error. 
// baud_reg = USART BAUD register value; clk_per = peripheral clock in Hz 
static uint16_t baud_reg(uint32_t clk_per)
{
    int8_t  err = VDD_IS_5V ? SIGROW.OSC20ERR5V : SIGROW.OSC20ERR3V;
    int32_t b   = (int32_t)((clk_per * 64UL + 8UL * BUS_BAUD) / (16UL * BUS_BAUD));
    b = (b * (1024L + err)) / 1024L;
    return (uint16_t)b;
}

// clk_set = set the main clock prescaler and the matching baud rate 
static void clk_set(uint8_t mclkctrlb, uint16_t baud)
{
    // The CCP unlock window is 4 instructions; keep interrupts out of it. 
    cli();
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, mclkctrlb);
    sei();
    USART0.BAUD = baud;
}

/* ---------------------------------------------------------------------------
 * EEPROM (memory mapped, written through the NVM controller page buffer)
 * ------------------------------------------------------------------------- */
// read one EEPROM byte at idx (index)
static uint8_t ee_read(uint8_t idx)
{
    return *(volatile uint8_t *)(EEPROM_START + idx);
}

/* cfg_save = save the configuration to EEPROM (only bytes that changed) */
static void cfg_save(void)
{
    uint8_t img[EE_SIZE_USED] = {
        EE_MAGIC_VAL, my_addr,
        (uint8_t)(cfg_interval_min >> 8), (uint8_t)cfg_interval_min,
        (uint8_t)(cfg_freq_khz >> 8),     (uint8_t)cfg_freq_khz,
    };
    bool dirty = false;

    while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) ;
    for (uint8_t i = 0; i < EE_SIZE_USED; i++) {
        if (ee_read(i) != img[i]) {
            *(volatile uint8_t *)(EEPROM_START + i) = img[i];    /* load page buffer */
            dirty = true;
        }
    }
    if (!dirty)
        return;
    cli();
    _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_PAGEERASEWRITE_gc);
    sei();
    while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) ;
}

// load the configuration from EEPROM, defaults if unprogrammed 
static void cfg_load(void)
{
    if (ee_read(EE_MAGIC) == EE_MAGIC_VAL) {
        my_addr          = ee_read(EE_ADDR);
        cfg_interval_min = ((uint16_t)ee_read(EE_INTERVAL_H) << 8) | ee_read(EE_INTERVAL_L);
        cfg_freq_khz     = ((uint16_t)ee_read(EE_FREQ_H) << 8) | ee_read(EE_FREQ_L);
    }
    if (my_addr == ADDR_BROADCAST)
        my_addr = ADDR_UNCONFIGURED;
}

/* ---------------------------------------------------------------------------
 * Probe drive (TCB0 8-bit PWM on PA6) and ADC
 * ------------------------------------------------------------------------- */
// pwm_configure = derive TCB period/prescaler for a probe frequency in kHz 
static void pwm_configure(uint16_t khz)
{
    const uint32_t base = F_CLK_FAST / 1000UL;      // kHz
    uint32_t n;

    if (khz == 0)
        khz = 1;
    /* Period in TCB ticks, rounded to the nearest EVEN number so the duty cycle is
     * exactly 50 % (an odd period gives 33..45 %, which shifts the rectified level). */
    pwm_div2 = 0;
    n = (base + khz) / (2 * khz) * 2;
    if (n > 256) {
        pwm_div2 = 1;
        n = (base / 2 + khz) / (2 * khz) * 2;
        if (n > 256)
            n = 256;
    }
    if (n < 2)
        n = 2;
    pwm_top        = (uint8_t)(n - 1);
    pwm_actual_khz = (uint16_t)((pwm_div2 ? base / 2 : base) / n);
}

/* start the probe drive square wave on PA6 */
static void pwm_on(void)
{
    uint16_t duty = ((uint16_t)pwm_top + 1) / 2;
    TCB0.CTRLA = 0;
    TCB0.CNT   = 0;
    /* Errata: CCMP must be accessed as one 16-bit register in 8-bit PWM mode.
     * CCMPL = period (TOP), CCMPH = compare (duty). */
    TCB0.CCMP  = (duty << 8) | pwm_top;
    TCB0.CTRLB = TCB_CNTMODE_PWM8_gc | TCB_CCMPEN_bm;
    TCB0.CTRLA = (pwm_div2 ? TCB_CLKSEL_CLKDIV2_gc : TCB_CLKSEL_CLKDIV1_gc) | TCB_ENABLE_bm;
}

// stop the probe frequency drive, PA6 low 
static void pwm_off(void)
{
    TCB0.CTRLA = 0;
    TCB0.CTRLB = 0;
    PORTA.OUTCLR = PIN6_bm;
}

// adc_read conversion of SENSOR-OUT, sum of 16 samples
static uint16_t adc_read(void)
{
    // ADC clock: F_CLK_FAST / 32 = 312 kHz at 10 MHz (must stay within 50 kHz..1.5 MHz)
#if ADC_USE_VDD_REF
    ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV32_gc;
#else
    VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_4V34_gc;
    ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_INTREF_gc | ADC_PRESC_DIV32_gc;
#endif
    ADC0.CTRLB    = ADC_SAMPNUM_ACC16_gc;           /* sum of 16 samples -> 0..16368 */
    ADC0.CTRLD    = ADC_INITDLY_DLY16_gc;
    ADC0.SAMPCTRL = ADC_SAMPLEN;
    ADC0.MUXPOS   = ADC_MUXPOS_AIN7_gc;
    ADC0.CTRLA    = ADC_ENABLE_bm;                  /* 10-bit resolution */
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    ADC0.COMMAND  = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) ;
    uint16_t res = ADC0.RES;
    ADC0.CTRLA = 0;
    return res;
}

// append a value (v) to the measurement history
static void hist_push(uint16_t v)
{
    hist[hist_head] = v;
    hist_head = (hist_head + 1) & (HIST_LEN - 1);
    if (hist_count < HIST_LEN)
        hist_count++;
    age_min = 0;
}

// one complete measurement: fast clock, probefrequency on, settle, ADC, probe frequency off */
static uint16_t measure(void)
{
    uint16_t v;

    clk_set(MEAS_MCLKCTRLB, baud_fast);
    pwm_on();
    _delay_ms((double)SENSOR_SETTLE_MS * ((double)F_CLK_FAST / (double)(F_CPU)));
    v = adc_read();
    pwm_off();
    clk_set(IDLE_MCLKCTRLB, baud_slow);

    hist_push(v);
    return v;
}

/* ---------------------------------------------------------------------------
 * RTC: one interrupt per minute, runs in Standby from the 32 kHz ULP oscillator
 * ------------------------------------------------------------------------- */
ISR(RTC_CNT_vect)
{
    RTC.INTFLAGS = RTC_OVF_bm;
    rtc_minutes++;
}

/* initialise the real-time counter for one interrupt per minute */
static void rtc_init(void)
{
    while (RTC.STATUS) ;
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
    RTC.PER = 59;                               /* 60 ticks of 1 s */
    while (RTC.STATUS & RTC_PERBUSY_bm) ;
    RTC.CNT = 0;
    while (RTC.STATUS & RTC_CNTBUSY_bm) ;
    RTC.INTFLAGS = RTC_OVF_bm;
    RTC.INTCTRL  = RTC_OVF_bm;
    RTC.CTRLA = RTC_PRESCALER_DIV32768_gc | RTC_RUNSTDBY_bm | RTC_RTCEN_bm;
}

/* ---------------------------------------------------------------------------
 * USART / bus
 * ------------------------------------------------------------------------- */
// RXC = receive complete (RXS = receive start) 
ISR(USART0_RXC_vect)       //shared by RXC and RXS 
{
#if WAKE_ON_START_BIT
    if (USART0.STATUS & USART_RXSIF_bm) {
        USART0.STATUS = USART_RXSIF_bm;
        if (!(USART0.STATUS & USART_RXCIF_bm))
            return;    // only a start bit so far
    }
#endif
    uint8_t f = USART0.RXDATAH;  // flags + 9th bit, read before RXDATAL 
    uint8_t d = USART0.RXDATAL;

    if (USART0.CTRLB & USART_MPCM_bm) {
        // Data receiver closed: only address frames reach; Open it for this address
        if (!(f & USART_DATA8_bm) || (f & USART_FERR_bm))
            return;
        if (d != my_addr && d != ADDR_BROADCAST)
            return;
        USART0.CTRLB &= (uint8_t)~USART_MPCM_bm;
    }
    uint8_t next = (rxq_head + 1) & (RXQ_LEN - 1);
    if (next == rxq_tail)
        return;              // queue full, drop frame
    rxq_d[rxq_head] = d;
    rxq_f[rxq_head] = f;
    rxq_head = next;
}

// initialise the serial port as an open drain UART
static void usart_init(void)
{
    PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc;    // TxD = PA1, RxD = PA2
    // TX DIR=0 -> open-drain mode USART drives the pin low itself 
    // !!!! the pin must not be an output (tinyAVR 0-series errata)
    PORTA.DIRCLR = PIN1_bm | PIN2_bm;
    //PORTA.PIN1CTRL = PORT_PULLUPEN_bm;              /* weak, the bus needs a real pull-up */
    //PORTA.PIN2CTRL = PORT_PULLUPEN_bm;

    baud_slow = baud_reg(F_CPU);
    baud_fast = baud_reg(F_CLK_FAST);
    USART0.BAUD  = baud_slow;
    USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc
                 | USART_SBMODE_1BIT_gc | USART_CHSIZE_9BITH_gc;
#if WAKE_ON_START_BIT
    USART0.CTRLA = USART_RXCIE_bm | USART_RXSIE_bm;
#else
    USART0.CTRLA = USART_RXCIE_bm;
#endif
    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_ODME_bm
                 | USART_SFDEN_bm | USART_MPCM_bm | USART_RXMODE_NORMAL_gc;
}

// re-enable the address filter and clear the receive queue 
static void bus_rearm(void)
{
    cli();
    rxq_head = rxq_tail = 0;
    USART0.CTRLB |= USART_MPCM_bm;
    sei();
}

// take one received byte (d = data, f = flags) from the queue, with timeout 
static bool rx_pop(uint8_t *d, uint8_t *f, uint16_t timeout_ms)
{
    uint16_t t = timeout_ms * 10;
    for (;;) {
        if (rxq_tail != rxq_head) {
            *d = rxq_d[rxq_tail];
            *f = rxq_f[rxq_tail];
            rxq_tail = (rxq_tail + 1) & (RXQ_LEN - 1);
            return true;
        }
        if (t == 0)
            return false;
        t--;
        _delay_us(100);
    }
}

/* Receive one data byte of the current request. */
static bool rx_data(uint8_t *d)
{
    uint8_t f;
    if (!rx_pop(d, &f, BYTE_TIMEOUT_MS))
        return false;
    return !(f & (USART_FERR_bm | USART_DATA8_bm));
}

// transmit one byte (b) as a data frame
static void tx_byte(uint8_t b)
{
    while (!(USART0.STATUS & USART_DREIF_bm)) ;
    USART0.TXDATAH = 0;                         /* 9th bit clear: data frame */
    USART0.TXDATAL = b;
}

// transmit the answer frame; p = payload
static void tx_response(uint8_t addr, uint8_t status, const uint8_t *p, uint8_t len)
{
    uint8_t crc = 0;

    _delay_ms(TURNAROUND_MS);
    USART0.STATUS = USART_TXCIF_bm;
    tx_byte(addr);   crc = crc8(crc, addr);
    tx_byte(status); crc = crc8(crc, status);
    tx_byte(len);    crc = crc8(crc, len);
    for (uint8_t i = 0; i < len; i++) {
        tx_byte(p[i]);
        crc = crc8(crc, p[i]);
    }
    tx_byte(crc);
    while (!(USART0.STATUS & USART_TXCIF_bm)) ;
}

/* ---------------------------------------------------------------------------
 * Command dispatch
 * ------------------------------------------------------------------------- */
// run one command (cmd); in/in_len = request payload, out/out_len = response payload
static uint8_t execute(uint8_t cmd, const uint8_t *in, uint8_t in_len,
                       uint8_t *out, uint8_t *out_len)
{
    *out_len = 0;

    switch (cmd) {
    case CMD_SET_ADDR: {
        if (in_len != 1)
            return ST_BAD_LEN;
        uint8_t a = in[0];
        if (a == ADDR_BROADCAST || a == ADDR_UNCONFIGURED)
            return ST_BAD_PARAM;
        cli();
        my_addr = a;
        sei();
        cfg_save();
        out[0] = a;
        *out_len = 1;
        return ST_OK;
    }

    case CMD_QUERY: {
        uint8_t n = 1;
        if (in_len > 1)
            return ST_BAD_LEN;
        if (in_len == 1 && in[0] != 0)
            n = in[0];
        if (n > hist_count)
            n = hist_count;
        out[0] = n;
        uint8_t o = 1;
        uint8_t idx = hist_head;
        for (uint8_t i = 0; i < n; i++) {           // newest first
            idx = (idx - 1) & (HIST_LEN - 1);
            out[o++] = (uint8_t)(hist[idx] >> 8);
            out[o++] = (uint8_t)hist[idx];
        }
        out[o++] = (uint8_t)(age_min >> 8);
        out[o++] = (uint8_t)age_min;
        *out_len = o;
        return ST_OK;
    }

    case CMD_SET_RATE:
        if (in_len != 2)
            return ST_BAD_LEN;
        cfg_interval_min = ((uint16_t)in[0] << 8) | in[1];
        minutes_elapsed = 0;
        cfg_save();
        return ST_OK;

    case CMD_MEASURE: {
        if (in_len != 0)
            return ST_BAD_LEN;
        uint16_t v = measure();
        minutes_elapsed = 0;
        out[0] = (uint8_t)(v >> 8);
        out[1] = (uint8_t)v;
        *out_len = 2;
        return ST_OK;
    }

    case CMD_SET_FREQ: {
        if (in_len != 2)
            return ST_BAD_LEN;
        uint16_t khz = ((uint16_t)in[0] << 8) | in[1];
        if (khz == 0)
            return ST_BAD_PARAM;
        cfg_freq_khz = khz;
        pwm_configure(khz);
        cfg_save();
        out[0] = (uint8_t)(pwm_actual_khz >> 8);
        out[1] = (uint8_t)pwm_actual_khz;
        *out_len = 2;
        return ST_OK;
    }

    case CMD_GET_CONFIG:
        if (in_len != 0)
            return ST_BAD_LEN;
        out[0] = FW_VERSION;
        out[1] = my_addr;
        out[2] = (uint8_t)(cfg_interval_min >> 8);
        out[3] = (uint8_t)cfg_interval_min;
        out[4] = (uint8_t)(cfg_freq_khz >> 8);
        out[5] = (uint8_t)cfg_freq_khz;
        out[6] = (uint8_t)(pwm_actual_khz >> 8);
        out[7] = (uint8_t)pwm_actual_khz;
        *out_len = 8;
        return ST_OK;

    default:
        return ST_BAD_CMD;
    }
}

/* Called when the ISR has opened the receiver for data bytes, i.e. an address frame for this device is queued. 
 * receive, check and answer one request from the queue */
static void handle_request(void)
{
    uint8_t addr, f, cmd, len, crc, rx_crc = 0;
    uint8_t payload[MAX_REQ_PAYLOAD];
    uint8_t resp[MAX_RESP_PAYLOAD];
    uint8_t resp_len, status;
    bool ok;

    if (!rx_pop(&addr, &f, 0) || !(f & USART_DATA8_bm)) {
        bus_rearm();    // stale data byte, should not happen 
        return;
    }

    ok = rx_data(&cmd) && rx_data(&len) && len <= MAX_REQ_PAYLOAD;
    for (uint8_t i = 0; ok && i < len; i++)
        ok = rx_data(&payload[i]);
    ok = ok && rx_data(&crc);
    bus_rearm();
    if (!ok)
        return;

    rx_crc = crc8(rx_crc, addr);
    rx_crc = crc8(rx_crc, cmd);
    rx_crc = crc8(rx_crc, len);
    for (uint8_t i = 0; i < len; i++)
        rx_crc = crc8(rx_crc, payload[i]);
    if (rx_crc != crc)
        return;                 // corrupted: stay silent

    status = execute(cmd, payload, len, resp, &resp_len);

    if (addr != ADDR_BROADCAST)
        tx_response(addr, status, resp, resp_len);
}

/* ---------------------------------------------------------------------------
 * Init / main loop
 * ------------------------------------------------------------------------- */
// initialise clock, pins and sleep mode
static void io_init(void)
{
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, IDLE_MCLKCTRLB);

    PORTA.DIRSET = PIN6_bm;                     // SENSOR-IN, idle low 
    PORTA.OUTCLR = PIN6_bm;
    PORTMUX.CTRLD &= (uint8_t)~PORTMUX_TCB0_bm; // TCB0 WO0 on PA6 

    PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc; // analog input 
    PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc; // unused 

    SLPCTRL.CTRLA = SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm;
}

int main(void)
{
    io_init();
    cfg_load();
    pwm_configure(cfg_freq_khz);
    usart_init();
    rtc_init();
    sei();

    measure();    // always have one value to report

    for (;;) {
        uint8_t ticks;

        cli();
        ticks = rtc_minutes;
        rtc_minutes = 0;
        sei();

        if (ticks) {
            age_min = (age_min > 0xFFFFu - ticks) ? 0xFFFFu : age_min + ticks;
            if (cfg_interval_min) {
                minutes_elapsed += ticks;
                if (minutes_elapsed >= cfg_interval_min) {
                    measure();
                    minutes_elapsed = 0;
                }
            }
        }

        if (rxq_tail != rxq_head) {
            handle_request();
            continue;
        }

        // Sleep unless something arrived in the meantime. sei() followed by
        cli();
        if (rtc_minutes == 0 && rxq_tail == rxq_head) {
            sei();
            __asm__ __volatile__("sleep");
        } else {
            sei();
        }
    }
}
