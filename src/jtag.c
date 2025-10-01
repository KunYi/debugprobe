
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/gpio.h>

#include "probe_config.h"
#include "probe.h"
#include "jtag.h"

typedef struct {
    PIO pio;
    uint sm_tms;      // SM0
    uint sm_data;     // SM1
    uint pin_tck;
    uint pin_tms;
    uint pin_tdi;
    uint pin_tdo;
    uint pin_trst;
    uint offset_tms;
    uint offset_data;
} jtag_pio_t;

static jtag_pio_t jtag = {
    .pio = pio0,
    .sm_tms = 0,
    .sm_data = 1,
    .pin_tck = PROBE_PIN_TCK,
    .pin_tms = PROBE_PIN_TMS,
    .pin_tdi = PROBE_PIN_TDI,
    .pin_tdo = PROBE_PIN_TDO,
    .pin_trst = PROBE_PIN_nTRST,
};

void jtag_pio_init(float freq_hz) {
    jtag.offset_tms = pio_add_program(jtag.pio, &jtag_tms_program);
    jtag.offset_data = pio_add_program(jtag.pio, &jtag_tdio_program);

    jtag_program_init(jtag.pio,
	jtag.sm_tms, jtag.offset_tms,
	jtag.sm_data, jtag.offset_data,
	jtag.pin_tck, jtag.pin_tms, jtag.pin_tdi, jtag.pin_tdo);

    jtag_set_clk_freq(freq_hz);
}

void jtag_set_clk_freq(uint freq_khz) {
    uint clk_sys_freq_khz = clock_get_hz(clk_sys) / 1000;
    probe_info("Set jtag freq %dKHz sysclk %dkHz\n", freq_khz, clk_sys_freq_khz);
    uint32_t divider = (((clk_sys_freq_khz + freq_khz - 1)/ freq_khz) + 3) / 4;

    if (divider == 0)
        divider = 1;
    if (divider > 65535)
        divider = 65535;

    pio_sm_set_clkdiv_int_frac(jtag.pio,  jtag.sm_tms, divider, 0);
    pio_sm_set_clkdiv_int_frac(jtag.pio,  jtag.sm_data, divider, 0);
}

void jtag_shift_tms(uint32_t bits, int nbits)
{
    pio_sm_set_enabled(jtag.pio, jtag.sm_data, false);
    pio_sm_set_enabled(jtag.pio, jtag.sm_tms, true);

}

void jtag_shift_data(uint8_t *tdi, uint8_t *tdo, int nbits)
{
    pio_sm_set_enabled(jtag.pio, jtag.sm_tms, false);
    pio_sm_set_enabled(jtag.pio, jtag.sm_data, true);

    int nbytes = (nbits + 7) / 8;
    int bits_left = nbits;

    while (nbytes > 0) {
	uint32_t out_bits = 0;
	int out_nbits = bits_left > 32 ? 32 : bits_left;
	for (int i = 0; i < (out_nbits + 7) / 8; i++) {
	    out_bits <<= 8;
	    out_bits |= *tdi++;
	    nbytes--;
	}
	bits_left -= out_nbits;

	// Shift to align to MSB
	if (out_nbits < 32) {
	    out_bits <<= (32 - out_nbits);
	}

	pio_sm_put_blocking(jtag.pio, jtag.sm_data, out_bits);

	// Read back
	uint32_t in_bits = pio_sm_get_blocking(jtag.pio, jtag.sm_data);
	if (tdo) {
	    if (out_nbits < 32) {
		in_bits >>= (32 - out_nbits);
	    }
	    for (int i = (out_nbits + 7) / 8 - 1; i >= 0; i--) {
		*tdo++ = (in_bits >> (i * 8)) & 0xff;
	    }
	}
    }
}
