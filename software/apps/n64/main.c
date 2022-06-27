#include <stdio.h>
#include <stdlib.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/structs/bus_ctrl.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "pico/stdlib.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "sprite.h"

#include "n64.pio.h"

// TMDS bit clock 252 MHz
// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

#define LED_PIN 16

#define UART_TX_PIN (28)
#define UART_RX_PIN (29) /* not available on the pico */
#define UART_ID     uart0
#define BAUD_RATE   115200


struct dvi_inst dvi0;
uint16_t framebuf[FRAME_WIDTH * FRAME_HEIGHT];

void core1_main() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	dvi_start(&dvi0);
	dvi_scanbuf_main_16bpp(&dvi0);
	__builtin_unreachable();
}

void core1_scanline_callback() {
	// Discard any scanline pointers passed back
	uint16_t *bufptr;
	while (queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
		;
	// // Note first two scanlines are pushed before DVI start
	static uint scanline = 2;
	bufptr = &framebuf[FRAME_WIDTH * scanline];
	queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
	scanline = (scanline + 1) % FRAME_HEIGHT;
}

uint32_t ringbuf[1024*1];
uint32_t ringbuf_ctr;
uint32_t ringbuf_idx;
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))


#define CAPTURE_SIZE (1024*8)
uint32_t capture_buf1[CAPTURE_SIZE];
uint32_t capture_buf2[CAPTURE_SIZE];
uint32_t *capture_buf = capture_buf1;
uint32_t *read_buf = capture_buf2;

static void ringbuf_print_and_reset(void)
{
	for (int i = 0; i < ARRAY_SIZE(ringbuf); i++) {
		uint32_t entry = ringbuf[i];
		// printf("%d: %d\r\n", ringbuf_ctr + i, entry);
		printf("%d:", ringbuf_ctr + i);
		printf(" %d %d %d %d: %d \r\n", !!(entry & 1), !!(entry & 2), !!(entry & 4), !!(entry & 8), entry >> 8);
	}

	ringbuf_idx = 0;
	ringbuf_ctr += ARRAY_SIZE(ringbuf);
}

static void ringbuf_put(uint32_t entry)
{
    ringbuf[ringbuf_idx++] = entry;

    if (ringbuf_idx < ARRAY_SIZE(ringbuf))
        return;

    ringbuf_print_and_reset();
}




int dma_chan;
int dma_count = 0;
int dma_buf_consumed = 1;

void dma_handler(void)
{
	// Clear the interrupt request.
	dma_hw->ints0 = 1u << dma_chan;

	if (!dma_buf_consumed) {
		// Application is lagging behind!
		printf(!"Shieeet!\r\n");
		// don't start another DMA
		return;
	}

	dma_buf_consumed = 0;
	dma_count++;

	// Swap write/read buffers
	if (capture_buf == capture_buf1) {
		capture_buf = capture_buf2;
		read_buf = capture_buf1;
	} else {
		capture_buf = capture_buf1;
		read_buf = capture_buf2;
	}

	dma_channel_set_write_addr(dma_chan, capture_buf, true);
}




int main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
#ifdef RUN_FROM_CRYSTAL
	set_sys_clock_khz(12000, true);
#else
	// Run system at TMDS bit clock
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
#endif

	// setup_default_uart();
	stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	printf("Configuring DVI\n");

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi0.scanline_callback = core1_scanline_callback;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	// Once we've given core 1 the framebuffer, it will just keep on displaying
	// it without any intervention from core 0
	sprite_fill16(framebuf, 0xffff, FRAME_WIDTH * FRAME_HEIGHT);
	uint16_t *bufptr = framebuf;
	queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
	bufptr += FRAME_WIDTH;
	queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);

	printf("Core 1 start\n");
	multicore_launch_core1(core1_main);

	printf("Start rendering\n");

	// for (int y = 0; y < FRAME_HEIGHT; y++) {
	// 	for (int x = 0; x < FRAME_WIDTH; x++) {
	// 		framebuf[y*FRAME_WIDTH + x] = x + y;
	// 	}
	// }

	for (int i = 0; i <= 8; i++) {
		gpio_init(i);
		gpio_set_dir(i, GPIO_IN);
		gpio_set_pulls(i, false, false);
	}

	// Init PIO before starting the second core
	PIO pio = pio1;
	uint sm = 0;
	uint offset = pio_add_program(pio, &n64_program);
	n64_program_init(pio, 0, offset);
	pio_sm_set_enabled(pio, 0, true);

	// Enable DMA for second DMA channel (DMA 0 is used by DVI)

	// Grant high bus priority to the DMA, so it can shove the processors out
	// of the way. This should only be needed if you are pushing things up to
	// >16bits/clk here, i.e. if you need to saturate the bus completely.
	// bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;


	dma_chan = dma_claim_unused_channel(true);
	printf("dma_chan=%d\r\n", dma_chan);

	dma_channel_config c = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	// read 0, 1, 2, 3 <loop>
	channel_config_set_ring(&c, false, 4);
	channel_config_set_read_increment(&c, false);

	// write 0, 1, ... 
	// channel_config_set_ring(&c, false, ARRAY_SIZE(capture_buf1));
	channel_config_set_write_increment(&c, true);
	channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

	dma_channel_configure(dma_chan, &c,
		capture_buf,        // Destination pointer
		&pio->rxf[sm],      // Source pointer
		ARRAY_SIZE(capture_buf1), // Number of transfers
		false                // Don't start immediately
	);

	// Tell the DMA to raise IRQ line 0 when the channel finishes a block
	dma_channel_set_irq1_enabled(dma_chan, true);

	// Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
	irq_set_exclusive_handler(DMA_IRQ_1, dma_handler);
	irq_set_enabled(DMA_IRQ_1, true);

	dma_channel_start(dma_chan);

	int count = 0;
	int row = 0;
	int column = 0;
	int should_print;

	uint32_t BGRS_r = 0;

	#define RGB888_TO_RGB565(r, g, b) (\
		(((r >> 3) &0x1f) << 11) | \
		(((g)&0x3f) << 5) | \
		((b)&0x1f)\
	)

	// #define NOT_CSYNC_POS 31
	// #define NOT_HSYNC_POS 30
	// #define NOT_CLAMP_POS 29
	// #define NOT_VSYNC_POS 28

	#define NOT_CSYNC_POS (0)
	#define NOT_HSYNC_POS (1)
	#define NOT_CLAMP_POS (2)
	#define NOT_VSYNC_POS (3)


	// state = STATE_INVALID;

	uint8_t sync;
	uint8_t sync_r;

	while (1) {
#if 0
		// Grab n64 video data
		uint32_t BGRS = pio_sm_get_blocking(pio, 0);

		sync = BGRS & 0xFF;
		count++;

		if (sync != sync_r) {
			if (sync == 0b1100)
				ringbuf_put(sync | (count << 8));
			count = 0;
		}

		sync_r = sync;

#elif 1

		// Handle everything in DMA
		__wfi();

		// for (int i = 0; i < ARRAY_SIZE(capture_buf1); i++) {
		for (int i = 0; i < 1024*2; i++) {
			// Grab n64 video data and increment the pointer
			uint32_t BGRS = *(read_buf++);

			sync = BGRS & 0xFF;
			count++;

			if (sync != sync_r) {
				if (sync == 0b1100)
					ringbuf_put(sync | (count << 8));
				count = 0;
			}

			sync_r = sync;


#if 0
			if (  BGRS_r == 0b1111 && BGRS == 0b1100)
				//  (BGRS   & (1 << NOT_VSYNC_POS)) &&
				//  !(BGRS_r & (1 << NOT_HSYNC_POS)) && (BGRS & (1 << NOT_HSYNC_POS))) {
				{
				// VSync && negedge HSync
				column = 0;
			} else if (BGRS == 0b0011) {
				// HSync stays high
				column++;
			}

			if (!(BGRS_r & (1 << NOT_VSYNC_POS)) && (BGRS & (1 << NOT_VSYNC_POS))) {
				// posedge VSync
				row = 0;
				should_print = 1;
			} else if (BGRS & (1 << NOT_VSYNC_POS)) {
				// VSync stays high
				row++;
			}

			// Limit bounds
			if (row >= FRAME_HEIGHT) {
				row = FRAME_HEIGHT - 1;
			}

			if (column >= FRAME_WIDTH) {
				column = FRAME_WIDTH - 1;
			}

			BGRS_r = BGRS;

			// if (BGRS & 0xFF == 0b1110) {
				// framebuf[row * FRAME_WIDTH + column] = RGB888_TO_RGB565(
				// 	BGRS >> 24,
				// 	BGRS >> 8,
				// 	BGRS);
			// }

			// framebuf[row * FRAME_WIDTH + count++] = RGB888_TO_RGB565(
			// if ((BGRS & 0b1110) == 0b1110) {

				// framebuf[count++] = 
				framebuf[row * FRAME_WIDTH + column] =
					(((BGRS >> 10) & 0x1f) << 11) |
					(((BGRS >> 16) & 0x3f) << 5) |
					( (BGRS >> 26) & 0x1f);

				if (count >= FRAME_WIDTH * FRAME_HEIGHT) {
					count = 0;
				}
			// }

#endif
		}

		// Inform the DMA handler that we are done
		dma_buf_consumed = 1;


#else

		// Grab n64 video data
		uint32_t BGRS = pio_sm_get_blocking(pio, 0);

		if (  BGRS_r == 0b1111 && BGRS == 0b1100)
			//  (BGRS   & (1 << NOT_VSYNC_POS)) &&
			//  !(BGRS_r & (1 << NOT_HSYNC_POS)) && (BGRS & (1 << NOT_HSYNC_POS))) {
			{
			// VSync && negedge HSync
			column = 0;
		} else if (BGRS == 0b0011) {
			// HSync stays high
			column++;
		}

		if (!(BGRS_r & (1 << NOT_VSYNC_POS)) && (BGRS & (1 << NOT_VSYNC_POS))) {
			// posedge VSync
			row = 0;
			should_print = 1;
		} else if (BGRS & (1 << NOT_VSYNC_POS)) {
			// VSync stays high
			row++;
		}

		// Limit bounds
		if (row >= FRAME_HEIGHT) {
			row = FRAME_HEIGHT - 1;
		}

		if (column >= FRAME_WIDTH) {
			column = FRAME_WIDTH - 1;
		}

		BGRS_r = BGRS;

		// if (BGRS & 0xFF == 0b1110) {
			// framebuf[row * FRAME_WIDTH + column] = RGB888_TO_RGB565(
			// 	BGRS >> 24,
			// 	BGRS >> 8,
			// 	BGRS);
		// }

		// framebuf[row * FRAME_WIDTH + count++] = RGB888_TO_RGB565(
		if ((BGRS & 0b1110) == 0b1110) {

			// framebuf[count++] = 
			framebuf[row * FRAME_WIDTH + column] =
				(((BGRS >> 10) & 0x1f) << 11) |
				(((BGRS >> 16) & 0x3f) << 5) |
				( (BGRS >> 26) & 0x1f);

			if (count >= FRAME_WIDTH * FRAME_HEIGHT) {
				count = 0;
			}
		}
#endif
	}
	__builtin_unreachable();
}

