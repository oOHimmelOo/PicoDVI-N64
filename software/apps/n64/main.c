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

uint32_t ringbuf[1024*8];
uint32_t ringbuf_ctr;
uint32_t ringbuf_idx;
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))


#define CAPTURE_SAMPLES (1024*4)
uint32_t capture_buf1[CAPTURE_SAMPLES];
uint32_t capture_buf2[CAPTURE_SAMPLES];
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
		printf("Application is lagging behind!\r\n");
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
	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

	dma_chan = dma_claim_unused_channel(true);
	printf("dma_chan=%d\r\n", dma_chan);

	dma_channel_config c = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	// read 0, 1, 2, 3 <loop>
	channel_config_set_ring(&c, false, 4);
	channel_config_set_read_increment(&c, false);

	// write 0, 1, 2, ... <stop>
	// channel_config_set_ring(&c, true, ARRAY_SIZE(capture_buf1));
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

	uint32_t BGRS_r = 0;


	#define NOT_CSYNC_POS (0)
	#define NOT_HSYNC_POS (1)
	#define NOT_CLAMP_POS (2)
	#define NOT_VSYNC_POS (3)

	uint8_t sync = 0;
	uint8_t sync_r = 0;

	int is_synced = 0;
	int sync_loops = 0;
	uint32_t sync_patterns[16];

	memset(sync_patterns, 0x00, sizeof(sync_patterns));

typedef enum {
							// 3519: 0 0 1 1: 695 
							// 3520: 1 1 1 1: 58 
							// 3521: 1 1 0 1: 6 
							// 3522: 1 1 1 1: 35 
							// 3523: 0 0 1 1: 695 
							// 3524: 1 1 1 1: 58 
							// 3525: 1 1 0 1: 6 
							// 3526: 1 1 1 1: 35 

							// 3527: 0 0 1 1: 695 
							// 3528: 1 1 1 1: 58 
							// 3529: 0 0 1 1: 736 
							// 3530: 1 1 1 1: 58 
							// 3531: 0 0 1 1: 736 
							// 3532: 1 1 1 1: 58 


	STATE_ROW_FP,           // 1111 58  ROW_ACTIVE      -> ROW_FP
	STATE_ROW_SYNC,         // 1101 6   ROW_FP          -> ROW_SYNC
	STATE_ROW_BP,           // 1111 35  ROW_SYNC        -> ROW_BP
	STATE_ROW_ACTIVE,       // 0011 695 ROW_BP          -> ROW_ACTIVE -> ROW_FP (loop)

	STATE_BLANK1,           // 0011 736 ROW_FP          -> BLANK1
	STATE_BLANK2,           // 1111 58  BLANK1          -> BLANK2
	STATE_BLANK3,           // 0011 736 BLANK2          -> BLANK3
	STATE_BLANK4,           // 1111 58  BLANK3          -> BLANK4

							// Let's ignore all pixels with ~VS=0
							// 3533: 0 0 1 0: 736 
							// 3534: 0 1 1 0: 58 
							// 3535: 1 1 1 0: 678 
							// 3536: 0 1 1 0: 58 
							// 3537: 0 0 1 0: 2 
							// 3538: 0 1 1 0: 57 
							// 3539: 1 1 1 0: 678 
							// 3540: 0 0 1 0: 58 
							// 3541: 0 1 1 0: 58 
	STATE_BLANK5,           // xxx0 2383  BLANK4        -> BLANK5

							// 3542: 0 1 1 1: 339 <--- Really weird ones, notice 0b0111
							// 3543: 0 0 1 1: 397 <--- Really weird ones
							// 3544: 1 1 1 1: 58 
							// 3545: 0 0 1 1: 736 <--- Two extra wide ones
							// 3546: 1 1 1 1: 58 
							// 3547: 0 0 1 1: 736 <--- Two extra wide ones
	STATE_BLANK6,           // 0111 339	BLANK5           -> BLANK6
	STATE_BLANK7,           // 0011 397	BLANK6           -> BLANK7
	STATE_BLANK8,           // 1111 58	BLANK7           -> BLANK8
	STATE_BLANK9,           // 0011 736	BLANK8           -> BLANK9
	STATE_BLANK10,          // 1111 58	BLANK9           -> BLANK10
	STATE_BLANK11,          // 0011 736	BLANK10          -> BLANK11 -> STATE_ROW_FP [Start Of Frame]

							// -> STATE_ROW_FP [Start Of Frame]
							// 3548: 1 1 1 1: 58 
							// 3549: 1 1 0 1: 6 
							// 3550: 1 1 1 1: 35 
							// 3551: 0 0 1 1: 695 <---- now it's normal again
							// 3552: 1 1 1 1: 58 
							// 3553: 1 1 0 1: 6 
							// 3554: 1 1 1 1: 35 
							// 3555: 0 0 1 1: 695 
							// 3556: 1 1 1 1: 58 
};





	while (1) {
		// Wait for DMA transfer to finish
		__wfi();

		if (dma_buf_consumed) {
			// The handled irq wasn't our DMA transfer (dma_handler sets this to 0)
			return;
		}

		// Make a copy of the read_buf pointer - DMA irq should finish before we're done.
		uint32_t *consume_buf = read_buf;

		for (int i = 0; i < CAPTURE_SAMPLES; i++) {
			// Grab n64 video data and increment the pointer
			uint32_t BGRS = *(consume_buf++);

#if 1
			sync = BGRS & 0xFF;
			count++;

			#define MASK      (0b1010)
			#define PATTERN_R (0b1000) // VSYNC | ~HSYNC
			#define PATTERN   (0b1010) // VSYNC |  HSYNC

			// if (((sync_r & MASK) == PATTERN_R) && ((sync & MASK) == PATTERN)) {
			if (sync_r != sync) {
				ringbuf_put(sync | (count << 8));
				count = 0;
			}

			sync_r = sync;

#elif 1

			sync = BGRS & 0xFF;
			count++;


			if (!is_synced) {
				int is_hsync = !(BGRS & NOT_HSYNC_POS);
				int is_vsync = !(BGRS & NOT_HSYNC_POS);

				int was_hsync = !(BGRS_r & NOT_HSYNC_POS);
				int was_vsync = !(BGRS_r & NOT_HSYNC_POS);

				sync_patterns[sync]++;


				// if (sync_r != sync) {
				// 	sync_patterns[sync_r & 0xF] = count;
				// 	count = 0;

				// 	sync_loops++;

				// 	if (sync_loops == 1024000) {
				// 		sync_loops = 0;
				// 		for (int i = 0; i < 16; i++) {
				// 			printf("Sync[%02X] = %d\r\n", i, sync_patterns[i]);
				// 		}
				// 	}
				// }

				sync_r = sync;
			}


#else

			// framebuf[row * FRAME_WIDTH + count++] = RGB888_TO_RGB565(
			// if ((BGRS & 0b1110) == 0b1110) {
				framebuf[count] = ( (BGRS >> 26) & 0x1f);
				// framebuf[row * FRAME_WIDTH + column] =
					// (((BGRS >> 10) & 0x1f) << 11) |
					// (((BGRS >> 16) & 0x3f) << 5) |
					// ( (BGRS >> 26) & 0x1f);

				count++;

				// if (count >= FRAME_WIDTH * FRAME_HEIGHT) {
				// 	count = 0;
				// }
			// }

#endif
		}

		if (count >= FRAME_WIDTH * FRAME_HEIGHT - CAPTURE_SAMPLES) {
			count = 0;
		}

		// Inform the DMA handler that we are done
		dma_buf_consumed = 1;

	}
	__builtin_unreachable();
}

