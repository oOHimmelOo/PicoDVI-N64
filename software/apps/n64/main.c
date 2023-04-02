// #pragma GCC optimize("Os")
// #pragma GCC optimize("O2")
#pragma GCC optimize("O3")


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
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 120
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

#define LED_PIN 16

#define UART_TX_PIN (28)
#define UART_RX_PIN (29) /* not available on the pico */
#define UART_ID     uart0
#define BAUD_RATE   115200

// #define USE_RGB565
// #define USE_RGB555
#define USE_RGB555_RAW


#define RGB888_TO_RGB565(_r, _g, _b) \
    (                                \
        (((_r) & 0xf8) <<  8) |      \
        (((_g) & 0xfc) <<  3) |      \
        (((_b))        >>  3)        \
    )


const PIO pio = pio1;
const uint sm = 0;
struct dvi_inst dvi0;
uint16_t framebuf[FRAME_WIDTH * FRAME_HEIGHT];

void core1_main() {
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
    dvi_start(&dvi0);
    dvi_scanbuf_main_16bpp(&dvi0);
    __builtin_unreachable();
}

void core1_scanline_callback() {
    static int h_offset;

    // Discard any scanline pointers passed back
    uint16_t *bufptr;
    while (queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
        ;
    // Note first two scanlines are pushed before DVI start
    static uint scanline = 2;
    bufptr = &framebuf[FRAME_WIDTH * scanline + (h_offset >> 4) * 2];
    queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
    scanline = (scanline + 1) % FRAME_HEIGHT;

// Enable to scroll horizontally
#if 0
    if (scanline == 0) {
        h_offset += 1;
        if (h_offset == 640 * 2) {
            h_offset = 0;
        }
    }
#endif
}


#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))


int main(void) {
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
    sprite_fill16(framebuf, RGB888_TO_RGB565(0xFF, 0x00, 0x00), FRAME_WIDTH * FRAME_HEIGHT);
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
    uint offset = pio_add_program(pio, &n64_program);
    n64_program_init(pio, 0, offset);
    pio_sm_set_enabled(pio, 0, true);

    int count = 0;
    int row = 0;
    // int column = 0;

    #define CSYNCB_POS (0)
    #define HSYNCB_POS (1)
    #define CLAMPB_POS (2)
    #define VSYNCB_POS (3)

    #define CSYNCB_MASK (1 << CSYNCB_POS)
    #define HSYNCB_MASK (1 << HSYNCB_POS)
    #define CLAMPB_MASK (1 << CLAMPB_POS)
    #define VSYNCB_MASK (1 << VSYNCB_POS)

    #define ACTIVE_PIXEL_MASK (VSYNCB_MASK | HSYNCB_MASK | CLAMPB_MASK)

    /*
    0      8       10   15    1B  1F
                    v    v     v   v
                    RRRRRGGGGGGBBBBB
   xBBBBBBBxGGGGGGGxRRRRRRRXXXXVLHC
                 BBBBBBBxGGGGGGGxRRRRRRRxXXXXVLHC
                               BBBBBBBxGGGGGGGxRRRRRRRxXXXXVLHC
                 BBBBBBBxGGGGGGGxRRRRRRRxXXXXVLHC
    */

    uint32_t BGRS;
    while (1) {
        // printf("START\n");

        // 1. Find posedge VSYNC
        do {
            BGRS = pio_sm_get_blocking(pio, sm);
        } while (!(BGRS & VSYNCB_MASK));

        // printf("VSYNC\n");

        int active_row = 0;
        for (row = 0; ; row++) {

            int skip_row = (
                // (row % 2 != 0) ||            // Skip every second line (TODO: Add blend option later)
                (row < 89) ||                // Libdragon top-aligned
                (active_row >= FRAME_HEIGHT) // Never attempt to write more rows than the framebuffer
            );

            // 2. Find posedge HSYNC
            do {
                BGRS = pio_sm_get_blocking(pio, sm);

                if ((BGRS & VSYNCB_MASK) == 0) {
                    // VSYNC found, time to quit
                    goto end_of_line;
                }

            } while ((BGRS & ACTIVE_PIXEL_MASK) != ACTIVE_PIXEL_MASK);

            if (skip_row) {
                // Skip rows based on logic above
                do {
                    BGRS = pio_sm_get_blocking(pio, sm);

                    if ((BGRS & VSYNCB_MASK) == 0) {
                        // VSYNC found, time to quit
                        goto end_of_line;
                    }
                } while ((BGRS & ACTIVE_PIXEL_MASK) == ACTIVE_PIXEL_MASK);

                continue;
            }

            // printf("HSYNC\n");
            count = active_row * FRAME_WIDTH;
            int count_max = count + FRAME_WIDTH;
            active_row++;

            // column = 0;

            // 3.  Capture scanline

            // 3.1 Crop left black bar
            // const int left_crop = 42; // LibDragon 320x240 left-aligned
            const int left_crop = 36; // LibDragon 640x240 left-aligned
            for (int left_ctr = 0; left_ctr < left_crop; left_ctr++) {
                BGRS = pio_sm_get_blocking(pio, sm);
            };

            // 3.2 Capture active pixels
            BGRS = pio_sm_get_blocking(pio, sm);
            int flipflop = 0;
            do {
                // 3.3 Convert to RGB565 or 555
                if (flipflop == 0) {
                    framebuf[count] = (
    #if defined(USE_RGB565)
                        ((BGRS <<  1) & 0xf800) |
                        ((BGRS >> 12) & 0x07e0) |
                        ((BGRS >> 26) & 0x001f)
                        // | 0x1f // Uncomment to tint everything with blue
    #elif defined(USE_RGB555)
                        ((BGRS <<  1) & 0xf800) |
                        ((BGRS >> 12) & 0x07c0) | // Mask so only 5 bits for green are used
                        ((BGRS >> 26) & 0x001f)
                        // | 0x1f // Uncomment to tint everything with blue
    #elif defined(USE_RGB555_RAW)
                        BGRS >> 8
    #else
    #error Define USE_RGB565 or USE_RGB555
    #endif
                    );
                } else {
                    const uint32_t src = framebuf[count];
                    const uint32_t R = (((src >>  0) & 0b11111) + ((BGRS >>  8) & 0b11111)) >> 1;
                    const uint32_t G = (((src >>  5) & 0b11111) + ((BGRS >> 13) & 0b11111)) >> 1;
                    const uint32_t B = (((src >> 10) & 0b11111) + ((BGRS >> 18) & 0b11111)) >> 1;
                    framebuf[count] = R | (G<<5) | (B<<10);
                    count++;
                }
                flipflop++;


                // Never write more than the line width.
                // Input might be weird and have too many active pixels - discard in those cases.
                if (count >= count_max) {
                    do {
                        // Consume all active pixels
                        BGRS = pio_sm_get_blocking(pio, sm);
                    } while ((BGRS & ACTIVE_PIXEL_MASK) == ACTIVE_PIXEL_MASK);
                    break;
                }

                // 3.4 Skip every second pixel
                // BGRS = pio_sm_get_blocking(pio, sm);
                // column++;

                // Skip one extra pixel, for debugging
                // BGRS = pio_sm_get_blocking(pio, sm);
                // column++;

                // Fetch new pixel in the end, so the loop logic can react to it first
                BGRS = pio_sm_get_blocking(pio, sm);

#if 0
                // Optional code to check for active pixels.
                // Disabled for performance reasons.
                if ((BGRS & ACTIVE_PIXEL_MASK) != ACTIVE_PIXEL_MASK) {
                    break;
                }
#endif
            } while (1);
        }

end_of_line:

    }
    __builtin_unreachable();
}

