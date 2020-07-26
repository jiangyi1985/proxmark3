//-----------------------------------------------------------------------------
// Ivan Jiang, 2020
//
// This code is licensed to you under the terms of the GNU GPL, version 2 or,
// at your option, any later version. See the LICENSE.txt file for the text of
// the license.
//-----------------------------------------------------------------------------
// Usage:
//          Very easy-to-use em410x read-and-replay maneuver, as well as writing
//          to T55x7 card and store read ID to flash (RDV4).
//
// MODE & LED:
//          Mode            LED     Start When      Working LED     When Finished        
//          ---------------------------------------------------------------------------------
//          READ (default)  A ON    auto begin      D blinking      switch to Simulation mode   
//          SIMULATION      B ON    auto begin  
//          WRITE           C ON    button held    
//
//          >>> SINGLE BUTTON CLICK to switch between modes <<<
//
//
//          To recall stored ID from flash execute:
//              mem spifss dump o emdump p
//          Or:
//              mem spifss dump o emdump f emdump
//              Then from shell:
//                  hexdump emdump -e '5/1 "%02X" /0 "\n"'
//-----------------------------------------------------------------------------
#include "standalone.h"
#include "proxmark3_arm.h"
#include "appmain.h"
#include "fpgaloader.h"
#include "lfops.h"
#include "util.h"
#include "dbprint.h"
#include "ticks.h"
#include "string.h"
#include "BigBuf.h"
#include "spiffs.h"
#include "commonutil.h"

#ifdef WITH_FLASH
#include "flashmem.h"
#endif

#define CLOCK 64 //for 125kHz

static uint64_t low = 0;
static uint32_t high = 0;
static uint8_t *bba;
static int buflen;
static uint8_t mode_count = 3;

void ModInfo(void) {
    DbpString("  LF EM410x fast read/sim/write(t55x7) - aka IvanRun (Ivan Jiang)");
}

static void led_state(int i)
{
    LEDsoff();
    LED(1 << i, 0);
}

static uint64_t ReversQuads(uint64_t bits) {
    uint64_t result = 0;
    for (int i = 0; i < 16; i++) {
        result += ((bits >> (60 - 4 * i)) & 0xf) << (4 * i);
    }
    return result >> 24;
}

static void FillBuff(uint8_t bit) {
    memset(bba + buflen, bit, CLOCK / 2);
    buflen += (CLOCK / 2);
    memset(bba + buflen, bit ^ 1, CLOCK / 2);
    buflen += (CLOCK / 2);
}

static void ConstructEM410xEmulBuf(uint64_t id) {
    int i, j, binary[4], parity[4];
    buflen = 0;
    for (i = 0; i < 9; i++)
        FillBuff(1);
    parity[0] = parity[1] = parity[2] = parity[3] = 0;
    for (i = 0; i < 10; i++) {
        for (j = 3; j >= 0; j--, id /= 2)
            binary[j] = id % 2;
        for (j = 0; j < 4; j++)
            FillBuff(binary[j]);
        FillBuff(binary[0] ^ binary[1] ^ binary[2] ^ binary[3]);
        for (j = 0; j < 4; j++)
            parity[j] ^= binary[j];
    }
    for (j = 0; j < 4; j++)
        FillBuff(parity[j]);
    FillBuff(0);
}

#ifdef WITH_FLASH
static void SaveIDtoFlash(int addr, uint64_t id) {
    uint8_t bt[5];
    const char *filename = "emdump";
    rdv40_spiffs_mount();
    for (int i = 0; i < 5; i++) {
        bt[4 - i] = (uint8_t)(id >> 8 * i & 0xff);
    }
    if (exists_in_spiffs(filename) == false) {
        rdv40_spiffs_write(filename, &bt[0], 5, RDV40_SPIFFS_SAFETY_NORMAL);
    } else {
        rdv40_spiffs_append(filename, &bt[0], 5, RDV40_SPIFFS_SAFETY_NORMAL);
    }
}
#endif

void RunMod(void) {
    StandAloneMode();
    FpgaDownloadAndGo(FPGA_BITSTREAM_LF);
    Dbprintf("[=] >>  LF EM410x fast read/sim/write(t55x7) started  <<");

    //      0 - read
    //      1 - simulate
    //      2 - write to t55x7
    uint8_t state = 0;

    bba = BigBuf_get_addr();

    // Dbprintf("[=] state --> %i, low --> %02x%08x", state, (uint32_t)(low>>32),(uint32_t)low);
    Dbprintf("[=] state --> %i", state);
    led_state(state);

    for (;;) {
        WDT_HIT();

        // exit from standalone mode, just send a usbcommand
        if (data_available()) break;

        // Was our button held down or pressed?
        int button_pressed = BUTTON_HELD(600);
        // if (button_pressed != BUTTON_HOLD)
        //     continue;

        switch (button_pressed)
        {
            case BUTTON_SINGLE_CLICK:
                Dbprintf("[=] BUTTON_SINGLE_CLICK");

                //switch to next state
                state = (state + 1) % mode_count;
                Dbprintf("[=] state --> %i", state);
                led_state(state);

                break;
            case BUTTON_HOLD:
                // Dbprintf("[=] btn hold");
                
                //indicate btn holding
                LEDsoff();

                WAIT_BUTTON_RELEASED();
                led_state(state);
                // Dbprintf("[=] btn released");

                break;
            default:
                break;
        }

        //SpinDelay(300);

        switch (state) {
            case 0:
                // Read mode
                //is_reading = true;
                lf_em410x_watch(1, &high, &low);
                Dbprintf("[=] read stopped. state --> %i, low --> %02x%08x", state, (uint32_t)(low>>32), (uint32_t)low);
                
#ifdef WITH_FLASH
                if(low != 0) {
                    SaveIDtoFlash(0, low);
                }
#endif

                //wait for possible button click event (if user cancel) to timeout 
                //before entering the next "switch(button_pressed)"
                SpinDelay(500);

                //switch to simulate mode
                state = 1;
                Dbprintf("[=] state --> %i", state);
                led_state(state);
                break;
            case 1:
                // Simulate mode

                //go back to read mode if there's no tag id
                if(low == 0) {
                    state = 0;
                    Dbprintf("[=] state --> %i", state);
                    led_state(state);
                    break;
                }

                // Dbprintf("[=] low: %02x%08x", (uint32_t)(low>>32), (uint32_t)low);
                // Dbprintf("[=] low: %02x%08x", (uint32_t)(ReversQuads(low)>>32), (uint32_t)ReversQuads(low));
                ConstructEM410xEmulBuf(ReversQuads(low));
                // Dbprintf("[=] buflen: %i", buflen);
                SimulateTagLowFrequency(buflen, 0, true);           
                break;
            case 2:
                // Write mode
                if (button_pressed == BUTTON_HOLD) {
                    //copy to t55x7
                    copy_em410x_to_t55xx(1, CLOCK, (uint32_t)(low >> 32), (uint32_t)(low & 0xffffffff));

                    //leds are turned off by copy_em410x_to_t55xx command
                    led_state(state);
                }
                break;
        }
    }

    DbpString("[=] exiting lf_em410_ivanrun");
    LEDsoff();
}
