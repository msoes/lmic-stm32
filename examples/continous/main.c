/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lmic.h"
#include "debug.h"

//////////////////////////////////////////////////
// CONFIGURATION (FOR APPLICATION CALLBACKS BELOW)
//////////////////////////////////////////////////

// application router ID (LSBF!)
//static const u1_t APPEUI[8]  = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0x04, 0x1D  };
static const u1_t APPEUI[8]  = { 0x1D, 0x04, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70  };

// unique device ID (LSBF!)
//static const u1_t DEVEUI[8]  = { 0x70, 0xB3, 0xD5, 0x70, 0x8F, 0xFF, 0xFA, 0x03 };
static const u1_t DEVEUI[8]  = { 0x03, 0xFA, 0xFF, 0x8F, 0x70, 0xD5, 0xB3, 0x70 };

// device-specific AES key (derived from device EUI)
static const u1_t DEVKEY[16] = { 0x7A, 0x9F, 0xDF, 0x3A, 0xEB, 0x16, 0x2D, 0xD5, 0xCF, 0x2B, 0x97, 0x9E, 0xAC, 0xC7, 0x68, 0x3D };


//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
}


//////////////////////////////////////////////////
// MAIN - INITIALIZATION AND STARTUP
//////////////////////////////////////////////////

// initial job
static void initfunc (osjob_t* j) {
    // reset MAC state
    LMIC_reset();
    // start joining
    LMIC_startJoining();
    // init done - onEvent() callback will be invoked...
}


// application entry point
int main () {
    osjob_t initjob;

    // initialize runtime env
    os_init();
    // initialize debug library
    debug_init();
    // setup initial job
    os_setCallback(&initjob, initfunc);
    // execute scheduled jobs and events
    os_runloop();
    // (not reached)
    return 0;
}


//////////////////////////////////////////////////
// UTILITY JOB
//////////////////////////////////////////////////

static osjob_t blinkjob;
static u1_t ledstate = 0;
static u2_t blinkfreq = 500;

static void blinkfunc (osjob_t* j) {
    // toggle LED
    ledstate = !ledstate;
    debug_led(ledstate);
    // reschedule blink job
    os_setTimedCallback(j, os_getTime()+ms2osticks(blinkfreq), blinkfunc);
}



//////////////////////////////////////////////////
// UTILITY JOB
//////////////////////////////////////////////////

int idle_sleep_cnt;
int deep_sleep_cnt;



static osjob_t reportjob;

static int reportcnt = 0;

static u2_t reportintvlsecs = 10;

// report sensor value every minute
static void reportfunc (osjob_t* j) {
    // read sensor
    debug_val("reportcnt = ", reportcnt);

    ledstate = !ledstate;
    debug_led(ledstate);
    
    // prepare and schedule data for transmission
    LMIC.frame[0] = (reportcnt << 24)&0xff;
    LMIC.frame[1] = (reportcnt << 16)&0xff;
    LMIC.frame[2] = (reportcnt << 16)&0xff;
    LMIC.frame[3] = reportcnt&0xff;
    LMIC.frame[4] = (idle_sleep_cnt << 24)&0xff;
    LMIC.frame[5] = (idle_sleep_cnt << 16)&0xff;
    LMIC.frame[6] = (idle_sleep_cnt << 16)&0xff;
    LMIC.frame[7] = idle_sleep_cnt&0xff;
    LMIC.frame[8] = (deep_sleep_cnt << 24)&0xff;
    LMIC.frame[9] = (deep_sleep_cnt << 16)&0xff;
    LMIC.frame[10] = (deep_sleep_cnt << 16)&0xff;
    LMIC.frame[11] = deep_sleep_cnt&0xff;
    reportcnt++;
    LMIC_setTxData2(1, LMIC.frame, 12, 0); // (port 1, 2 bytes, unconfirmed)

    // reschedule job in 60 seconds
    os_setTimedCallback(j, os_getTime()+sec2osticks(reportintvlsecs), reportfunc);
}


//////////////////////////////////////////////////
// LMIC EVENT CALLBACK
//////////////////////////////////////////////////

void onEvent (ev_t ev) {
    debug_event(ev);

    switch(ev) {

      // starting to join network
      case EV_JOINING:
          // start blinking
          blinkfunc(&blinkjob);
          break;
          
      // network joined, session established
      case EV_JOINED:
          // cancel blink job
          os_clearCallback(&blinkjob);
          // lets send data
          reportfunc(&reportjob);
          break;
    }
}
