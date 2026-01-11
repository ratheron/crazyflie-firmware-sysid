/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "LEDPATTERNS"

// Pack WRGB into 0xWWRRGGBB
#define WRGB(w, r, g, b)  ( ((uint32_t)(w) << 24) | ((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b) )
// Color extractors
#define GET_R(c)  ((float)(((c) >> 16) & 0xFF))
#define GET_G(c)  ((float)(((c) >>  8) & 0xFF))
#define GET_B(c)  ((float)((c) & 0xFF))
// Clip
#define CLIP(v, low, up)  ( (v) < low ? low : ((v) > up ? up : (v)) )


static uint8_t pattern = 0;
static logVarId_t idX, idY, idZ;
static logVarId_t idVx, idVy, idVz;

static uint32_t prevWrgbBot = 0xFFFFFFFF;
static uint32_t prevWrgbTop = 0xFFFFFFFF;

//----------------General Color definitions-------
#define UINT8MAX 255
#define MAX_BRIGHTNESS 255//64 // 255
#define ONG_BRIGHTNESS 156//39 // 156
#define MID_BRIGHTNESS 128//32 // 128
#define COLOR_OFF       WRGB(0, 0, 0, 0)
#define COLOR_RED       WRGB(0, MAX_BRIGHTNESS, 0, 0)
#define COLOR_GREEN     WRGB(0, 0, MAX_BRIGHTNESS, 0)
#define COLOR_BLUE      WRGB(0, 0, 0, MAX_BRIGHTNESS)
#define COLOR_YELLOW    WRGB(0, MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0)
#define COLOR_GOLD      WRGB(0, MAX_BRIGHTNESS, ONG_BRIGHTNESS, 0)
#define COLOR_ORANGE    WRGB(0, MAX_BRIGHTNESS, MID_BRIGHTNESS, 0)
#define COLOR_TURQUOISE WRGB(0, 0, MAX_BRIGHTNESS, MAX_BRIGHTNESS)
#define COLOR_PINK      WRGB(0, MAX_BRIGHTNESS, 0, MAX_BRIGHTNESS)

//----------------Pattern1: LED Cycle-------------

//------------------------------------------------

//----------------Pattern2: Color Mapping---------
// Flight Space (make a little smaller and clip)
#define MIN_X_BOUND -1.0f
#define MAX_X_BOUND 1.0f
#define MIN_Y_BOUND -1.0f
#define MAX_Y_BOUND 1.0f
#define MIN_Z_BOUND 1.0f
#define MAX_Z_BOUND 2.0f
// Colors height
#define COLOR_HEIGHT_LOW   COLOR_ORANGE
#define COLOR_HEIGHT_HIGH  COLOR_BLUE
// Colors xy
#define COLOR_XY_1   COLOR_RED
#define COLOR_XY_2   COLOR_GREEN
#define COLOR_XY_3   COLOR_GOLD
#define COLOR_XY_4   COLOR_TURQUOISE

//------------------------------------------------

//----------------Pattern3: Christmas Tree--------
#define STAR_HEIGHT 1.5f  // Any drone above this height will shine yellow
//------------------------------------------------

//----------------Pattern4: Snowflake-------------
#define LANDING_Z 0.0f
#define FADE_PERIOD_MS 2000
//------------------------------------------------

//----------------Pattern5: Flicker---------------

// Define limits for off duration (in ms)
#define FLICKER_MIN_OFF 200
#define FLICKER_MAX_OFF 600
// Keep these static for state tracking
static TickType_t nextFlickerTop = 0;
static TickType_t nextFlickerBot = 0;
static uint8_t deckTopOn  = 1;
static uint8_t deckBotOn  = 1;
//------------------------------------------------

//----------------Pattern6: Velocity Indicator--------------
#define MAX_VEL 1.0f   // m/s, adjust as needed

#define COLOR_VEL_MIN WRGB(0,  0,   0, MAX_BRIGHTNESS)   // Blue
#define COLOR_VEL_MID WRGB(0,  0, MAX_BRIGHTNESS,   0)   // Green
#define COLOR_VEL_MAX WRGB(0,MAX_BRIGHTNESS,   0,   0)   // Red
//-----------------------------------------------------------


// Helper function to update WRGB parameters safely
static void updateDeckParamIfChanged(paramVarId_t id, uint32_t newValue, uint32_t *prevValue)
{
    if (id.id != 0xffffu && newValue != *prevValue) {
        paramSetInt(id, newValue);
        *prevValue = newValue;
    }
}


void appMain()
{
    DEBUG_PRINT("Starting WRGB color patterns app...\n");

    // Detect LED decks
    paramVarId_t idBottomDetect = paramGetVarId("deck", "bcColorLedBot");
    paramVarId_t idTopDetect    = paramGetVarId("deck", "bcColorLedTop");

    uint8_t bottomAttached = paramGetUint(idBottomDetect);
    uint8_t topAttached    = paramGetUint(idTopDetect);

    // Deck WRGB param IDs
    paramVarId_t idWrgbBot = {0};
    paramVarId_t idWrgbTop = {0};

    // Thermal log IDs
    logVarId_t idDeckTemp    = {0}, idThrottlePct = {0};

    // Brightness correction
    if (bottomAttached) {
        idWrgbBot = paramGetVarId("colorLedBot", "wrgb8888");
        idDeckTemp    = logGetVarId("colorLedBot", "deckTemp");
        idThrottlePct = logGetVarId("colorLedBot", "throttlePct");
        paramSetInt(paramGetVarId("colorLedBot", "brightCorr"), 1);
        DEBUG_PRINT("Color LED Bottom deck detected\n");
    }
    if (topAttached) {
        idWrgbTop = paramGetVarId("colorLedTop", "wrgb8888");
        idDeckTemp    = logGetVarId("colorLedTop", "deckTemp");
        idThrottlePct = logGetVarId("colorLedTop", "throttlePct");
        paramSetInt(paramGetVarId("colorLedTop", "brightCorr"), 1);
        DEBUG_PRINT("Color LED Top deck detected\n");
    }

    uint8_t r = 0, g = 0, b = 0, w = 0;
    int step = 0;

    TickType_t lastWakeTime = xTaskGetTickCount();
    uint32_t lastThermalCheck = xTaskGetTickCount();
    const uint32_t thermalCheckInterval = M2T(100);

    /* Track previous pattern to detect transitions */
    uint8_t prevPattern = pattern; // initialize to current value so we don't clear immediately

    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");
    idZ = logGetVarId("stateEstimate", "z");

    idVx = logGetVarId("stateEstimate", "vx");
    idVy = logGetVarId("stateEstimate", "vy");
    idVz = logGetVarId("stateEstimate", "vz");

    while (1)
    {
        /* If the parameter was changed since last iteration, handle the transition */
        if (prevPattern != pattern) {
            DEBUG_PRINT("ledpat.pattern changed %d -> %d\n", prevPattern, pattern);

            // Clear prevWrgb to force updates in the new pattern
            prevWrgbBot = 0xFFFFFFFF;
            prevWrgbTop = 0xFFFFFFFF;

            // If we just switched into pattern == 0, clear the LEDs once.
            if (pattern == 0) {
            if (bottomAttached) paramSetInt(idWrgbBot, COLOR_OFF);
            if (topAttached)    paramSetInt(idWrgbTop, COLOR_OFF);
            }

            prevPattern = pattern;
    }

    if (pattern == 1)
    {
        //---------------------------------------------------------
        // LED CYCLE
        //---------------------------------------------------------
        int phase = step / (UINT8MAX+1);
        int value = step % (UINT8MAX+1);

        switch(phase) {
        case 0:  // G: 0→255, R: 255→0
            r = MAX_BRIGHTNESS - value;
            g = value;
            b = 0;
            w = 0;
            break;

        case 1:  // B: 0→255, G: 255→0
            r = 0;
            g = MAX_BRIGHTNESS - value;
            b = value;
            w = 0;
            break;

        case 2:  // W: 0→255, B: 255→0
            r = 0;
            g = 0;
            b = MAX_BRIGHTNESS - value;
            w = value;
            break;

        case 3: // R: 0→255, W: 255→0
            r = value;
            g = 0;
            b = 0;
            w = MAX_BRIGHTNESS - value;
            break;
        }

        uint32_t wrgb_value = WRGB(w,r,g,b);

        // Set both decks to same value
        if (bottomAttached) paramSetInt(idWrgbBot, wrgb_value);
        if (topAttached)    paramSetInt(idWrgbTop, wrgb_value);

        step = (step + 1) % ((UINT8MAX+1) * 4);  // original behavior
    }
    else if (pattern == 2)
    {
        //---------------------------------------------------------
        // COLOR MAPPING
        //---------------------------------------------------------
        float x = logGetFloat(idX);
        float y = logGetFloat(idY);
        float z = logGetFloat(idZ);

        // NORMALIZED 0–1 POSITION
        float xn = (x - MIN_X_BOUND) / (MAX_X_BOUND - MIN_X_BOUND);
        float yn = (y - MIN_Y_BOUND) / (MAX_Y_BOUND - MIN_Y_BOUND);
        float zn = (z - MIN_Z_BOUND) / (MAX_Z_BOUND - MIN_Z_BOUND);
        xn = CLIP(xn, 0.0f, 1.0f);
        yn = CLIP(yn, 0.0f, 1.0f);
        zn = CLIP(zn, 0.0f, 1.0f);

        //---------------------------------------------------------
        // BOTTOM DECK: HEIGHT
        //---------------------------------------------------------
        float br = GET_R(COLOR_HEIGHT_LOW)  + (GET_R(COLOR_HEIGHT_HIGH) - GET_R(COLOR_HEIGHT_LOW)) * zn;
        float bg = GET_G(COLOR_HEIGHT_LOW)  + (GET_G(COLOR_HEIGHT_HIGH) - GET_G(COLOR_HEIGHT_LOW)) * zn;
        float bb = GET_B(COLOR_HEIGHT_LOW)  + (GET_B(COLOR_HEIGHT_HIGH) - GET_B(COLOR_HEIGHT_LOW)) * zn;

        uint32_t bottom_wrgb = WRGB(0, (uint8_t)br, (uint8_t)bg, (uint8_t)bb);

        //---------------------------------------------------------
        // TOP DECK: XY 
        //---------------------------------------------------------
        // XY_1 → XY_2 (left-to-right at bottom)
        float r12 = GET_R(COLOR_XY_1) + (GET_R(COLOR_XY_2) - GET_R(COLOR_XY_1)) * xn;
        float g12 = GET_G(COLOR_XY_1) + (GET_G(COLOR_XY_2) - GET_G(COLOR_XY_1)) * xn;
        float b12 = GET_B(COLOR_XY_1) + (GET_B(COLOR_XY_2) - GET_B(COLOR_XY_1)) * xn;

        // XY_4 → XY_3 (left-to-right at top)
        float r43 = GET_R(COLOR_XY_4) + (GET_R(COLOR_XY_3) - GET_R(COLOR_XY_4)) * xn;
        float g43 = GET_G(COLOR_XY_4) + (GET_G(COLOR_XY_3) - GET_G(COLOR_XY_4)) * xn;
        float b43 = GET_B(COLOR_XY_4) + (GET_B(COLOR_XY_3) - GET_B(COLOR_XY_4)) * xn;

        // Interpolate bottom→top using Y
        float tr = r12 + (r43 - r12) * yn;
        float tg = g12 + (g43 - g12) * yn;
        float tb = b12 + (b43 - b12) * yn;

        uint32_t top_wrgb = WRGB(0, (uint8_t)tr, (uint8_t)tg, (uint8_t)tb);

        // Set both decks to same value
        if (bottomAttached) paramSetInt(idWrgbBot, bottom_wrgb);
        if (topAttached)    paramSetInt(idWrgbTop, top_wrgb);
    }
    else if (pattern == 3)
    {
        //---------------------------------------------------------
        // CHRISTMAS TREE
        //---------------------------------------------------------
        float z = logGetFloat(idZ);

        if (z > STAR_HEIGHT)
        {
            // Update decks safely
            if (bottomAttached) updateDeckParamIfChanged(idWrgbBot, COLOR_YELLOW, &prevWrgbBot);
            if (topAttached)    updateDeckParamIfChanged(idWrgbTop, COLOR_YELLOW, &prevWrgbTop);
        }
        else
        {

            // Update decks safely
            if (bottomAttached) updateDeckParamIfChanged(idWrgbBot, COLOR_RED, &prevWrgbBot);
            if (topAttached)    updateDeckParamIfChanged(idWrgbTop, COLOR_GREEN, &prevWrgbTop);
        }
    }
    else if (pattern == 4)
    {
        //---------------------------------------------------------
        // SNOWFLAKE
        //---------------------------------------------------------
        // Capture max height when pattern is activated
        static float max_height = 0.0f;
        static uint8_t patternPrev = 0;
        if (patternPrev != 4) {
            max_height = logGetFloat(idZ);  // current height becomes max brightness
        }
        patternPrev = 4;

        // Get current height
        float z = logGetFloat(idZ);

        // Clamp for safety
        if (z < LANDING_Z) z = LANDING_Z;
        if (z > max_height) z = max_height;

        // Linear scaling: LANDING_Z -> 0, max_height -> 1
        float heightFactor = (z - LANDING_Z) / (max_height - LANDING_Z);

        // Sinusoidal fade: smooth pulsing
        TickType_t t = xTaskGetTickCount();
        float fade = 0.5f * (1.0f + sinf(2.0f * 3.14159f * t * portTICK_PERIOD_MS / FADE_PERIOD_MS));

        uint8_t white = (uint8_t)(MAX_BRIGHTNESS * heightFactor * fade);

        // Prepare WRGB value (R=G=B=0)
        uint32_t wrgb_value = ((uint32_t)white << 24) | 0x00000000;

        // Update decks safely
        if (bottomAttached) updateDeckParamIfChanged(idWrgbBot, wrgb_value, &prevWrgbBot);
        if (topAttached)    updateDeckParamIfChanged(idWrgbTop, wrgb_value, &prevWrgbTop);
    }
    else if (pattern == 5)
    {
        //---------------------------------------------------------
        // FLICKER
        //---------------------------------------------------------
        uint32_t wrgb_value = COLOR_GOLD;
        uint32_t wrgb_off   = COLOR_OFF;

        TickType_t t = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // --- Top deck ---
        if (t >= nextFlickerTop)
        {
            deckTopOn = !deckTopOn;  // toggle on/off
            // Randomize next toggle: simple pseudo-random based on tick
            uint32_t dur = FLICKER_MIN_OFF + ((t * 37) & (FLICKER_MAX_OFF - FLICKER_MIN_OFF));
            nextFlickerTop = t + dur;
        }

        // --- Bottom deck ---
        if (t >= nextFlickerBot)
        {
            deckBotOn = !deckBotOn;  // toggle on/off
            uint32_t dur = FLICKER_MIN_OFF + ((t * 53) & (FLICKER_MAX_OFF - FLICKER_MIN_OFF));
            nextFlickerBot = t + dur;
        }

        uint32_t wrgb_top  = deckTopOn  ? wrgb_value : wrgb_off;
        uint32_t wrgb_bot  = deckBotOn  ? wrgb_value : wrgb_off;

        if (topAttached)    updateDeckParamIfChanged(idWrgbTop, wrgb_top, &prevWrgbTop);
        if (bottomAttached) updateDeckParamIfChanged(idWrgbBot, wrgb_bot, &prevWrgbBot);
    }
    else if (pattern == 6)
    {
        //---------------------------------------------------------
        // VELOCITY COLOR INDICATOR
        //---------------------------------------------------------

        float vx = logGetFloat(idVx);
        float vy = logGetFloat(idVy);
        float vz = logGetFloat(idVz);

        float vel = sqrtf(vx*vx + vy*vy + vz*vz);

        // Clamp 0 → MAX_VEL
        if (vel < 0) vel = 0;
        if (vel > MAX_VEL) vel = MAX_VEL;

        float t = vel / MAX_VEL;   // 0→1 normalized speed

        // Smooth gradient Blue → Green → Red
        uint8_t r, g, b;

        if (t < 0.5f) {
            // Blue → Green (0–50%)
            float k = t / 0.5f;
            r = 0;
            g = (uint8_t)(MAX_BRIGHTNESS * k);
            b = (uint8_t)(MAX_BRIGHTNESS * (1.0f - k));
        } else {
            // Green → Red (50–100%)
            float k = (t - 0.5f) / 0.5f;
            r = (uint8_t)(MAX_BRIGHTNESS * k);
            g = (uint8_t)(MAX_BRIGHTNESS * (1.0f - k));
            b = 0;
        }

        uint32_t wrgb_value = WRGB(0, r, g, b);

        if (topAttached)    updateDeckParamIfChanged(idWrgbTop, wrgb_value, &prevWrgbTop);
        if (bottomAttached) updateDeckParamIfChanged(idWrgbBot, wrgb_value, &prevWrgbBot);
    }
    else
    {
        //---------------------------------------------------------
        // pattern == 0 -> DO NOTHING
        // This allows the deck's own parameters & callbacks
        // to control the LEDs normally.
        //---------------------------------------------------------
    }

    // Thermal throttling log
    if (xTaskGetTickCount() - lastThermalCheck >= thermalCheckInterval) {
        uint8_t throttlePct = logGetUint(idThrottlePct);
        if (throttlePct) {
        uint8_t deckTemp = logGetUint(idDeckTemp);
        DEBUG_PRINT("WARNING: Thermal throttling active! Temp: %d°C, Throttle: %d%%\n",
                    deckTemp, throttlePct);
        }
        lastThermalCheck = xTaskGetTickCount();
    }

    vTaskDelayUntil(&lastWakeTime, M2T(10));
    }
}

// -------- PARAMETERS --------
PARAM_GROUP_START(ledpat)
PARAM_ADD(PARAM_UINT8, pattern, &pattern)
PARAM_GROUP_STOP(ledpat)
// ----------------------------