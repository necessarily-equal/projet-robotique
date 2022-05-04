#include "leds.h"

#include <math.h>

#include <spi_comm.h>

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

struct rgb_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

void leds_init(void) {
    spi_comm_start();
}

static struct rgb_t frequency_to_rgb(uint16_t freq) {
    // https://en.wikipedia.org/wiki/HSL_and_HSV#/media/File:HSV-RGB-comparison.svg
    // hue == 0 corresponds to 0° and hue == 1.0f corresponds to 240° on this graph

    if (freq == 0)
	    return {0, 0, 0};

    // bring the frequency to the [0;1] interval
    static const float min = log2f(NOTE_B0);
    static const float max = log2f(NOTE_DS8);
    float hue = (log2f(freq) - min) / (max - min);

    struct rgb_t result = {0, 0, 0};

    // calculate red
    if (hue < 0.25f)
	    result.r = 255;
    else if (hue < 0.5f)
	    result.r = 255.0f * 4.0f * (0.5f - hue);

    // calculate green
    if (hue < 0.0f) // for resilience to very low frequencies
	    ;
    else if (hue < 0.25f)
	    result.g = 255.0f * 4.0f * hue;
    else if (hue < 0.75f)
	    result.g = 255;
    else if (hue < 1.0f)
	    result.g = 255.0f * 4.0f * (1.0f - hue);

    // calculate blue
    if (hue > 0.75f)
	    result.b = 255;
    else if (hue > 0.5f)
	    result.b = 255.0f * 4.0f * (hue - 0.5f);

    return result;
}

void leds_set_frequency(uint16_t freq) {
    const struct rgb_t color = frequency_to_rgb(freq);

    set_rgb_led(0, color.r, color.g, color.b);
    set_rgb_led(1, color.r, color.g, color.b);
    set_rgb_led(2, color.r, color.g, color.b);
    set_rgb_led(3, color.r, color.g, color.b);
}

/*
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    int freq;
    while(scanf("%d\n", &freq) == 1) {
        struct rgb_t res = frequency_to_rgb(freq);
        printf("%d:\tr: %d, g: %d, b: %d\n", freq, res.r, res.g, res.b);
    }

    return 0;
}
*/

/*
struct rgb_map_t {
    uint16_t note;
    struct rgb_t color;
};

static rgb_map_t rgb_mapping[] = {
    { NOTE_B0,   {   0,   0,   0 }, },
    { NOTE_C1,   {   0,   0,   0 }, },
    { NOTE_CS1,  {   0,   0,   0 }, },
    { NOTE_D1,   {   0,   0,   0 }, },
    { NOTE_DS1,  {   0,   0,   0 }, },
    { NOTE_E1,   {   0,   0,   0 }, },
    { NOTE_F1,   {   0,   0,   0 }, },
    { NOTE_FS1,  {   0,   0,   0 }, },
    { NOTE_G1,   {   0,   0,   0 }, },
    { NOTE_GS1,  {   0,   0,   0 }, },
    { NOTE_A1,   {   0,   0,   0 }, },
    { NOTE_AS1,  {   0,   0,   0 }, },
    { NOTE_B1,   {   0,   0,   0 }, },
    { NOTE_C2,   {   0,   0,   0 }, },
    { NOTE_CS2,  {   0,   0,   0 }, },
    { NOTE_D2,   {   0,   0,   0 }, },
    { NOTE_DS2,  {   0,   0,   0 }, },
    { NOTE_E2,   {   0,   0,   0 }, },
    { NOTE_F2,   {   0,   0,   0 }, },
    { NOTE_FS2,  {   0,   0,   0 }, },
    { NOTE_G2,   {   0,   0,   0 }, },
    { NOTE_GS2,  {   0,   0,   0 }, },
    { NOTE_A2,   {   0,   0,   0 }, },
    { NOTE_AS2,  {   0,   0,   0 }, },
    { NOTE_B2,   {   0,   0,   0 }, },
    { NOTE_C3,   {   0,   0,   0 }, },
    { NOTE_CS3,  {   0,   0,   0 }, },
    { NOTE_D3,   {   0,   0,   0 }, },
    { NOTE_DS3,  {   0,   0,   0 }, },
    { NOTE_E3,   {   0,   0,   0 }, },
    { NOTE_F3,   {   0,   0,   0 }, },
    { NOTE_FS3,  {   0,   0,   0 }, },
    { NOTE_G3,   {   0,   0,   0 }, },
    { NOTE_GS3,  {   0,   0,   0 }, },
    { NOTE_A3,   {   0,   0,   0 }, },
    { NOTE_AS3,  {   0,   0,   0 }, },
    { NOTE_B3,   {   0,   0,   0 }, },
    { NOTE_C4,   {   0,   0,   0 }, },
    { NOTE_CS4,  {   0,   0,   0 }, },
    { NOTE_D4,   {   0,   0,   0 }, },
    { NOTE_DS4,  {   0,   0,   0 }, },
    { NOTE_E4,   {   0,   0,   0 }, },
    { NOTE_F4,   {   0,   0,   0 }, },
    { NOTE_FS4,  {   0,   0,   0 }, },
    { NOTE_G4,   {   0,   0,   0 }, },
    { NOTE_GS4,  {   0,   0,   0 }, },
    { NOTE_A4,   {   0,   0,   0 }, },
    { NOTE_AS4,  {   0,   0,   0 }, },
    { NOTE_B4,   {   0,   0,   0 }, },
    { NOTE_C5,   {   0,   0,   0 }, },
    { NOTE_CS5,  {   0,   0,   0 }, },
    { NOTE_D5,   {   0,   0,   0 }, },
    { NOTE_DS5,  {   0,   0,   0 }, },
    { NOTE_E5,   {   0,   0,   0 }, },
    { NOTE_F5,   {   0,   0,   0 }, },
    { NOTE_FS5,  {   0,   0,   0 }, },
    { NOTE_G5,   {   0,   0,   0 }, },
    { NOTE_GS5,  {   0,   0,   0 }, },
    { NOTE_A5,   {   0,   0,   0 }, },
    { NOTE_AS5,  {   0,   0,   0 }, },
    { NOTE_B5,   {   0,   0,   0 }, },
    { NOTE_C6,   {   0,   0,   0 }, },
    { NOTE_CS6,  {   0,   0,   0 }, },
    { NOTE_D6,   {   0,   0,   0 }, },
    { NOTE_DS6,  {   0,   0,   0 }, },
    { NOTE_E6,   {   0,   0,   0 }, },
    { NOTE_F6,   {   0,   0,   0 }, },
    { NOTE_FS6,  {   0,   0,   0 }, },
    { NOTE_G6,   {   0,   0,   0 }, },
    { NOTE_GS6,  {   0,   0,   0 }, },
    { NOTE_A6,   {   0,   0,   0 }, },
    { NOTE_AS6,  {   0,   0,   0 }, },
    { NOTE_B6,   {   0,   0,   0 }, },
    { NOTE_C7,   {   0,   0,   0 }, },
    { NOTE_CS7,  {   0,   0,   0 }, },
    { NOTE_D7,   {   0,   0,   0 }, },
    { NOTE_DS7,  {   0,   0,   0 }, },
    { NOTE_E7,   {   0,   0,   0 }, },
    { NOTE_F7,   {   0,   0,   0 }, },
    { NOTE_FS7,  {   0,   0,   0 }, },
    { NOTE_G7,   {   0,   0,   0 }, },
    { NOTE_GS7,  {   0,   0,   0 }, },
    { NOTE_A7,   {   0,   0,   0 }, },
    { NOTE_AS7,  {   0,   0,   0 }, },
    { NOTE_B7,   {   0,   0,   0 }, },
    { NOTE_C8,   {   0,   0,   0 }, },
    { NOTE_CS8,  {   0,   0,   0 }, },
    { NOTE_D8,   {   0,   0,   0 }, },
    { NOTE_DS8,  {   0,   0,   0 }, },
};
*/
