/* Copyright 2018 Jason Williams (Wilba)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "quantum.h"
#include "wt_mono_backlight.h"
#include "wt_rgb_backlight_api.h" // reuse these for now
#include "wt_rgb_backlight_keycodes.h" // reuse these for now
//#include <math.h>

#include <avr/interrupt.h>
#include "drivers/avr/i2c_master.h"

#include "progmem.h"
#include "quantum/color.h"
#include "tmk_core/common/eeprom.h"

#include "via.h" // uses EEPROM address, lighting value IDs
#define MONO_BACKLIGHT_CONFIG_EEPROM_ADDR (VIA_EEPROM_CUSTOM_CONFIG_ADDR)

#if VIA_EEPROM_CUSTOM_CONFIG_SIZE == 0
#error VIA_EEPROM_CUSTOM_CONFIG_SIZE was not defined to store backlight_config struct
#endif

#include "drivers/issi/is31fl3736.h"

#define ISSI_ADDR_DEFAULT 0x50

#define BACKLIGHT_EFFECT_MAX 10

#ifndef MONO_BACKLIGHT_COLOR_1
#define MONO_BACKLIGHT_COLOR_1 { .h = 0, .s = 255 }
#endif

//SVENGE MODS
#if defined(MONO_BACKLIGHT_WT75_C)
#define BACKLIGHT_LED_COUNT 96
#endif
//END SVENGE MODS


backlight_config g_config = {
    .disable_when_usb_suspended = MONO_BACKLIGHT_DISABLE_WHEN_USB_SUSPENDED,
    .disable_after_timeout = MONO_BACKLIGHT_DISABLE_AFTER_TIMEOUT,
    .brightness = MONO_BACKLIGHT_BRIGHTNESS,
    .effect = MONO_BACKLIGHT_EFFECT,
    .effect_speed = MONO_BACKLIGHT_EFFECT_SPEED,
    .color_1 = MONO_BACKLIGHT_COLOR_1,
    .caps_lock_indicator = RGB_BACKLIGHT_CAPS_LOCK_INDICATOR,
    .layer_1_indicator = RGB_BACKLIGHT_LAYER_1_INDICATOR,
    .layer_2_indicator = RGB_BACKLIGHT_LAYER_2_INDICATOR,
    .layer_3_indicator = RGB_BACKLIGHT_LAYER_3_INDICATOR
};

bool g_suspend_state = false;
uint8_t g_indicator_state = 0;

// Global tick at 20 Hz
uint32_t g_tick = 0;

// Svenge global int
uint8_t sv_led = 0;
uint8_t sv_speed = 0;

// Ticks since this key was last hit -- added
uint8_t g_key_hit[BACKLIGHT_LED_COUNT];

// Ticks since any key was last hit.
uint32_t g_any_key_hit = 0;


//SVENGE MODS
typedef struct Point {
    uint8_t x;
    uint8_t y;
} Point;


// index in range 0..71 (LA0..LA17, LB0..LB17, LC0..LC17, LD0..LD17)
// point values in range x=0..224 y=0..64
// origin is center of top-left key (i.e Esc)
const Point g_map_led_to_point[BACKLIGHT_LED_COUNT] PROGMEM = {
    //0     1           2       3SP       4SP      5?       6?          7?
    {3,61}, {22,61}, {39,61}, {94,61}, {102,61}, {102,61}, {102,61}, {102,61}, 
    //8
    {9,50}, {255,255}, {33,50}, {48,50}, {63,50}, {77,50}, {92,50}, {107,50}, 
    //16
    {5,38}, {25,38}, {41,38}, {56,38}, {70,38}, {84,38}, {99,38}, {113,38}, 
    //24
    {4,27}, {22,27}, {37,27}, {52,27}, {66,27}, {81,27}, {96,27}, {110,27}, 
    //32
    {0,15}, {15,15}, {30,15}, {44,15}, {59,15}, {74,15}, {88,15}, {103,15}, 
    //40
    {0,0}, {19,0}, {33,0}, {48,0}, {63,0}, {81,0},  {96,0}, {109,0}, 
    //48
    {124,0}, {142,0}, {157,0}, {173,0}, {187,0}, {255,255}, {255,255}, {206,0},
    //56
    {117,15}, {132,15}, {147,15}, {161,15}, {176,15}, {198,15}, {224,15}, {255,255}, 
    //64
    {124,27}, {139,27}, {154,27}, {168,27}, {183,27}, {202,27}, {224,27}, {255,255}, 
    //72
    {128,38}, {143,38}, {157,38}, {172,38}, {196,38}, {255,255}, {224,38}, {255,255}, 
    //80
    {121,50}, {135,50}, {150,50}, {165,50}, {185,50}, {255,255}, {255,255}, {255,255}, 
    //88
    {150,61}, {173,61}, {195,64}, {210,64}, {210,53}, {224,64}, {255,255}, {255,255}
};
const Point g_map_led_to_point_polar[BACKLIGHT_LED_COUNT] PROGMEM = {
    //0         1           2       3SP       4SP      5?       6?          7?
    {97,255}, {93,244}, {89,203}, {71,88}, {68,79}, {68,79}, {68,79}, {68,79}, 
    //8
    {105,255}, {255,255}, {100,209}, {96,172}, {91,135}, {84,102}, {76,69}, {67,48}, 
    //16
    {120,255}, {118,225}, {116,184}, {113,145}, {109,110}, {101,74}, {86,37}, {62,16}, 
    //24
    {134,255}, {135,233}, {137,194}, {139,156}, {142,120}, {148,81}, {161,43}, {187,14}, 
    //32
    {147,255}, {150,254}, {153,216}, {157,181}, {162,144}, {168,108}, {176,76}, {185,50},
    //40
    {159,255}, {163,254}, {166,220}, {170,185}, {175,151}, {180,115}, {185,92}, {190,83}, 
    //48
    {196,88}, {202,113}, {207,143}, {211,178}, {215,211}, {255,255}, {255,255}, {220,255},
    //56
    {195,46}, {204,68}, {213,100}, {219,134}, {225,171}, {230,226}, {235,255}, {255,255}, 
    //64
    {216,34}, {232,71}, {239,109}, {243,145}, {245,184}, {247,233}, {249,255}, {255,255}, 
    //72
    {37,44}, {24,82}, {18,117}, {172,38}, {14,156}, {255,255}, {8,255}, {255,255}, 
    //80
    {58,52}, {50,75}, {42,109}, {165,50}, {35,145}, {255,255}, {255,255}, {255,255}, 
    //88
    {49,123}, {42,174}, {38,230}, {35,255}, {26,255}, {32,255}, {255,255}, {255,255}
};

// This may seem counter-intuitive, but it's quite flexible.
// For each LED, get it's position to decide what color to make it.
// This solves the issue of LEDs (and switches) not aligning to a grid,
// or having a large "bitmap" and sampling them.
void map_led_to_point( uint8_t index, Point *point )
{
    // Slightly messy way to get Point structs out of progmem.
    uint8_t *addr = (uint8_t*)&g_map_led_to_point[index];
    point->x = pgm_read_byte(addr);
    point->y = pgm_read_byte(addr+1);

    return;
}
void map_led_to_point_polar( uint8_t index, Point *point )
{
    // Slightly messy way to get Point structs out of progmem.
    uint8_t *addr = (uint8_t*)&g_map_led_to_point_polar[index];
    point->x = pgm_read_byte(addr);
    point->y = pgm_read_byte(addr+1);
}

//
// Maps switch matrix coordinate (row,col) to LED index
//
const uint8_t g_map_row_column_to_led[MATRIX_ROWS][MATRIX_COLS] PROGMEM = {
    { 40,   41,   42,   43,   44,   45,   46,   47,   48,   49,   50,   51,   52,   255,  55,   255  },
    { 32,   33,   34,   35,   36,   37,   38,   39,   56,   57,   58,   59,   60,   61,   61,   62   }, 
    { 24,   25,   26,   27,   28,   29,   30,   31,   64,   65,   66,   67,   68,   69,   255,  70   }, 
    { 16,   17,   18,   19,   20,   21,   22,   23,   72,   73,   74,   75,   76,   255,  255,  78   },
    { 8,    255,  10,   11,   12,   13,   14,   15,   80,   81,   82,   83,   84,   92,   255,  255  },
    { 0,    1,    2,    255,  255,  255,  3,    255,  255,  255,  88,   89,   90,   91,   255,  93   } 
};

void map_row_column_to_led( uint8_t row, uint8_t column, uint8_t *led )
{
    *led = 255;
    if ( row < MATRIX_ROWS && column < MATRIX_COLS )
    {
        *led = pgm_read_byte(&g_map_row_column_to_led[row][column]);
    }
}

void map_led_to_row_column( uint8_t led, uint8_t *row, uint8_t *column )
{
    for (int i=0; i<MATRIX_ROWS; i++) 
    {
        for (int j=0; j<MATRIX_COLS; j++)
        {
            if (g_map_row_column_to_led[i][j] == led)
            {
                *row = i;
                *column = j;
            }
        }
    }
}
//END SVENGE MODS






void backlight_init_drivers(void)
{
	// Initialize I2C
	i2c_init();
	IS31FL3736_init( ISSI_ADDR_DEFAULT );

	for ( uint8_t index = 0; index < 96; index++ )	{
		IS31FL3736_mono_set_led_control_register( index, true );
	}
	IS31FL3736_update_led_control_registers( ISSI_ADDR_DEFAULT, 0x00 );

    // clear the key hits
    for ( int led=0; led<BACKLIGHT_LED_COUNT; led++ )
    {
        g_key_hit[led] = 255;
    }
}

void backlight_set_key_hit(uint8_t row, uint8_t column)
{
    //added led function here - svenge
    uint8_t led;
    map_row_column_to_led(row,column,&led);
    g_key_hit[led] = 0;

    g_any_key_hit = 0;
}

// This is (F_CPU/1024) / 20 Hz
// = 15625 Hz / 20 Hz
// = 781
#define TIMER3_TOP 781

void backlight_timer_init(void)
{
	static uint8_t backlight_timer_is_init = 0;
	if ( backlight_timer_is_init ) {
		return;
	}
	backlight_timer_is_init = 1;

	// Timer 3 setup
	TCCR3B = _BV(WGM32) | 			// CTC mode OCR3A as TOP
			 _BV(CS32) | _BV(CS30); // prescale by /1024
	// Set TOP value
	uint8_t sreg = SREG;
	cli();

	OCR3AH = (TIMER3_TOP >> 8) & 0xff;
	OCR3AL = TIMER3_TOP & 0xff;
	SREG = sreg;
}

void backlight_timer_enable(void)
{
	TIMSK3 |= _BV(OCIE3A);
}

void backlight_timer_disable(void)
{
	TIMSK3 &= ~_BV(OCIE3A);
}

void backlight_set_suspend_state(bool state)
{
	g_suspend_state = state;
}

void backlight_set_indicator_state(uint8_t state)
{
    g_indicator_state = state;
}

void backlight_set_brightness_all( uint8_t value )
{
	IS31FL3736_mono_set_brightness_all( value );
}

void backlight_effect_all_off(void)
{
	IS31FL3736_mono_set_brightness_all( 0 );
}

void backlight_effect_all_on(void)
{
	IS31FL3736_mono_set_brightness_all( g_config.brightness );
}

// SVENGE'S CUSTOM LED AFFECTS
void backlight_effect_indexer(bool initialize)
{
    // Initialize all off and make sure speed var is reset
    IS31FL3736_mono_set_brightness_all( 0 & 0xFF );
    if ( initialize )
    {
        sv_speed = 0;
        sv_led = 0;
    }
	
    // Proc action based on global speed variable
    if ( sv_speed >= (3 - g_config.effect_speed) )
    {
        IS31FL3736_mono_set_brightness( sv_led, 0 & 0xFF );

        if ( sv_led < 95 )
        {
            sv_led++;
        }
        else
        {
            sv_led = 0;
        }
        
        IS31FL3736_mono_set_brightness( sv_led, g_config.brightness );
        sv_speed = 0;
    }
    else
    {
        IS31FL3736_mono_set_brightness( sv_led, g_config.brightness );
        sv_speed++;
    }
}
void backlight_effect_mods(bool initialize)
{
    // Initialize all off
    IS31FL3736_mono_set_brightness_all( 0 & 0xFF );

    for ( int i=0; i<10; i++ )
    {
        IS31FL3736_mono_set_brightness( i, g_config.brightness );
    }
    IS31FL3736_mono_set_brightness( 16, g_config.brightness );
    IS31FL3736_mono_set_brightness( 24, g_config.brightness );
    IS31FL3736_mono_set_brightness( 32, g_config.brightness );
    for ( int i=40; i<56; i++ )
    {
        IS31FL3736_mono_set_brightness( i, g_config.brightness );
    }
    IS31FL3736_mono_set_brightness( 61, g_config.brightness );
    IS31FL3736_mono_set_brightness( 62, g_config.brightness );
    IS31FL3736_mono_set_brightness( 69, g_config.brightness );
    IS31FL3736_mono_set_brightness( 70, g_config.brightness );
    IS31FL3736_mono_set_brightness( 76, g_config.brightness );
    IS31FL3736_mono_set_brightness( 78, g_config.brightness );
    for ( int i=84; i<96; i++ )
    {
        IS31FL3736_mono_set_brightness( i, g_config.brightness );
    }
}

void backlight_effect_rowsplash(void)
{  
    uint8_t rowlight[MATRIX_ROWS];
    uint8_t led;
    uint16_t offset;
    for ( uint8_t j=0; j<MATRIX_ROWS; j++ )
    {
        rowlight[j] = 0;
        for ( uint8_t i=0; i<MATRIX_COLS; i++ )
        {
            map_row_column_to_led(j,i,&led);
            offset = (g_key_hit[led] * (6 + g_config.effect_speed*4));
            if (offset > g_config.brightness)
            {
                offset = g_config.brightness;
            }
            if (rowlight[j] < ( g_config.brightness - (offset & 0xFF) ))
            {
                rowlight[j] = ( g_config.brightness - (offset & 0xFF) );
            }
        }
        for ( uint8_t i=0; i<MATRIX_COLS; i++ )
        {
            map_row_column_to_led(j,i,&led);
            IS31FL3736_mono_set_brightness(led, rowlight[j] );
        }
        
    }
}

void backlight_effect_crossplash(void)
{  
    uint8_t rowlight[MATRIX_ROWS];
    uint8_t colight[MATRIX_COLS];
    uint8_t led;
    uint16_t offset;
    uint8_t brt;
    for ( uint8_t j=0; j<MATRIX_ROWS; j++ ) {
        for ( uint8_t i=0; i<MATRIX_COLS; i++ ) {
        rowlight[j] = 0;
        colight[i] = 0;
        }
    }
    for ( uint8_t j=0; j<MATRIX_ROWS; j++ )
    {
        for ( uint8_t i=0; i<MATRIX_COLS; i++ )
        {
            map_row_column_to_led(j,i,&led);
            offset = (g_key_hit[led] * (6 + g_config.effect_speed*4));
            if (offset > g_config.brightness)
            {
                offset = g_config.brightness;
            }
            if (rowlight[j] < ( g_config.brightness - (offset & 0xFF) ))
            {
                rowlight[j] = ( g_config.brightness - (offset & 0xFF) );
            }
            if (colight[i] < ( g_config.brightness - (offset & 0xFF) ))
            {
                colight[i] = ( g_config.brightness - (offset & 0xFF) );
            }
        }
    }
    for ( uint8_t j=0; j<MATRIX_ROWS; j++ ) {
        for ( uint8_t i=0; i<MATRIX_COLS; i++ ) {
        brt = rowlight[j] > colight[i] ? rowlight[j] : colight[i];
            map_row_column_to_led(j,i,&led);
            IS31FL3736_mono_set_brightness(led, brt);
        }
    }
}


void backlight_effect_raindrops(bool initialize)
{
    // Change one LED every tick
    uint8_t led_to_change = ( g_tick & 0x000 ) == 0 ? rand() % 96 : 255;

    for ( int i=0; i<96; i++ )
    {
        // If initialize, all get set to random brightness
        // If not, all but one will stay the same as before.
        if ( initialize || i == led_to_change )
        {
            IS31FL3736_mono_set_brightness(i, rand() & 0xFF );
        }
    }
}

void backlight_effect_cycle_all(void)
{
	uint8_t offset = ( g_tick << g_config.effect_speed ) & 0xFF;

	backlight_set_brightness_all( offset );
}

// here starts the fancy ones imported from RGB
void backlight_effect_cycle_left_right(void)
{
    //point.x scales from 0 to 224
    //uint32_t g_tick max int value is 0xFFFFFFFF
    //uint8_t offset = ( g_tick << g_config.effect_speed ) & 0xFF;
    
    Point point;
    uint8_t brt;
    uint8_t offset = ( g_tick << g_config.effect_speed ) & 0xFF;
    uint8_t offset2 = 30;
    
    
    for ( int i=0; i<BACKLIGHT_LED_COUNT; i++ )
    {
        map_led_to_point( i, &point );
        if ( (point.x >= (offset - offset2)) && (point.x <= (offset + offset2)))
        {
            if ( g_config.brightness > abs(point.x - offset) * (255 / offset2)  )
            {
                brt = g_config.brightness - abs(point.x - offset) * (255 / offset2);
            }
            else
            {
                brt = 0;
            }
        }
        else 
        {
            brt = 0;
        }
        IS31FL3736_mono_set_brightness( i, brt );
    }
}


void backlight_effect_keyfade(void)
{
    for ( int i=0; i<BACKLIGHT_LED_COUNT; i++ )
    {
        uint16_t offset = (g_key_hit[i] * (1 + g_config.effect_speed));
        if (offset > g_config.brightness)
        {
            offset = g_config.brightness;
        }
        IS31FL3736_mono_set_brightness( i, g_config.brightness - (offset & 0xFF) );
    }
}




void backlight_effect_indicators_set_colors( uint8_t index, uint8_t ind_brite )
{
    if ( index == 254 )
    {
        backlight_set_brightness_all( ind_brite );
    }
    else
    {
        IS31FL3736_mono_set_brightness( index, ind_brite );

        // If the spacebar LED is the indicator,
        // do the same for the other spacebars
        if ( index == 3 ) 
        {
            IS31FL3736_mono_set_brightness( index+1, ind_brite );
            IS31FL3736_mono_set_brightness( index+2, ind_brite );
            IS31FL3736_mono_set_brightness( index+3, ind_brite );
        }
    }
}


// This runs after another backlight effect and replaces
// colors already set
void backlight_effect_indicators(void)
{
if ( g_config.caps_lock_indicator.index != 255 &&
            ( g_indicator_state & (1<<USB_LED_CAPS_LOCK) ) )
    {
        backlight_effect_indicators_set_colors( g_config.caps_lock_indicator.index, 255 );
    }
    else
    {
        backlight_effect_indicators_set_colors( g_config.caps_lock_indicator.index, 0 );
    }
    // This if/else if structure allows higher layers to
    // override lower ones. If we set layer 3's indicator
    // to none, then it will NOT show layer 2 or layer 1
    // indicators, even if those layers are on via the
    // MO13/MO23 Fn combo magic.
    //
    // Basically we want to handle the case where layer 3 is
    // still the backlight configuration layer and we don't
    // want "all LEDs" indicators hiding the backlight effect,
    // but still allow end users to do whatever they want.
    if ( IS_LAYER_ON(3) )
    {
        if ( g_config.layer_3_indicator.index != 255 )
        {
            backlight_effect_indicators_set_colors( g_config.layer_3_indicator.index, 255 );
        }
    }
    else if ( IS_LAYER_ON(2) )
    {
        if ( g_config.layer_2_indicator.index != 255 )
        {
            backlight_effect_indicators_set_colors( g_config.layer_2_indicator.index, 255 );
        }
    }
    else if ( IS_LAYER_ON(1) )
    {
        if ( g_config.layer_1_indicator.index != 255 )
        {
            backlight_effect_indicators_set_colors( g_config.layer_1_indicator.index, 255 );
        }
    }
}

ISR(TIMER3_COMPA_vect)
{
	// delay 1 second before driving LEDs or doing anything else
	static uint8_t startup_tick = 0;
	if ( startup_tick < 20 ) {
		startup_tick++;
		return;
	}

	g_tick++;

    if ( g_any_key_hit < 0xFFFFFFFF )
    {
        g_any_key_hit++;
    }

    for ( int led = 0; led < BACKLIGHT_LED_COUNT; led++ )
    {
        if ( g_key_hit[led] < 255 )
        {
            g_key_hit[led]++;
        }
    }

    // Ideally we would also stop sending zeros to the LED driver PWM buffers
    // while suspended and just do a software shutdown. This is a cheap hack for now.
    bool suspend_backlight = ((g_suspend_state && g_config.disable_when_usb_suspended) ||
            (g_config.disable_after_timeout > 0 && g_any_key_hit > g_config.disable_after_timeout * 60 * 20));
    uint8_t effect = suspend_backlight ? 0 : g_config.effect;

    // Keep track of the effect used last time,
    // detect change in effect, so each effect can
    // have an optional initialization.
    static uint8_t effect_last = 255;
    bool initialize = effect != effect_last;
    effect_last = effect;

    // this gets ticked at 20 Hz.
    // each effect can opt to do calculations
    // and/or request PWM buffer updates.
    switch ( effect )
    {
        case 0:
            backlight_effect_all_off();
            break;
        case 1:
            backlight_effect_all_on();
            break;
        case 2:
            backlight_effect_raindrops(initialize);
            break;
        case 3:
            backlight_effect_indexer(initialize);
            break;
        case 4:
            backlight_effect_mods(initialize);
            break;
        case 5:
            backlight_effect_cycle_all();
            break;
        case 6:
            backlight_effect_rowsplash();
            break;
        case 7:
            backlight_effect_crossplash();
            break;
        case 8:
            backlight_effect_cycle_left_right();
            break;
        case 9:
            backlight_effect_keyfade();
            break;
        default:
            backlight_effect_all_off();
            break;
	}

    if ( ! suspend_backlight )
    {
        backlight_effect_indicators();
    }
}

// Some helpers for setting/getting HSV
void _set_color( HS *color, uint8_t *data )
{
    color->h = data[0];
    color->s = data[1];
}

void _get_color( HS *color, uint8_t *data )
{
    data[0] = color->h;
    data[1] = color->s;
}

void backlight_config_set_value( uint8_t *data )
{
    bool reinitialize = false;
    uint8_t *value_id = &(data[0]);
    uint8_t *value_data = &(data[1]);
    switch ( *value_id )
    {
        case id_disable_when_usb_suspended:
        {
            g_config.disable_when_usb_suspended = (bool)*value_data;
            break;
        }
        case id_disable_after_timeout:
        {
            g_config.disable_after_timeout = *value_data;
            break;
        }
        case id_brightness:
        {
            g_config.brightness = *value_data;
            break;
        }
        case id_effect:
        {
            g_config.effect = *value_data;
            break;
        }
        case id_effect_speed:
        {
            g_config.effect_speed = *value_data;
            break;
        }
        case id_color_1:
        {
            _set_color( &(g_config.color_1), value_data );
            break;
        }
    }

    if ( reinitialize )
    {
        backlight_init_drivers();
    }
}

void backlight_config_get_value( uint8_t *data )
{
    uint8_t *value_id = &(data[0]);
    uint8_t *value_data = &(data[1]);
    switch ( *value_id )
    {
        case id_disable_when_usb_suspended:
        {
            *value_data = ( g_config.disable_when_usb_suspended ? 1 : 0 );
            break;
        }
        case id_disable_after_timeout:
        {
            *value_data = g_config.disable_after_timeout;
            break;
        }
        case id_brightness:
        {
            *value_data = g_config.brightness;
            break;
        }
        case id_effect:
        {
            *value_data = g_config.effect;
            break;
        }
        case id_effect_speed:
        {
            *value_data = g_config.effect_speed;
            break;
        }
        case id_color_1:
        {
            _get_color( &(g_config.color_1), value_data );
            break;
        }
    }
}

void backlight_config_load(void)
{
    eeprom_read_block( &g_config, ((void*)MONO_BACKLIGHT_CONFIG_EEPROM_ADDR), sizeof(backlight_config) );
}

void backlight_config_save(void)
{
    eeprom_update_block( &g_config, ((void*)MONO_BACKLIGHT_CONFIG_EEPROM_ADDR), sizeof(backlight_config) );
}

void backlight_update_pwm_buffers(void)
{
	IS31FL3736_update_pwm_buffers(ISSI_ADDR_DEFAULT,0x00);
}

bool process_record_backlight(uint16_t keycode, keyrecord_t *record)
{
    // Record keypresses for backlight effects
    if ( record->event.pressed )
    {
        backlight_set_key_hit( record->event.key.row, record->event.key.col );
    }

    switch(keycode)
    {
        case BR_INC:
            if (record->event.pressed)
            {
                backlight_brightness_increase();
            }
            return false;
            break;
        case BR_DEC:
            if (record->event.pressed)
            {
                backlight_brightness_decrease();
            }
            return false;
            break;
        case EF_INC:
            if (record->event.pressed)
            {
                backlight_effect_increase();
            }
            return false;
            break;
        case EF_DEC:
            if (record->event.pressed)
            {
                backlight_effect_decrease();
            }
            return false;
            break;
        case ES_INC:
            if (record->event.pressed)
            {
                backlight_effect_speed_increase();
            }
            return false;
            break;
        case ES_DEC:
            if (record->event.pressed)
            {
                backlight_effect_speed_decrease();
            }
            return false;
            break;
    }

    return true;
}

// Deals with the messy details of incrementing an integer
uint8_t increment( uint8_t value, uint8_t step, uint8_t min, uint8_t max )
{
    int16_t new_value = value;
    new_value += step;
    return MIN( MAX( new_value, min ), max );
}

uint8_t decrement( uint8_t value, uint8_t step, uint8_t min, uint8_t max )
{
    int16_t new_value = value;
    new_value -= step;
    return MIN( MAX( new_value, min ), max );
}

void backlight_effect_increase(void)
{
    g_config.effect = increment( g_config.effect, 1, 0, BACKLIGHT_EFFECT_MAX );
    backlight_config_save();
}

void backlight_effect_decrease(void)
{
    g_config.effect = decrement( g_config.effect, 1, 0, BACKLIGHT_EFFECT_MAX );
    backlight_config_save();
}

void backlight_effect_speed_increase(void)
{
    g_config.effect_speed = increment( g_config.effect_speed, 1, 0, 3 );
    backlight_config_save();
}

void backlight_effect_speed_decrease(void)
{
    g_config.effect_speed = decrement( g_config.effect_speed, 1, 0, 3 );
    backlight_config_save();
}

void backlight_brightness_increase(void)
{
    g_config.brightness = increment( g_config.brightness, 8, 0, 255 );
    backlight_config_save();
}

void backlight_brightness_decrease(void)
{
    g_config.brightness = decrement( g_config.brightness, 8, 0, 255 );
    backlight_config_save();
}
