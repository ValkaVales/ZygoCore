#pragma once

#include <zygo/core/types.h>


namespace zygo {

#define RGB2COL(r, g, b) ((((r)&0xff) << 16) + (((g)&0xff) << 8) + ((b)&0xff))
#define RGBD2COL(r, g, b) RGB2COL( (int)((r)*0xff), (int)((g)*0xff), (int)((b)*0xff) )

#define RED(color)   ((float)( color      & 0xff) / 255.0f)
#define GREEN(color) ((float)((color>>8 ) & 0xff) / 255.0f)
#define BLUE(color)  ((float)((color>>16) & 0xff) / 255.0f)


void hsvToRGB( Real h, Real s, Real v, Real &r, Real &g, Real &b ); // h is in degrees (0...360),  s and v are in (0...1)
void rgbToHSV( Real r, Real g, Real b, Real& h, Real& s, Real &v );

uint hsvToColor( Real h, Real s, Real v ); // h is in degrees (0...360),  s and v are in (0...1)
void colorToHSV( uint color, Real& h, Real& s, Real& v );

uint hueToColor( Real value ); // 'value' should be from 0 to 1

uint makeColorLighter( uint color );

} // namespace zygo
