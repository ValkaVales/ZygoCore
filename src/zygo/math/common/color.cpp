#include "color.h"
#include "scalar.h"


namespace zygo {

void hsvToRGB( Real h, Real s, Real v, Real &r, Real &g, Real &b ) // h is in degrees (0...360),  other values are in (0...1)
{
  if ( s <= 0.0 )
  {
    r = v;
    g = v;
    b = v;
    return;
  }

  Real hh = h;
  if ( hh >= 360.0 )
    hh = 0.0;

  hh /= 60.0;
  int i = (int)hh;

  Real ff = hh - i; // == fmod( h, 60.0 )

  Real p = v * (1.0 - s);
  Real q = v * (1.0 - (s * ff));
  Real t = v * (1.0 - (s * (1.0 - ff)));

  switch ( i )
  {
  case 0:
    r = v;
    g = t;
    b = p;
    break;
  case 1:
    r = q;
    g = v;
    b = p;
    break;
  case 2:
    r = p;
    g = v;
    b = t;
    break;

  case 3:
    r = p;
    g = q;
    b = v;
    break;
  case 4:
    r = t;
    g = p;
    b = v;
    break;
  case 5:
  default:
    r = v;
    g = p;
    b = q;
    break;
  }
}

void rgbToHSV( Real r, Real g, Real b, Real& h, Real& s, Real &v )
{
  Real min_color_component = r < g ? r : g;
  min_color_component = min_color_component < b ? min_color_component : b;

  Real max_color_component = r > g ? r : g;
  max_color_component = max_color_component > b ? max_color_component : b;

  v = max_color_component;
  Real delta = max_color_component - min_color_component;
  if ( delta < EPSILON )
  {
    s = 0.0;
    h = 0.0; // undefined
    return;
  }

  if ( isZero( max_color_component ) )
  {
    // if max_color_component is 0, then r = g = b = 0
    s = 0.0;
    h = 0.0; // undefined
    return;
  }

  s = (delta / max_color_component);

  if ( r >= max_color_component )
    h = (g - b) / delta;          // between yellow & magenta
  else
  if ( g >= max_color_component )
    h = 2.0 + (b - r) / delta;    // between cyan & yellow
  else
    h = 4.0 + (r - g) / delta;    // between magenta & cyan

  h *= 60.0; // in degrees

  if ( h < 0.0 )
    h += 360.0;
}

uint hsvToColor( Real h, Real s, Real v ) // h is in degrees (0...360),  s and v are in (0...1)
{
  Real r;
  Real g;
  Real b;

  hsvToRGB( h, s, v, r, g, b );

  return RGBD2COL( r, g, b );
}

void colorToHSV( uint color, Real& h, Real& s, Real& v )
{
  uint b = color & 0xff;    color >>= 8;
  uint g = color & 0xff;    color >>= 8;
  uint r = color & 0xff;

  rgbToHSV( 
    (Real)r / 255.0,
    (Real)g / 255.0,
    (Real)b / 255.0,
    h,
    s,
    v
  );
}

uint hueToColor( Real value )
{
  return hsvToColor( 360.0 * (1.0 - value), 1.0, 1.0 );
}

uint makeColorLighter( uint color )
{
  Real h;
  Real s;
  Real v;

  colorToHSV( color, h, s, v );
  return hsvToColor( h, s * 0.5, v );
}

} // namespace zygo
