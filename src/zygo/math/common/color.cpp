#include "color.h"
#include "scalar.h"


namespace zygo {

void hsvToRGB( Real h, Real s, Real v, Real &r, Real &g, Real &b ) // h is in degrees (0...360),  other values are in (0...1)
{
  if ( s <= REAL_ZERO )
  {
    r = v;
    g = v;
    b = v;
    return;
  }

  Real hh = std::fmod( h, DEG_PI_MUL_2 );
  if ( hh < 0 )
    hh += DEG_PI_MUL_2;

  hh /= Real(60.0);
  int i = (int)hh;

  Real ff = hh - i; // == fmod( h, 60 )

  Real p = v * (REAL_ONE - s);
  Real q = v * (REAL_ONE - (s * ff));
  Real t = v * (REAL_ONE - (s * (REAL_ONE - ff)));

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
    s = REAL_ZERO;
    h = REAL_ZERO; // undefined
    return;
  }

  if ( isZero( max_color_component ) )
  {
    // if max_color_component is 0, then r = g = b = 0
    s = REAL_ZERO;
    h = REAL_ZERO; // undefined
    return;
  }

  s = (delta / max_color_component);

  if ( r >= max_color_component )
    h = (g - b) / delta;          // between yellow & magenta
  else
  if ( g >= max_color_component )
    h = Real(2.0) + (b - r) / delta;    // between cyan & yellow
  else
    h = Real(4.0) + (r - g) / delta;    // between magenta & cyan

  h *= Real(60.0); // in degrees

  if ( h < REAL_ZERO )
    h += DEG_PI_MUL_2;
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
    (Real)r / Real(255.0),
    (Real)g / Real(255.0),
    (Real)b / Real(255.0),
    h,
    s,
    v
  );
}

uint hueToColor( Real value )
{
  return hsvToColor( DEG_PI_MUL_2 * (REAL_ONE - value), REAL_ONE, REAL_ONE );
}

uint makeColorLighter( uint color )
{
  Real h;
  Real s;
  Real v;

  colorToHSV( color, h, s, v );
  return hsvToColor( h, s * REAL_HALF, v );
}

} // namespace zygo
