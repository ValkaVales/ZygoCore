#include "circular_buffer.h"
#include <zygo/core/assert.h>
#include <memory.h>


namespace zygo {

CircularBufferBase::CircularBufferBase( int size )
  : total_size  ( size )
  , i1          ( 0 )
  , i2          ( 0 )
  , count       ( 0 )
{
}

void CircularBufferBase::elemPushed()
{
  i2 = (i2 + 1) % total_size;

  if ( count++ >= total_size )
  {
    count = total_size;
    i1 = i2;
  }
}

void CircularBufferBase::elemPoped()
{
  ZgAssert( count > 0 );

  i1 = (i1 + 1) % total_size;
  --count;
}


// ===================================================================================================== CircularBuffer, for integral types
template<typename T>
CircularBuffer<T>::CircularBuffer( int size )// , bool auto_delete, bool calc_total_sum )
  : CircularBufferBase ( size )
  , data            ( NULL )
  , total_sum       ( 0 )
{
  data = new T[size];
  memset( data, 0, sizeof( T ) * size );
}

template<typename T>
CircularBuffer<T>::~CircularBuffer()
{
  delete[] data;
}

template<typename T>
void CircularBuffer<T>::push( T elem )
{
  if ( isFull() )
  {
    T oldest = data[startIdx()];
    total_sum -= oldest;
  }

  data[endIdx()] = elem;
  total_sum += elem;

  elemPushed();
}

template<typename T>
T CircularBuffer<T>::pop()
{
  ZgAssert( !isEmpty() );

  T res = data[startIdx()];
  total_sum -= res;

  elemPoped();
  return res;
}

template<typename T>
T CircularBuffer<T>::get( int idx ) const
{
  ZgAssert( idx >= 0 && idx < curCount() );

  idx += startIdx();
  idx %= totalSize();

  return data[idx];
}

// ===================================================================================================== CircularBufferForObj, for objects
template<class T>
CircularBufferForObj<T>::CircularBufferForObj( int size, bool auto_delete )
  : CircularBufferBase ( size )
  , data            ( NULL )
  , auto_delete     ( auto_delete )
{
  data = new T*[size];
  memset( data, 0, sizeof( T* ) * size );
}

template<class T>
CircularBufferForObj<T>::~CircularBufferForObj()
{
  if ( auto_delete )
    deleteAll();

  delete[] data;
}

template<class T>
void CircularBufferForObj<T>::deleteAll()
{
  for ( int i = 0; i < totalSize(); ++i )
  {
    T* elem = data[i];
    delete elem;
  }
}

template<class T>
void CircularBufferForObj<T>::push( T * elem )
{
  if ( isFull() )
  {
    if ( auto_delete )
    {
      T* oldest = data[startIdx()];
      delete oldest;
    }
  }

  data[endIdx()] = elem;
  elemPushed();
}

template<class T>
T * CircularBufferForObj<T>::pop()
{
  ZgAssert( !isEmpty() );

  T * res = data[startIdx()];

  elemPoped();
  return res;
}

template<class T>
T * CircularBufferForObj<T>::get( int idx )
{
  ZgAssert( idx >= 0 && idx < curCount() );

  idx += startIdx();
  idx %= totalSize();

  return data[idx];
}

template<class T>
T const * CircularBufferForObj<T>::get( int idx ) const
{
  ZgAssert( idx >= 0 && idx < curCount() );

  idx += startIdx();
  idx %= totalSize();

  return data[idx];
}

} // namespace zygo
