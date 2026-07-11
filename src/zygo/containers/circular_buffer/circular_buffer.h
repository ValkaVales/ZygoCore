#pragma once

#include "circular_buffer_base.h"
#include <zygo/core/assert.h>
#include <cstdint>
#include <type_traits>
#include <vector>


namespace zygo {

template<typename T>
class CircularBuffer : public CircularBufferBase
{
  static_assert(  std::is_arithmetic_v<T>                     , "CircularBuffer<T> supports arithmetic types only" );
  static_assert( !std::is_same_v<std::remove_cv_t<T>, bool>   , "CircularBuffer<bool> is not supported" );
  static_assert( !std::is_const_v<T> && !std::is_volatile_v<T>, "CircularBuffer<T> requires a non-cv-qualified type" );

public:
  using ValueType = T;

  // Accumulator wide enough to avoid overflow (integers) and to limit drift in the incremental sum (floating point promotes float -> double).
  using SumType =
    std::conditional_t<std::is_floating_point_v<T>,
    std::conditional_t<(sizeof(T) > sizeof(double)), T, double>,
    std::conditional_t<std::is_signed_v<T>, std::int64_t, std::uint64_t>
    >;

private:
  std::vector<T> data;
  SumType total_sum;

public:
  explicit CircularBuffer( SizeType capacity )
    : CircularBufferBase( capacity )
    , data      ( capacity )
    , total_sum ( SumType(0) )
  {
    ZgAssertRelease( capacity > 0 );
  }

  void clear() noexcept
  {
    clearBase();
    total_sum = SumType(0);
  }

  // Adds a new value.
  // If the buffer is full, the oldest value is overwritten.
  void push( T value ) noexcept
  {
    if ( isFull() )
    {
      total_sum -= static_cast<SumType>( data[getFirst()] );
      elemPoped();
    }

    const SizeType index = elemPushed();

    data[index] = value;
    total_sum += static_cast<SumType>( value );
  }


  [[nodiscard]]
  T pop() noexcept
  {
    ZgAssertRelease( !isEmpty() );

    const T result = data[getFirst()];
    total_sum -= static_cast<SumType>( result );

    elemPoped();
    return result;
  }


  bool tryPop( T& result ) noexcept
  {
    if ( isEmpty() )
      return false;

    result = pop();
    return true;
  }


  [[nodiscard]] T front() const noexcept {  ZgAssert( !isEmpty() );   return data[getFirst()];      }
  [[nodiscard]] T back () const noexcept {  ZgAssert( !isEmpty() );   return data[prevPhysIndex()]; }

  [[nodiscard]] T get( SizeType index ) const noexcept { ZgAssert( index < getCount() );    return data[physicalIndex( index )]; }
  [[nodiscard]] T operator[]( SizeType index ) const noexcept {  return get( index );  }

  // Changes an existing element while keeping total_sum correct.
  void set( SizeType index, T value ) noexcept
  {
    ZgAssertRelease( index < getCount() );
    const SizeType physical_index = physicalIndex( index );

    total_sum -= static_cast<SumType>( data[physical_index] );
    total_sum += static_cast<SumType>( value );

    data[physical_index] = value;
  }

  [[nodiscard]]
  SumType totalSum() const noexcept
  {
    return total_sum;
  }

  [[nodiscard]]
  double getAverage() const noexcept
  {
    ZgAssert( !isEmpty() );
    return static_cast<double>( total_sum ) / static_cast<double>( getCount() );
  }
};

} // namespace zygo
