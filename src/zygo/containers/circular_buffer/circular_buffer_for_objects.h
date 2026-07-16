#pragma once

#include "circular_buffer_base.h"
#include <zygo/core/assert.h>

#include <optional>
#include <utility>
#include <vector>


namespace zygo {

// Fixed-capacity ring buffer for arbitrary (movable) object types.
// Slots are std::optional<T>, so unused slots hold no constructed object.
// When full, push()/emplace() destroy the oldest element first.
//
// Copy and move come from the compiler (rule of zero): first/count live in the base, whose move resets the source, so a moved-from buffer is a valid empty buffer.
// Works with move-only T (e.g. std::unique_ptr).
template<class T>
class CircularBufferForObjects : public CircularBufferBase
{
public:
  using ValueType = T;

private:
  std::vector<std::optional<T>> data;

public:
  explicit CircularBufferForObjects( SizeType capacity )
    : CircularBufferBase( capacity )
    , data  ( capacity )
  {
    ZgAssertRelease( capacity > 0 );
  }

  // Rule of five
  // Copy operations are usable when T itself is copyable.
  ~CircularBufferForObjects() = default;

  CircularBufferForObjects            ( CircularBufferForObjects const& ) = default;
  CircularBufferForObjects& operator= ( CircularBufferForObjects const& ) = default;

  CircularBufferForObjects( CircularBufferForObjects&& other ) noexcept
    : CircularBufferBase( std::move(other) )
    , data( std::move(other.data) )
  {
    // Keep the moved-from object fully consistent with capacity() == 0.
    other.data.clear();
  }

  CircularBufferForObjects& operator=( CircularBufferForObjects&& other ) noexcept
  {
    if ( this == &other )
      return *this;

    CircularBufferBase::operator= ( std::move(other) );

    data = std::move( other.data );
    other.data.clear();
    return *this;
  }

  //
  void clear() noexcept
  {
    for ( SizeType i = 0; i < getCount(); ++i )
      data[physicalIndex(i)].reset();

    clearBase();
  }

  void push( T const& value ) { emplace( value ); }
  void push( T     && value ) { emplace( std::move( value ) ); }

  template<class... Args>
  T& emplace( Args&&... args )
  {
    ZgAssertRelease( !data.empty() );

    // When the buffer is full, drop the oldest element first.
    if ( isFull() )
      removeFront();

    // Construct BEFORE bumping count: if the constructor throws, the buffer is left exactly as it was (minus the oldest element, already dropped).
    const SizeType index = curPhysIndex();
    data[index].emplace( std::forward<Args>( args )... );
    commitPush();

    return *data[index];
  }

  T pop()
  {
    ZgAssertRelease( !isEmpty() );
    T result( std::move( *data[getFirst()] ) );
    removeFront();
    return result;
  }

  std::optional<T> tryPop()
  {
    if ( isEmpty() )
      return std::nullopt;

    std::optional<T> result(
      std::in_place,
      std::move( *data[getFirst()] )
    );

    removeFront();
    return result;
  }

  //
  T      & front()        { ZgAssert( !isEmpty() );   return *data[getFirst()];       }
  T const& front() const  { ZgAssert( !isEmpty() );   return *data[getFirst()];       }
  T      & back ()        { ZgAssert( !isEmpty() );   return *data[prevPhysIndex()];  }
  T const& back () const  { ZgAssert( !isEmpty() );   return *data[prevPhysIndex()];  }

  T      & get( SizeType index )        { ZgAssert( index < getCount() );   return *data[physicalIndex( index )]; }
  T const& get( SizeType index ) const  { ZgAssert( index < getCount() );   return *data[physicalIndex( index )]; }

  T      & operator[]( SizeType index )       { return get( index ); }
  T const& operator[]( SizeType index ) const { return get( index ); }

private:
  void removeFront() noexcept
  {
    ZgAssertRelease( !isEmpty() );

    data[getFirst()].reset();
    elemPopped();
  }
};

} // namespace zygo
