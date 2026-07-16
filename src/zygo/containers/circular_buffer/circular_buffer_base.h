#pragma once

#include <zygo/core/assert.h>
#include <cstddef>
#include <utility>


namespace zygo {

class CircularBufferBase
{
public:
  using SizeType = std::size_t;

private:
  SizeType capacity0; // the same as data.size()
  SizeType first;
  SizeType count;

public:
  [[nodiscard]] inline SizeType capacity() const noexcept { return capacity0; } // totalSize
  [[nodiscard]] inline SizeType size    () const noexcept { return count; } // curCount
  [[nodiscard]] inline bool     isEmpty () const noexcept { return count == 0; }
  [[nodiscard]] inline bool     isFull  () const noexcept { return count == capacity0 && capacity0 != 0; }   // A moved-from buffer has capacity == 0 and count == 0. It is empty, but it is not considered full.

protected:
  explicit CircularBufferBase( SizeType capacity )
    : capacity0 ( capacity )
    , first     ( 0 )
    , count     ( 0 )
  {
    ZgAssertRelease( capacity > 0 );
  }

  // Copy preserves the exact ring state.
  CircularBufferBase           ( CircularBufferBase const& ) = default;
  CircularBufferBase& operator=( CircularBufferBase const& ) = default;

  // Move transfers the ring state and leaves the source as a valid empty, zero-capacity object.
  // The derived class must also move its storage and reset any cached state.
  CircularBufferBase( CircularBufferBase&& other ) noexcept
    : capacity0 ( std::exchange( other.capacity0, SizeType(0) ) )
    , first     ( std::exchange( other.first    , SizeType(0) ) )
    , count     ( std::exchange( other.count    , SizeType(0) ) )
  {
    other.first = 0;
    other.count = 0;
  }

  CircularBufferBase& operator=( CircularBufferBase&& other ) noexcept
  {
    if ( this == &other )
      return *this;

    capacity0 = std::exchange( other.capacity0, SizeType(0) );
    first     = std::exchange( other.first    , SizeType(0) );
    count     = std::exchange( other.count    , SizeType(0) );

    return *this;
  }

  // Non-virtual destructor: this base carries no resources and is never deleted through a base pointer.
  // Protected to prevent accidental deletion.
  ~CircularBufferBase() = default;

  //
  inline SizeType getFirst() const noexcept { return first; }
  inline SizeType getCount() const noexcept { return count; }

  inline void clearBase() noexcept
  {
    first = 0;
    count = 0;
  }

  inline void elemPopped() noexcept
  {
    moveFirstToNext();
    if ( --count == 0 )
      first = 0;
  }

  // Two-phase push for exception safety:
  //   index = curPhysIndex();  construct the element;  commitPush();
  // count is bumped only after the (possibly throwing) construction succeeds.
  inline SizeType elemPushed() noexcept // Returns the physical index of the pushed element.
  {
    SizeType res = curPhysIndex();
    commitPush();
    return res;
  }

  inline void commitPush() noexcept { ++count; }

  inline SizeType curPhysIndex () const noexcept { return physicalIndex( count ); }
  inline SizeType prevPhysIndex() const noexcept { return physicalIndex( count - 1 ); }

  inline SizeType physicalIndex( SizeType logical_index ) const noexcept
  {
    return (first + logical_index) % capacity0;
  }

private:
  inline void moveFirstToNext() noexcept
  {
    first = nextIndex( first );
  }

  inline SizeType nextIndex( SizeType index ) const noexcept
  {
    ++index;
    return index == capacity0 ? 0 : index;
  }
};

} // namespace zygo
