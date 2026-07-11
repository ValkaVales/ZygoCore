#pragma once

#include <zygo/core/assert.h>
#include <cstddef>


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
  // Rule of five.
  // Copy is plain member-wise.
  // Move resets the source to an empty buffer, so that a moved-from derived object (whose data vector is now empty) stays consistent: count == 0, isEmpty() == true.
  // Because first/count live here, both derived classes can rely on the compiler's implicit copy/move (rule of zero) and still get correct move semantics.
  CircularBufferBase           ( CircularBufferBase const& ) = default;
  CircularBufferBase& operator=( CircularBufferBase const& ) = default;

  CircularBufferBase( CircularBufferBase&& other ) noexcept
    : capacity0 ( other.capacity0 )
    , first     ( other.first )
    , count     ( other.count )
  {
    other.first = 0;
    other.count = 0;
  }

  CircularBufferBase& operator=( CircularBufferBase&& other ) noexcept
  {
    capacity0 = other.capacity0;
    first     = other.first;
    count     = other.count;

    other.first = 0;
    other.count = 0;
    return *this;
  }

  //
  [[nodiscard]] inline SizeType capacity() const noexcept { return capacity0; } // totalSize
  [[nodiscard]] inline SizeType size    () const noexcept { return count; } // curCount
  [[nodiscard]] inline bool     isEmpty () const noexcept { return count == 0; }
  [[nodiscard]] inline bool     isFull  () const noexcept { return count == capacity0; }

protected:
  explicit CircularBufferBase( SizeType capacity )
    : capacity0 ( capacity )
    , first     ( 0 )
    , count     ( 0 )
  {
    ZgAssertRelease( capacity > 0 );
  }

  // Non-virtual destructor: this base carries no resources and is never deleted through a base pointer.
  // Protected so no one can do that by accident.
  ~CircularBufferBase() = default;

  //
  inline SizeType getFirst() const noexcept { return first; }
  inline SizeType getCount() const noexcept { return count; }

  inline void clearBase() noexcept
  {
    first = 0;
    count = 0;
  }

  inline void elemPoped() noexcept
  {
    moveFirstToNext();
    if ( --count == 0 )
      first = 0;
  }

  // Two-phase push for exception safety:
  //   index = curPhysIndex();  construct the element;  commitPush();
  // count is bumped only after the (possibly throwing) construction succeeds.
  inline SizeType elemPushed() noexcept // returns phys index of pushed elem
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
