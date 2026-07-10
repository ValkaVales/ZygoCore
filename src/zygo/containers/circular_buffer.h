#pragma once


namespace zygo {

class CircularBufferBase
{
private:
  int total_size;
  int i1;
  int i2;
  int count;

public:
  CircularBufferBase( int size );

  inline int totalSize() const { return total_size; }
  inline int curCount () const { return count     ; }
  inline int size     () const { return count     ; }

  inline bool isEmpty() const { return count == 0; }
  inline bool isFull () const { return count == total_size; }
  
protected:
  void elemPushed();
  void elemPoped();

  inline int startIdx() const { return i1; }
  inline int endIdx  () const { return i2; }
};


// ===================================================================================================== for integral types
template<typename T>
class CircularBuffer : public CircularBufferBase
{
private:
  T * data;
  T total_sum;

public:
  CircularBuffer( int size );
  ~CircularBuffer();

  void push( T elem );
  T pop();
  T get( int idx ) const;

  inline T operator[]( int idx ) const { return get( idx ); }

  inline T totalSum  () const { return total_sum; }
  inline T getAverage() const { return total_sum / curCount(); }
};


// ===================================================================================================== for objects
template<class T>
class CircularBufferForObj : public CircularBufferBase
{
private:
  T ** data;
  bool auto_delete;

public:
  CircularBufferForObj( int size, bool auto_delete );
  ~CircularBufferForObj();

  void push( T * elem );
  T * pop();
  T       * get( int idx );
  T const * get( int idx ) const;

  inline T const * operator[]( int idx ) const { return get( idx ); }

private:
  void deleteAll();
};

} // namespace zygo
