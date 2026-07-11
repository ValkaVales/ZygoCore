#include "matrix.h"
#include <zygo/core/assert.h>
#include <zygo/math/common/scalar.h>


namespace zygo {

Matrix Matrix::inverse1x1() const
{
  ZgAssert( dimx == 1 && dimy == 1 );
  Matrix res( 1, 1 );

#ifdef MATRIX_EXTRA_RELEASE_ASSERTS
  ZgAssertRelease( !isZero( m[0] ) );
#else
  ZgAssert( !isZero( m[0] ) );
#endif

  res.m[0] = REAL_ONE / m[0];
  return res;
}

Matrix Matrix::inverse2x2() const
{
  ZgAssert( dimx == 2 && dimy == 2 );

  Real determinant = m[0] * m[3] - m[1] * m[2];

#ifdef MATRIX_EXTRA_RELEASE_ASSERTS
  ZgAssertRelease( !isZero( determinant ) );
#else
  ZgAssert( !isZero( determinant ) );
#endif

  Real invd = REAL_ONE / determinant;

  Matrix res( 2, 2 );

  res.m[0] =  m[3] * invd,
  res.m[1] = -m[1] * invd,
  res.m[2] = -m[2] * invd,
  res.m[3] =  m[0] * invd;

  return res;
}

Matrix Matrix::inverse3x3() const
{
  ZgAssert( dimx == 3 && dimy == 3 );

  Real determinant = 
    get(0,0) * (get(1,1) * get(2,2) - get(2,1) * get(1,2))
   -get(1,0) * (get(0,1) * get(2,2) - get(2,1) * get(0,2))
   +get(2,0) * (get(0,1) * get(1,2) - get(1,1) * get(0,2));

#ifdef MATRIX_EXTRA_RELEASE_ASSERTS
  ZgAssertRelease( !isZero( determinant ) );
#else
  ZgAssert( !isZero( determinant ) );
#endif

  Real invd = REAL_ONE / determinant;

  Matrix res( 3, 3 );

  // TODO: use single idx, not i,j
  res.setAt( 0, 0,    (get(1,1) * get(2,2) - get(2,1) * get(1,2)) * invd );
  res.setAt( 1, 0,   -(get(1,0) * get(2,2) - get(2,0) * get(1,2)) * invd );
  res.setAt( 2, 0,    (get(1,0) * get(2,1) - get(2,0) * get(1,1)) * invd );
  res.setAt( 0, 1,   -(get(0,1) * get(2,2) - get(2,1) * get(0,2)) * invd );
  res.setAt( 1, 1,    (get(0,0) * get(2,2) - get(2,0) * get(0,2)) * invd );
  res.setAt( 2, 1,   -(get(0,0) * get(2,1) - get(2,0) * get(0,1)) * invd );
  res.setAt( 0, 2,    (get(0,1) * get(1,2) - get(1,1) * get(0,2)) * invd );
  res.setAt( 1, 2,   -(get(0,0) * get(1,2) - get(1,0) * get(0,2)) * invd );
  res.setAt( 2, 2,    (get(0,0) * get(1,1) - get(1,0) * get(0,1)) * invd );

  return res;
}

// ------------------------------------------------------------------------------------------------------------------------- Universal inverse
// Инвертирование квадратной матрицы любого размера.
// Метод Гаусса-Жордана с частичным выбором ведущего элемента.
//
// Идея: приписываем справа единичную матрицу [A | E] и элементарными операциями над строками превращаем A в E.
// Те же операции превращают E в A^-1:
//   [A | E]  ->  [E | A^-1]
//
// Возвращает false, если матрица вырождена (|pivot| <= epsilon).
bool Matrix::tryInverse( Matrix& res, Real epsilon ) const
{
  ZgAssert( dimx == dimy );
  ZgAssert( res.dimx == dimx && res.dimy == dimy );

  const int n = dimx;

  Matrix a = *this;    // рабочая копия: будет приведена к E
  res.makeIdentity();  // сюда теми же операциями приедет A^-1

  for ( int col = 0; col < n; ++col )
  {
    // --- 1. Частичный выбор ведущего: максимальный |a[row][col]| среди row >= col.
    //        Это не оптимизация, а необходимость: без него метод численно неустойчив и падает на любом нуле на диагонали.
    int pivot_row  = col;
    Real max_abs = std::fabs( a[col][col] );

    for ( int row = col + 1; row < n; ++row )
    {
      Real v = std::fabs( a[row][col] );
      if ( v > max_abs )
      {
        max_abs   = v;
        pivot_row = row;
      }
    }

    if ( max_abs <= epsilon )
      return false; // вырожденная матрица

    // --- 2. Переставляем строки в обеих матрицах
    a  .swapRows( col, pivot_row );
    res.swapRows( col, pivot_row );

    // --- 3. Нормируем ведущую строку
    Real const inv_pivot = REAL_ONE / a[col][col];

    Real* a_piv = a  [col];
    Real* r_piv = res[col];

    for ( int i = col; i < n; ++i )   // в 'a' левее col уже нули
      a_piv[i] *= inv_pivot;
    for ( int i = 0; i < n; ++i )     // в 'res' нужна вся строка
      r_piv[i] *= inv_pivot;

    a_piv[col] = REAL_ONE;          // ровно 1, без ошибки округления

    // --- 4. Обнуляем столбец col во всех остальных строках
    for ( int row = 0; row < n; ++row )
    {
      if ( row == col )
        continue;

      Real const k = a[row][col];
      if ( k == REAL_ZERO )
        continue;

      Real* a_row = a  [row];
      Real* r_row = res[row];

      for ( int i = col; i < n; ++i ) // в 'a' левее col уже нули
        a_row[i] -= k * a_piv[i];

      for ( int i = 0; i < n; ++i )
        r_row[i] -= k * r_piv[i];

      a_row[col] = REAL_ZERO;       // ровно 0
    }
  }

  return true;
}

Matrix Matrix::inverse( Real epsilon ) const
{
  Matrix res( dimy, dimx );
  const bool ok = tryInverse( res, epsilon );
  ZgAssertRelease( ok );
  return res;
}

} // namespace zygo
