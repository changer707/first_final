#ifndef MATRIX_H
#define MATRIX_H
#include <initializer_list>
#include <cstdlib>
#include <ostream>

template <class T>
class Matrix {
public:
  Matrix();//3个私有成员变量全为0或空指针
  Matrix(const size_t rows, const size_t columns);
  Matrix(const std::initializer_list<std::initializer_list<T>> init);
  Matrix(const Matrix<T> &other);//4个构造函数重载
  Matrix<T> & operator= (const Matrix<T> &other);//=重载
  ~Matrix();
  // all operations modify the matrix in-place.
  void resize(const size_t rows, const size_t columns, const T default_value = 0);//重新生成一个rows*columns的二维数组，并更新3个私有成员的值
  void clear();//将m_matrix指向的二维数组全置0
  T& operator () (const size_t x, const size_t y);
  const T& operator () (const size_t x, const size_t y) const;//以上两个函数用于返回m_columns[x][y]

  const T min() const;
  const T max() const;//分别求m_matrix[][]中的最大元素与最小元素

  inline size_t minsize() { return ((m_rows < m_columns) ? m_rows : m_columns); }
  inline size_t columns() const { return m_columns;}
  inline size_t rows() const { return m_rows;}

  friend std::ostream& operator<<(std::ostream& os, const Matrix &matrix)//<<的重载, 输出对象matrix.m_matrix指向的各元素
  {
      os << "Matrix:" << std::endl;
      for (size_t row = 0 ; row < matrix.rows() ; row++ )
      {
          for (size_t col = 0 ; col < matrix.columns() ; col++ )
          {
              os.width(8);
              os << matrix(row, col) << ",";
          }
          os << std::endl;
      }
      return os;
  }

private:
  T **m_matrix;//类似行指针
  size_t m_rows;
  size_t m_columns;//三个私有成员变量和众多成员函数
};

#ifndef USE_EXPORT_KEYWORD
#include "matrix.cpp"
//#define export /*export*/
#endif
#endif // MATRIX_H
