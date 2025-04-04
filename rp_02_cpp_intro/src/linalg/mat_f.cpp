#include "mat_f.h"
#include <assert.h>
#include <iostream>

using namespace std;

//default ctor
MatF::MatF():
  rows(0),
  cols(0),
  dimension(0),
  v(nullptr){}

// ctor with dim
MatF::MatF(int rows_, int cols_):
  rows(rows_),
  cols(cols_),
  dimension(rows*cols),
  v(new float[dimension]){}

// copy ctor
MatF::MatF(const MatF& other):
  MatF(other.rows, other.cols){
  for (int i=0; i<dimension; ++i)
    v[i]=other.v[i];
}

// dtor
MatF::~MatF() {
  if (dimension)
    delete [] v;
}

void MatF::fill(float f) {
  for (int i=0; i<dimension; ++i)
    v[i]=f;
}

void MatF::randFill() {
  for (int i=0; i<dimension; ++i)
    v[i]=drand48();
}

// read/write access to element at pos
float& MatF::at(int pos) {
  return v[pos];
}
  
// read access to element at pos
// const after () means that the method does not modify the invoking object
const float& MatF::at(int pos) const {
  return v[pos];
}

// read/write access to element at pos
float& MatF::at(int r, int c) {
  return v[r*cols+c];
}
  
// read access to element at pos
// const after () means that the method does not modify the invoking object
const float& MatF::at(int r, int c) const {
  return v[r*cols+c];
}

// op =, deep copy
MatF& MatF::operator =(const MatF& other) {
  // TODO: fillme
  if(dimension!=other.dimension) {
    delete [] v;
    v=0;
    dimension=other.dimension;
    if (! dimension)
      return *this;
    v=new float[dimension];
  }
  for(int i = 0; i < other.dimension; i++)
    v[i]=other.v[i];
  return *this;
}

// returns the sum this + other
MatF MatF::operator +(const MatF& other) const {
  assert (other.cols==cols && other.rows==rows && "dim mismatch");
  MatF returned (*this);

  // TODO: fillme
  for(int i = 0; i < other.dimension; i++)
    returned.at(i)+=other.at(i);
  return returned;
}

// returns the difference this - other
MatF MatF::operator -(const MatF& other) const {
  assert (other.cols==cols && other.rows==rows && "dim mismatch");
  MatF returned (*this);

  // TODO: fillme
  for(int i = 0; i < other.dimension; i++)
    returned.at(i)-=other.at(i);
  return returned;
}  

// returns this*f
MatF MatF::operator* (float f) const {
  MatF returned (*this);

  // TODO: fillme
  for(int i = 0; i < dimension; i++)
    returned.at(i)*=f;
  return returned;
}

// returns this * other
VecF MatF::operator *(const VecF& other) const {
  assert(other.dim==cols && "dim mismatch");
  VecF returned (rows);
  // TODO: fillme
  for(int o = 0; o<rows;o++){
    for (int i = 0; i<cols;i++){
      returned.at(o)+=this->at(o,i)*other.at(i);
    }
  }
  return returned;
}

// returns this* other
MatF MatF::operator *(const MatF& other) const {
  assert(cols==other.rows && "dimension mismatch");
  MatF returned(rows, other.cols);
  //TODO: fillme
  for(int o = 0; o<rows;o++){
    for (int i = 0; i<other.cols;i++){
      for(int common = 0; common<cols;common++)
      returned.at(o,i)+=this->at(o,common)*other.at(common,i);
    }
  }
  return returned;
}

MatF MatF::transpose() const {
  MatF returned(cols, rows);
  for (int r=0; r<rows; ++r)
    for (int c=0; c<cols; ++c)
      returned.at(c,r)=at(r,c);
  return returned;
}


std::ostream& operator <<(std::ostream& os, const MatF& m) {
  os << "{ ptr: " << &m <<", rows: " << m.rows << ",  cols: " << m.cols << endl;
  os << "  values:[ " << endl;
  for (int r=0; r<m.rows; ++r) {
    os << "\t";
    for (int c=0; c<m.cols; ++c) {
      os << m.at(r,c) <<  " ";
    }
    os << endl;
  }
  os << " ]" << endl << "}";
  return os;
}
