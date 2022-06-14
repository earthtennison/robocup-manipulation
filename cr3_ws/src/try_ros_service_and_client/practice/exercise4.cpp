#include <iostream>

int main(){
  double* x;
  double* y;
  double* z;
  x = new double [3];
  x[0] = 1;
  x[1] = 2;
  x[2] = 3;
  y = new double [3];
  y[0] = 4;
  y[1] = 5;
  y[2] = 6;
  z = new double [3];
  for (int i=0; i<=1000000000; i++)
    {
      double* a;
      a= new double [1];
      a[0]=0;
      for (int j=0; j<3; j++){
	a[0] += x[j]*y[j];
      }
      delete[] a;
      if(i == 1000000000){
	std::cout << "completed" << std::endl;
      }
    }
  return 0;
}
