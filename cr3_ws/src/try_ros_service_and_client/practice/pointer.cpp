#include <iostream>
using namespace std;

int main(){
  double y;
  double* p_x;

  y = 3.0;
  p_x = &y;
  cout << "y = " << y << "\n";
  cout << "p_x = " << p_x << "\n";
  *p_x = 1.0;
  cout << "y = " << y << "\n";
  cout << "p_x = " << p_x << "\n";

  return 0;


}
