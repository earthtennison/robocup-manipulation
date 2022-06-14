#include <iostream>
using namespace std;

// Class Template
template <class T>
class Number {
private:
  T num;
public:
  Number(T n) : num(n) {}

  T getNum() {
    return num;
  }
};

int main() {
  
  // create object with int type
  Number<int> numberInt(7);

  // create object with double type
  Number<double> numberDouble(7.7);


  cout << "int Number = " << numberInt.getNum() << endl;
  cout << "double Number = " << numberDouble.getNum() << endl;

}
