#include <iostream>

using namespace std;

class Animal {
  // eat() function
  // sleep() function
public:
  void eat() {
    cout << "I can eat!" << endl;
  }
  void sleep() {
    cout << "I can sleep!" << endl;
  }
};


class Dog : public Animal {
  // bark() function
public:
  void bark() {
    cout << "I can bark! Woof Woof!!" << endl;
  }
};

int main() {
  Dog dog1;

  dog1.eat();
  dog1.sleep();

  dog1.bark();

  return 0;
}
