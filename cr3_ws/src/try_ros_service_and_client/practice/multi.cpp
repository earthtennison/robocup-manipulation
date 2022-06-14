#include <iostream>

using namespace std;


class Animal {
public:
  void info() {
    cout << "I am an animal." << endl;
  }
};

// derived class 1

class Dog : public Animal {
public:
  void bark() {
    cout << "I am a Dog. Woof woof." << endl;
  }
};

class Cat : public Animal {
public:
  void meow() {
    cout << "I am a cat. Meow." << endl;
  }
};

int main() {
  Dog dog1;
  cout << "Dog class:" << endl;
  dog1.info();
  dog1.bark();

  Cat cat1;
  cout << "\nCat class:" << endl;
  cat1.info();
  cat1.meow();
  return 0;
}
