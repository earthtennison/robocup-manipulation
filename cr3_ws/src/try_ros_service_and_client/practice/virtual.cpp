#include <iostream>
using namespace std;


class Animal {
private:
  string type;
public:
  Animal() : type("Animal") {}

  virtual string getType() {
    return type;
  };
};

class Dog : public Animal {
private:
  string type;
public:
  Dog() : type("Dog") {}
  string getType() override {
    return type;
  }
};

class Cat : public Animal {
private:
  string type;
public:
  Cat() : type("Cat") {}
  string getType() override {
    return type;
  }
};

void print(Animal* ani) {
  cout << "Animal: " << ani->getType() << endl;
}

int main() {
  Animal* animal1 = new Animal();
  Animal* dog1 = new Dog();
  Animal* cat1 = new Cat();

  print(animal1);
  print(dog1);
  print(cat1);

  return 0;

}
