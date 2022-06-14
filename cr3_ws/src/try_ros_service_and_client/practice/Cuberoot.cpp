#include <iostream>
#include <cmath>

void CalculateCubeRoot(double& x, double K, double tolerance = 1.0e-6,
		       int maxIteration = 100);

int main(){
  double x = 1.0;
  double K = 12.0;

  CalculateCubeRoot(x, K);

  double tolerance = 0.001;;
  x = 1.0;
  CalculateCubeRoot(x, K, tolerance);

  int maxIterations = 50;
  x = 1.0;
  CalculateCubeRoot(x, K, tolerance, maxIteration);

  return 0;

}

void CalculateCubeRoot(double& x, double K, double tolerance, int maxIterations){
  int iterations = 0;
  double residual = x*x*x-K;
  while ((fabs(residual) > tolerance) && (iteration < maxIterations))
    {
      x = x-(x*x*x-K)/(3.0*x*x);
      residual = x*x*x-K;
      iterations++;
    }
}
