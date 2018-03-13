#include <iostream>
#include "def.h"
#include "algorithm.h"
#include "model.h"

using namespace std;

int main()
{
    Model model;
    model.set_coordinates(arma::colvec({0,0,-50}));
    model.set_linear_velocity(arma::colvec({2,0,0}));

    model[alg::algorithm_gliding];
    model(std::cout, 50);
}

