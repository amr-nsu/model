#include <iostream>
#include "def/def.h"
#include "algorithm.h"
#include "consumption.h"
#include "model.h"

using namespace std;

int main()
{
    Model model;
    model.set_coordinates(arma::colvec({0,0,-50}));
    model.set_linear_velocity(arma::colvec({2,0,0}));

    //model(con::linear_thrust_depended_consuption);
    model(con::no_consuption);


    ofstream file("data_test_14");
    //ofstream file("data_k");

    model[alg::algorithm_gliding];

    model[alg::algorithm_keeping_heigth];

    model(file, 50);

    model[alg::algorithm_minization_fuel_consumption];

    model(file, 150);


}

