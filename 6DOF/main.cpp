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

    model(con::linear_thrust_depended_consuption);


    ofstream file("data_m");
    //ofstream file("data_k");

    model[alg::algorithm_keeping_heigth];

    model(file, 50);





    model[alg::algorithm_minization_fuel_consumption];

    //model[alg::algorithm_keeping_heigth];


    model(file, 250);

    double MASS = 2133 * 1e-3;

    const arma::vec TRANS_FUSE_BODY_VECTOR = arma::colvec({0.037, 0, 0.149}); // denoted by S_FB


    arma::mat INERTIA = def::get_rectangle_inertia(
                0.17, 0.15, 0.4, MASS/(0.17* 0.15 *0.4), TRANS_FUSE_BODY_VECTOR
                );

    std::cout <<INERTIA;
}

