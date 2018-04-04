#ifndef DEF_MATH_H
#define DEF_MATH_H

#include <armadillo>

///note файл стандартных математических определений

namespace def
{

    template <class Type>
    inline int sign(Type x)
    {
      if (x == 0.)  return 0;
      if (x > 0.)  return 1;
      else return -1;
    }

    inline arma::mat skew_matrix(const arma::vec& vector)
    {
        double i = vector(0);
        double j = vector(1);
        double k = vector(2);

        arma::mat tmp(3,3);

        tmp(0, 0) = 0;
        tmp(0, 1) = -k;
        tmp(0, 2) = j;

        tmp(1, 0) = k;
        tmp(1, 1) = 0;
        tmp(1, 2) = -i;

        tmp(2, 0) = -j;
        tmp(2, 1) = i;
        tmp(2, 2) = 0;

        return std::move(tmp);
    }

    inline arma::mat coord_rotational_matrix(const arma::vec& eulerAngles)
    {
        double phi = eulerAngles(0);
        double theta = eulerAngles(1);
        double psi = eulerAngles(2);

        arma::mat tmp(3,3);

        tmp(0, 0) = cos(theta) * cos(psi);
        tmp(1, 0) = cos(theta) * sin(psi);
        tmp(2, 0) = -sin(theta);

        tmp(0, 1) = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
        tmp(1, 1) = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
        tmp(2, 1) = cos(theta) * sin(phi);

        tmp(0, 2) = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
        tmp(1, 2) = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
        tmp(2, 2) = cos(phi) * cos(theta);

        return std::move(tmp);
    }

    inline arma::mat angular_velocity_rotational_matrix(const arma::vec& eulerAngles)
    {
        double phi = eulerAngles(0);
        double theta = eulerAngles(1);

        arma::mat tmp(3,3);

        tmp(0, 0) = 1;
        tmp(0, 1) = sin(phi) * tan(theta);
        tmp(0, 2) = cos(phi) * tan(theta);

        tmp(1, 0) = 0;
        tmp(1, 1) = cos(phi);
        tmp(1, 2) = -sin(phi);

        tmp(2, 0) = 0;
        tmp(2, 1) = sin(phi) / cos(theta);
        tmp(2, 2) = cos(phi) / cos(theta);

        return std::move(tmp);
    }

}

#endif // DEF_MATH_H

