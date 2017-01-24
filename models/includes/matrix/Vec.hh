/********************************* TRICK HEADER *******************************
PURPOSE:
      (Vector wrapper for trick interfacing)
LIBRARY DEPENDENCY:
      ((../src/Vec3.cpp))
*******************************************************************************/

#include <armadillo>

template<unsigned int n>
class Vec : public arma::vec {
    public:

        Vec() : arma::vec(&this->memory[0][0], n, m, false, true){
            this->zeros();
        }

        using arma::vec::operator=;

        double memory[m][n];
};

typedef Vec<3> vec3;
