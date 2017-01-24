/********************************* TRICK HEADER *******************************
PURPOSE:
      (Matrix wrapper for trick interfacing)
LIBRARY DEPENDENCY:
      ((../src/Mat33.cpp))
*******************************************************************************/

#include <armadillo>

template<unsigned int n, unsigned int m>
class Mat : public arma::mat {
    public:

        Mat() : arma::mat(&this->memory[0][0], n, m, false, true){
            this->zeros();
        }

        using arma::mat::operator=;

        double memory[m][n];
};

typedef Mat<3, 3> Mat33;
