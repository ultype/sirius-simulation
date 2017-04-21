/********************************* TRICK HEADER *******************************
PURPOSE:
      (Matrix wrapper for trick interfacing)
LIBRARY DEPENDENCY:
      ((../src/Mat33.cpp))
*******************************************************************************/

#include <armadillo>

template<unsigned int n, unsigned int m>
class TrickMatrix : public arma::Mat<double> {
    public:

        TrickMatrix() : arma::Mat<double>(&this->memory[0][0], n, m, false, true){
            this->zeros();
        }

        using arma::mat::operator=;

        double memory[m][n];


};

typedef TrickMatrix<3, 3> TrickMatrix33;

namespace arma{
    template<unsigned int n, unsigned int m>
    struct is_Mat< TrickMatrix<n, m> >
      { static const bool value = true; };


    template<unsigned int n, unsigned int m>
        class Proxy< TrickMatrix<n, m> > : public Proxy< Mat<double> >{
            using Proxy< Mat<double> >::Proxy;
        };
}
