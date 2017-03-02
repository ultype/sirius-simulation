///////////////////////////////////////////////////////////////////////////////
// FILE 'utility_functions.cpp'
//
//'Matrix' class member functions
// Module utility functions:
//    angle
//    sign
//    cad_geo84_in
//    cad_kepler (based on Morth)
// Stochastic functions
//    exponential
//    gauss
//    markov
//    rayleigh
//    uniform
//    unituni
// Table look-up
// Integration
// US76 Atmosphere
// US76 Atmosphere extended to 1000km (NASA Marshall)
//
// 010628 Created by Peter H Zipfel
// 020723 In 'markov' replaced static variable by '&value_saved', PZi
// 020829 Dynamically dimensioned utilities, PZi
// 030319 Added US76 atmosphere, PZi
// 030411 WGS84 utilities, PZi
// 030519 Overloaded operator [] for vector of type 'Matrix', PZi
// 030717 Improved table look-up, PZi
// 040311 US76 Atmosphere extended to 1000km (NASA Marshall), PZi
// 040319 Kepler utility, PZi
// 040326 Unit vector cross product operator%, PZi
// 040510 Added cad_in_orb, cad_orb_in, cad_tip, PZi
// 050202 Simplified and renamed 'integrate(...)' to Modified Euler method, PZi
// 071029 Added 'cholesky', PZi
// 071106 Added scalar division operator /, PZi
///////////////////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_DEPRECATE
#include <fstream>
#include <cstring>
#include <cmath>
#include <stdlib.h>
#include <assert.h>
#include "math/utility.hh"
#include "math/integrate.hh"
#include "aux/utility_header.hh"
#include <iomanip>
//#include "global_header.hh"


///////////////////////////////////////////////////////////////////////////////
/////////////////////// 'Matrix' member functions /////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// One dimensional and two dimensional arrays of any dimension of type 'double'
//
// 020826 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
// Constructors
///////////////////////////////////////////////////////////////////////////////
Matrix::Matrix()
{
}

Matrix::Matrix(int row_size, int col_size)
{
    // std::cout<<" >>> constructing >>>\n";

    num_row = row_size;
    num_col = col_size;

    pbody = NULL;

    // allocating memory
    num_elem = row_size * col_size;
    pbody = new double[num_elem];
    assert(pbody && " *** Error: matrix memory allocation failed *** ");

    // initializing array to zero
    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = 0.;
}

Matrix::Matrix(const Matrix &MAT)
{
    // std::cout<<" >>> copy constructing >>>\n";

    num_row = MAT.num_row;
    num_col = MAT.num_col;
    num_elem = MAT.num_elem;
    pbody = new double[num_elem];
    assert(pbody && " *** Error: matrix memory allocation failed *** ");

    // copying
    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = (*(MAT.pbody + i));
}

Matrix::Matrix(const double v[3])
{
    num_row = 3;
    num_col = 1;
    pbody = NULL;

    // allocating memory
    num_elem = num_row * num_col;
    pbody = new double[num_elem];
    assert(pbody && " *** Error: matrix memory allocation failed *** ");

    memcpy(pbody, (const double*) v, sizeof(double) * 3);
}

Matrix::Matrix(const double v[][3])
{
    num_row = 3;
    num_col = 3;
    pbody = NULL;

    // allocating memory
    num_elem = num_row * num_col;
    pbody = new double[num_elem];
    assert(pbody && " *** Error: matrix memory allocation failed *** ");

    memcpy(pbody, (const double*) v, sizeof(double) * 9);
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
///////////////////////////////////////////////////////////////////////////////
Matrix::~Matrix()
{
    //    std::cout<<" <<< destructing <<<\n";
    delete[] pbody;
}


///////////////////////////////////////////////////////////////////////////////
// Printing matrix to console
// Example: MAT.print();
///////////////////////////////////////////////////////////////////////////////
void Matrix::print()
{
    double *pmem = pbody;

    // outside loop rows, inside loop columns
    for (int i = 0; i < num_row; i++) {
        for (int j = 0; j < num_col; j++) {
            std::cout<<setprecision(12)<< *pbody << "\t";
            pbody++;
        }
        std::cout << '\n';
    }
    // resetting pointer
    pbody = pmem;
    std::cout << "\n\n";
}

///////////////////////////////////////////////////////////////////////////////
// Absolute value of vector
// Example: avalue = VEC.absolute();
///////////////////////////////////////////////////////////////////////////////
double Matrix::absolute()
{
    if (num_row > 1 && num_col > 1) {
        std::cerr
            << " *** Warning: not a vector in 'Matrix::absolute()' *** \n";
    }
    double ret = 0.0;

    for (int i = 0; i < num_elem; i++)
        ret += (*(pbody + i)) * (*(pbody + i));
    ret = sqrt(ret);

    return ret;
}

///////////////////////////////////////////////////////////////////////////////
// Adjoint matrix (same as determinant procedure however the matrix element
// is NOT multiplied into each cofactor)
// Example: BMAT = AMAT.adjoint();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::adjoint()
{
    assert((num_row == num_col) &&
           " *** Error: matrix not square in 'Matrix::adjoint()' *** ");

    assert((num_row != 1 || num_col != 1) &&
           " *** Error: only one element in 'Matrix::adjoint()' *** ");

    Matrix RESULT(num_row, num_col);

    for (int i = 0; i < num_elem; i++) {
        // row #
        int row = i / num_col + 1;
        // column #
        int col = i % num_col + 1;

        if (((row + col) % 2) == 0)
            *(RESULT.pbody + i) = sub_matrix(row, col).determinant();
        else
            *(RESULT.pbody + i) = (-1.0) * sub_matrix(row, col).determinant();
    }
    return RESULT.trans();
}

//////////////////////////////////////////////////////////////////////////////
// Assigns a value to a matrix element (offset!)
// Example: MAT.assign_loc(r,c,val); ((r+1)th-row, (c+1)th-col)
///////////////////////////////////////////////////////////////////////////////
void Matrix::assign_loc(const int &r, const int &c, const double &val)
{
    assert(((r <= num_row - 1) && (c <= num_col - 1)) &&
           " *** Error: location outside array in 'Matrix::assign_loc()' *** ");

    // assigning value
    int offset = num_col * (r) + c;
    *(pbody + offset) = val;
}

///////////////////////////////////////////////////////////////////////////////
// Builds a 3x1 vector from three parameters
// Example: VEC.build_vec3(v1,v2,v3);
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::build_vec3(const double &v1, const double &v2, const double &v3)
{
    num_row = 3;
    num_col = 1;
    *pbody = v1;
    *(pbody + 1) = v2;
    *(pbody + 2) = v3;

    return *this;
}


Matrix &Matrix::build_vec3(const double v[3])
{
    num_row = 3;
    num_col = 1;

    memcpy(pbody, v, sizeof(double) * 3);

    return *this;
}
///////////////////////////////////////////////////////////////////////////////
// Builds a 3x3 matrix from nine paramters arranged in rows
// Example: MAT.build_mat33(v11,v12,v13,v21,v22,v23,v31,v32,v33);
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::build_mat33(const double &v11,
                            const double &v12,
                            const double &v13,
                            const double &v21,
                            const double &v22,
                            const double &v23,
                            const double &v31,
                            const double &v32,
                            const double &v33)
{
    num_row = 3;
    num_col = 3;
    *pbody = v11;
    *(pbody + 1) = v12;
    *(pbody + 2) = v13;
    *(pbody + 3) = v21;
    *(pbody + 4) = v22;
    *(pbody + 5) = v23;
    *(pbody + 6) = v31;
    *(pbody + 7) = v32;
    *(pbody + 8) = v33;

    return *this;
}

Matrix &Matrix::build_mat33(const double v[][3])
{
    num_row = 3;
    num_col = 3;

    memcpy(pbody, (const double*) v, sizeof(double) * 9);

    return *this;
}

Matrix &Matrix::fill(double v[3])
{
    memcpy(v, pbody, sizeof(double) * 3);

    return *this;
}

Matrix &Matrix::fill(double m[][3])
{
    memcpy((double*) m, pbody, sizeof(double) * 9);

    return *this;
}

///////////////////////////////////////////////////////////////////////////////
// Calulates Cartesian from polar coordinates
// |V1|             | cos(elevation)*cos(azimuth)|
// |V2| = magnitude*|cos(elevation)*sin(azimuth) |
// |V3|             |      -sin(elevation)       |
//
// Example: VEC.cart_from_pol(magnitude,azimuth,elevation);
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::cart_from_pol(const double &magnitude,
                              const double &azimuth,
                              const double &elevation)
{
    *pbody = magnitude * (cos(elevation) * cos(azimuth));
    *(pbody + 1) = magnitude * (cos(elevation) * sin(azimuth));
    *(pbody + 2) = magnitude * (sin(elevation) * (-1.0));

    return *this;
}
///////////////////////////////////////////////////////////////////////////////
// Returns square root matrix of square matrix MAT
// Example: SQRTMAT=MAT.cholesky();
// 071029 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::cholesky()
{
    assert((num_row == num_col) &&
           " *** Error: matrix not square 'Matrix::cholesky()' *** ");

    Matrix SQRTMAT(num_row, num_col);
    double sum(0);
    int dim = num_row;

    for (int i = 0; i < dim; i++) {
        for (int j = 0; j < dim; j++) {
            // off-diagonal elements
            if (j < i) {
                sum = 0;
                if (j > 0) {
                    for (int k = 0; k < j; k++)
                        sum += (*(SQRTMAT.pbody + i * dim + k)) *
                               (*(SQRTMAT.pbody + j * dim + k));
                }
                if (*(SQRTMAT.pbody + j * dim + j) == 0)
                    *(SQRTMAT.pbody + i * dim + j) = 0;
                else
                    *(SQRTMAT.pbody + i * dim + j) =
                        (*(pbody + i * dim + j) - sum) /
                        (*(SQRTMAT.pbody + j * dim + j));
            }
            // diagonal elements
            else if (j == i) {
                sum = 0;
                if (i > 0) {
                    for (int k = 0; k < i; k++)
                        sum += *(SQRTMAT.pbody + i * dim + k) *
                               (*(SQRTMAT.pbody + i * dim + k));
                }
                *(SQRTMAT.pbody + i * dim + j) =
                    sqrt(*(pbody + i * dim + i) - sum);
            } else {
                *(SQRTMAT.pbody + i * dim + j) = 0;
            }
        }
    }
    return SQRTMAT;
}
///////////////////////////////////////////////////////////////////////////////
// Returns column vector of column #
// Example: VEC = MAT.col_vec(2); (2nd column!)
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::col_vec(const int &col)
{
    assert((col > 0 && col <= num_col) &&
           " *** Error: column outside array in 'Matrix::col_vec()' *** ");

    Matrix RESULT(num_row, 1);

    for (int i = 0; i < num_row; i++) {
        int offset = i * num_col + col - 1;
        *(RESULT.pbody + i) = (*(pbody + offset));
    }
    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Returns the determinant
// Determinant recursive procedure
// Example: det = MAT.determinant();
///////////////////////////////////////////////////////////////////////////////
double Matrix::determinant()
{
    assert((num_row == num_col) &&
           " *** Error: matrix not square in 'Matrix::determinant()' *** ");

    double result = 0.0;

    // base case of a single matrix element
    if ((num_col == 1) && (num_row == 1))
        return *pbody;

    // second base case of a 2x2 matrix
    else if ((num_col == 2) && (num_row == 2))
        return (*pbody) * (*(pbody + 3)) - (*(pbody + 1)) * (*(pbody + 2));

    else {
        for (int j = 0; j < num_col; j++) {
            // use cofactors and submatricies to finish for nxn
            if ((j % 2) == 0) {
                // odd column (numbered!)
                result += sub_matrix(1, j + 1).determinant() * (*(pbody + j));
            } else {
                // even column (numbered!)
                result += (-1.0) * sub_matrix(1, j + 1).determinant() *
                          (*(pbody + j));
            }
        }
    }
    return result;
}

///////////////////////////////////////////////////////////////////////////////
// Returns nxn diagonal matrix  from nx1 vector
// Example: DIAMAT=VEC.diamat_vec()
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::diamat_vec()
{
    assert(num_col == 1 &&
           " *** Error: not a vector in 'Matrix::diagmat_vec()' *** ");

    Matrix RESULT(num_row, num_row);
    for (int i = 0; i < num_row; i++) {
        int offset = i * num_row + i;
        *(RESULT.pbody + offset) = (*(pbody + i));
    }
    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Returns nx1 diagonal vector from nxn matrix
// Example: VEC=MAT.diavec_mat();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::diavec_mat()
{
    assert(num_row == num_col &&
           " *** Error: matrix not square in 'Matrix::diavec_mat()' *** ");

    Matrix RESULT(num_row, 1);
    for (int i = 0; i < num_row; i++) {
        int offset = i * num_row + i;
        *(RESULT.pbody + i) = (*(pbody + offset));
    }
    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Dimensions a matrix of size row x col
// Only used to initialize arrays in class 'Variable'
// Example: MAT.dimension(3,3);
///////////////////////////////////////////////////////////////////////////////
void Matrix::dimension(int row, int col)
{
    num_row = row;
    num_col = col;

    pbody = NULL;

    // allocating memory
    num_elem = row * col;
    pbody = new double[num_elem];
    assert(pbody &&
           " *** Error: memory allocation failed 'Matrix::dimension()' *** ");

    // initializing array to zero
    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = 0.;
}
///////////////////////////////////////////////////////////////////////////////
// Bi-variate ellipse
// calculating major and minor semi-axes of ellipse and rotation angle
//    from the symmetrical pos semi-definite MAT(2x2) matrix
// coordinate axes orientation:
//          ^ 1-axis
//          |
//          |
//          |---> 2-axis
//
// angle is measured from 1st coordinate axis to the right
//
// major_semi_axis = ELLIPSE.get_loc(0,0);
// minor_semi_axis = ELLIPSE.get_loc(1,0);
// angle      = ELLIPSE.get_loc(2,0);
//
// Example: ELLIPSE = MAT.ellipse();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::ellipse()
{
    Matrix ELLIPSE(3, 1);
    double dum(0);
    double dum1(0);
    double dum2(0);
    double phi(0);
    double ak1(0);
    double ak2(0);
    Matrix X1V(2, 1);  // major principal axes of ellipse
    Matrix X2V(2, 1);  // minor principal axes of ellipse

    double a11 = *pbody;
    double a22 = *(pbody + 3);
    double a12 = *(pbody + 1);
    double a1122 = a11 + a22;
    double aq1122 = a1122 * a1122;
    dum1 = aq1122 - 4. * (a11 * a22 - a12 * a12);
    if (dum1 >= 0.)
        dum2 = sqrt(dum1);

    // major and minor semi-axes of ellipse
    double ama = (a1122 + dum2) / 2.;
    double ami = (a1122 - dum2) / 2.;
    ELLIPSE.assign_loc(0, 0, ama);
    ELLIPSE.assign_loc(1, 0, ami);
    if (ama == ami)
        return ELLIPSE;

    // angle of orientation of major axis wrt first principal axis
    if (a11 - ama != 0.) {
        dum1 = -a12 / (a11 - ama);
        ak1 = sqrt(1. / (1. + dum1 * dum1));
        X1V.assign_loc(0, 0, dum1 * ak1);
        X1V.assign_loc(1, 0, ak1);
        dum = dum1 * ak1;
        if (fabs(dum) > 1.)
            dum = 1. * sign(dum);
        phi = acos(dum);
        ELLIPSE.assign_loc(2, 0, phi);
    } else {
        dum1 = -a12 / (a22 - ama);
        ak1 = sqrt(1. / (1. + dum1 * dum1));
        X1V.assign_loc(0, 0, ak1);
        X1V.assign_loc(1, 0, dum1 * ak1);
        if (fabs(ak1) > 1.)
            ak1 = 1. * sign(ak1);
        phi = acos(ak1);
        ELLIPSE.assign_loc(2, 0, phi);
    }
    // second principal axis - not used
    if (a11 - ami != 0.) {
        dum2 = -a12 / (a11 - ami);
        ak2 = sqrt(1. / (1. + dum2 * dum2));
        X2V.assign_loc(0, 0, dum2 * ak2);
        X2V.assign_loc(1, 0, ak2);
    } else {
        dum2 = -a12 / (a22 - ami);
        ak2 = sqrt(1. / (1. + dum2 * dum2));
        X2V.assign_loc(0, 0, ak2);
        X2V.assign_loc(1, 0, dum2 * ak2);
    }
    return ELLIPSE;
}

///////////////////////////////////////////////////////////////////////////////
// Returns the number of columns of matrix MAT
// Example: nc = MAT.get_cols();
///////////////////////////////////////////////////////////////////////////////
int Matrix::get_cols()
{
    return num_col;
}

///////////////////////////////////////////////////////////////////////////////
// Returns offset-index given row# and col#
// Example: i = MAT.get_index(2,3); (2nd row, 3rd column)
//////////////////////////////////////////////////////////////////////////////
int Matrix::get_index(const int &row, const int &col)
{
    int index;
    index = (row - 1) * num_col + col - 1;
    return index;
}

///////////////////////////////////////////////////////////////////////////////
// Returns the value at offset-row 'r' offset-col 'c' of MAT
// Example: value = MAT.get_loc(2,1); (3rd row, 2nd column)
///////////////////////////////////////////////////////////////////////////////
double Matrix::get_loc(const int &r, const int &c)
{
    assert((r < num_row) && (c < num_col) &&
           " *** Error: invalid matrix location,'Matrix::get_loc()' *** ");

    return *(pbody + r * num_col + c);
}

///////////////////////////////////////////////////////////////////////////////
// Returns the number of rows in the matrix
// Example: nr = MAT.get_rows();
///////////////////////////////////////////////////////////////////////////////
int Matrix::get_rows()
{
    return num_row;
}

///////////////////////////////////////////////////////////////////////////////
// Returns the pointer to MAT
// Example: ptr = MAT.get_pbody();
///////////////////////////////////////////////////////////////////////////////
double *Matrix::get_pbody()
{
    return pbody;
}

///////////////////////////////////////////////////////////////////////////////
// Builds a square identity matrix of object 'Matrix MAT'
// Example: MAT.identity();
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::identity()
{
    assert(num_row == num_col &&
           " *** Error: matrix not square 'Matrix::identiy()'*** ");

    for (int r = 0; r < num_row; r++)
        *(pbody + r * num_row + r) = 1.;

    return *this;
}

///////////////////////////////////////////////////////////////////////////////
// Returns the inverse of a square matrix AMAT
// Inversion  INVERSE =(1/det(A))*Adj(A)
// Example: INVERSE = AMAT.inverse();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::inverse()
{
    assert(num_col == num_row &&
           " *** Error: not a square matrix 'Matrix::inverse()' *** ");

    Matrix RESULT(num_row, num_col);
    double d = 0.;

    d = determinant();

    assert(d != 0. && " *** Error: singular! 'Matrix::inverse()' *** ");

    d = 1. / d;
    RESULT = adjoint();
    RESULT = RESULT * d;

    return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
// Returns 3x3 matrix row-wise from 9x1 vector
// Example: MAT=VEC. mat33_vec9();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::mat33_vec9()
{
    assert((num_row == 9 && num_col == 1) &&
           " *** Error: vector not 9 x 1 'Matrix::mat33_vec9()' *** ");

    Matrix RESULT(3, 3);
    for (int i = 0; i < 9; i++) {
        *(RESULT.pbody + i) = *(pbody + i);
    }
    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Forms  matrix MAT with all elements '1.' from object MAT(num_row,num_col)
// Example: MAT.ones();
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::ones()
{
    for (int r = 0; r < num_elem; r++)
        *(pbody + r) = 1.;

    return *this;
}

///////////////////////////////////////////////////////////////////////////////
// Inequality relational operator, returns true or false
// returns true if elements differ by more than EPS
// Example: if(AMAT!=BMAT){...};
///////////////////////////////////////////////////////////////////////////////
bool Matrix::operator!=(const Matrix &B)
{
    // check dimensions
    if (num_col != B.num_col)
        return true;
    else if (num_row != B.num_row)
        return true;

    for (int i = 0; i < num_elem; i++) {
        // check to see if values differ by more or less than EPS
        if ((*(pbody + i) - (*(B.pbody + i))) > EPS)
            return true;
        else if ((*(pbody + i) - (*(B.pbody + i))) < (-1. * EPS))
            return true;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// Scalar multiplication operator
// Note: scalar must be the second operand
// Example: CMAT = AMAT * b;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator*(const double &b)
{
    Matrix RESULT(num_row, num_col);

    for (int i = 0; i < num_elem; i++)
        *(RESULT.pbody + i) = *(pbody + i) * b;

    return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
// Scalar division operator
// Note: scalar must be the second operand
// Example: CMAT = AMAT / b;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator/(const double &b)
{
    Matrix RESULT(num_row, num_col);

    for (int i = 0; i < num_elem; i++)
        *(RESULT.pbody + i) = (*(pbody + i)) / b;

    return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
// Multiplication operator, returns matrix product
// associative but not commutative
// Example: CMAT = AMAT * BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator*(const Matrix &B)
{
    // create resultant matrix
    Matrix RESULT(num_row, B.num_col);
    int r = 0;
    int c = 0;

    // check for proper dimensions
    assert((num_col == B.num_row) &&
           " *** Error: incompatible dimensions 'Matrix::operator*()' *** ");

    for (int i = 0; i < RESULT.num_elem; i++) {
        r = i / B.num_col;
        c = i % B.num_col;
        for (int k = 0; k < num_col; k++) {
            *(RESULT.pbody + i) +=
                *(pbody + k + num_col * r) * (*(B.pbody + k * B.num_col + c));
        }
    }
    return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
// Scalar multiplication assignment operator (scalar element by element
// multiplication)
// Example: AMAT *= b; meaning: AMAT = AMAT * b
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::operator*=(const double &b)
{
    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = *(pbody + i) * b;

    return *this;
}

///////////////////////////////////////////////////////////////////////////////
// Matrix multiplication assignment operator
// matrix B in argument must be square
// Example: AMAT *= BMAT; meaning: AMAT = AMAT * BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::operator*=(const Matrix &B)
{
    // create resultant matrix
    Matrix RESULT(num_row, B.num_col);

    // check for proper dimensions
    assert(
        (num_col == B.num_row) &&
        " *** Error: incompatible dimensions in 'Matrix::operator*=()' *** ");

    // check for squareness of B
    assert((B.num_col == B.num_row) &&
           " *** Error: Second matrix is not square in 'Matrix::operator*=()' "
           "*** ");

    int i(0);
    for (i = 0; i < RESULT.num_elem; i++) {
        int r = i / B.num_col;
        int c = i % B.num_col;
        for (int k = 0; k < num_col; k++) {
            *(RESULT.pbody + i) +=
                *(pbody + k + num_col * r) * (*(B.pbody + k * B.num_col + c));
        }
    }
    num_col = RESULT.num_col;
    num_row = RESULT.num_row;
    num_elem = num_row * num_col;
    for (i = 0; i < num_elem; i++)
        *(pbody + i) = *(RESULT.pbody + i);

    return *this;
}

///////////////////////////////////////////////////////////////////////////////
// Scalar Addition operator (scalar element by element addition)
// Note: scalar must be the second operand
// Example: CMAT = AMAT + b;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator+(const double &b)
{
    Matrix RESULT(num_row, num_col);

    for (int i = 0; i < num_elem; i++)
        *(RESULT.pbody + i) = *(pbody + i) + b;

    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Addition operator, returns matrix addition
// Example: CMAT = AMAT + BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator+(const Matrix &B)
{
    Matrix RESULT(num_row, num_col);

    assert(((num_col == B.num_col) && (num_row == B.num_row)) &&
           " *** Error: matrices have different dimensions in "
           "'Matrix::operator +' *** ");

    for (int i = 0; i < num_elem; i++)
        *(RESULT.pbody + i) = *(pbody + i) + (*(B.pbody + i));

    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Scalar addition assignment operator (scalar element by element addition)
// Example: AMAT += b; meaning: AMAT = AMAT + b
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::operator+=(const double &b)
{
    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = *(pbody + i) + b;

    return *this;
}

///////////////////////////////////////////////////////////////////////////////
// Matrix addition assignment operator
// Example: AMAT += BMAT; meaning: AMAT = AMAT + BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::operator+=(const Matrix &B)
{
    assert(((num_col == B.num_col) && (num_row == B.num_row)) &&
           " *** Error: matrices have different dimensions in "
           "'Matrix::operator +=' *** ");

    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = *(pbody + i) + (*(B.pbody + i));

    return *this;
}

///////////////////////////////////////////////////////////////////////////////
// Scalar substraction operator (scalar element by element substraction)
// Note: scalar must be the second operand
// Example: CMAT = AMAT - b;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator-(const double &b)
{
    Matrix RESULT(num_row, num_col);
    for (int i = 0; i < num_elem; i++)
        *(RESULT.pbody + i) = *(pbody + i) - b;

    return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
// Substraction operator, returns matrix substraction
// Example: CMAT = AMAT - BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator-(const Matrix &B)
{
    Matrix RESULT(num_row, num_col);

    assert(((num_col == B.num_col) && (num_row == B.num_row)) &&
           " *** Error: matrices have different dimensions in "
           "'Matrix::operator -' *** ");

    for (int i = 0; i < num_elem; i++)
        *(RESULT.pbody + i) = *(pbody + i) - *(B.pbody + i);

    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Scalar substraction assignment operator (scalar element by element
// substraction)
// Example: AMAT -= b; meaning: AMAT = AMAT - b
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::operator-=(const double &b)
{
    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = *(pbody + i) - b;

    return *this;
}

///////////////////////////////////////////////////////////////////////////////
// Matrix subtraction assignment operator
// Example: AMAT -= BMAT; meaning: AMAT = AMAT - BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::operator-=(const Matrix &B)
{
    assert(((num_col == B.num_col) && (num_row == B.num_row)) &&
           " *** Error: matrices have different dimensions in "
           "'Matrix::operator +=' *** ");

    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = *(pbody + i) - (*(B.pbody + i));

    return *this;
}

///////////////////////////////////////////////////////////////////////////////
// Assignment operator (deep copy)
// Example: AMAT = BMAT; also: AMAT = BMAT = CMAT;
// Actually: AMAT.operator=(BMAT); also: AMAT.operator=(BMAT.operator=(CMAT));
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::operator=(const Matrix &B)
{
    assert((num_row == B.num_row) && (num_col == B.num_col) &&
           " *** Error: incompatible dimensions in 'Matrix::operator=()' *** ");

    delete[] pbody;
    num_elem = B.num_elem;
    num_row = B.num_row;
    num_col = B.num_col;
    pbody = new double[num_elem];

    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = (*(B.pbody + i));

    return *this;
}
///////////////////////////////////////////////////////////////////////////////
// Equality relational operator
// returns true if elements differ by less than EPS
// Example: if(AMAT==BMAT){...};
///////////////////////////////////////////////////////////////////////////////
bool Matrix::operator==(const Matrix &B)
{
    // check dimensions
    if (num_col != B.num_col)
        return false;
    else if (num_row != B.num_row)
        return false;

    for (int i = 0; i < num_elem; i++) {
        // check to see if values differ by more or less than EPS
        if ((*(pbody + i) - (*(B.pbody + i))) > EPS)
            return false;
        else if ((*(pbody + i) - (*(B.pbody + i))) < (-1. * EPS))
            return false;
    }
    return true;
}
///////////////////////////////////////////////////////////////////////////////
// Extracting components from vector with offset operator []
// returns the component(i) from vector VEC[i] or assigns a value to component
// VEC[i]
// Examples: comp_i=VEC[i]; VEC[i]=comp_i;
///////////////////////////////////////////////////////////////////////////////
double &Matrix::operator[](const int &r)
{
    assert(((r < num_row) && (num_col = 1)) &&
           " *** Error: invalid offset index in 'Matrix::operator[]' *** ");
    return *(pbody + r);
}
///////////////////////////////////////////////////////////////////////////////
// Scalar product operator (any combination of row or column vectors)
// Example: value = AMAT ^ BMAT;
///////////////////////////////////////////////////////////////////////////////
double Matrix::operator^(const Matrix &B)
{
    // initialize the result
    double result(0);

    // check dimensions
    bool one = false;
    bool dim = false;
    // true if both arrays have dimension '1'
    if ((num_row == 1 || num_col == 1) && (B.num_row == 1 || B.num_col == 1))
        one = true;
    // true if both arrays have at least one equal dimension
    if ((num_row == B.num_row || num_row == B.num_col) &&
        (num_col == B.num_col || num_col == B.num_row))
        dim = true;

    assert((one && dim) &&
           " *** Error: incompatible dimensions in 'Matrix::operator^()' *** ");

    for (int i = 0; i < num_row; i++)
        result += *(pbody + i) * (*(B.pbody + i));
    return result;
}
///////////////////////////////////////////////////////////////////////////////
// Unit vector cross product from two 3x1 vectors
// Example: UNIT = VEC1%VEC2;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator%(const Matrix &B)
{
    // initializing the result
    Matrix RESULT(3, 1);
    // local variables
    double v1(0);
    double v2(0);
    double v3(0);
    double dv(0);

    // check for proper dimensions
    assert((num_elem == 3 && B.num_elem == 3) &&
           " *** Error: incompatible dimensions in 'Matrix::operator%()' *** ");

    v1 = *(pbody + 1) * (*(B.pbody + 2)) - *(pbody + 2) * (*(B.pbody + 1));
    v2 = *(pbody + 2) * (*(B.pbody)) - *(pbody) * (*(B.pbody + 2));
    v3 = *(pbody) * (*(B.pbody + 1)) - *(pbody + 1) * (*(B.pbody));
    dv = sqrt(v1 * v1 + v2 * v2 + v3 * v3);

    // check for zero magnitude
    assert(dv != 0 &&
           " *** Error: divide by zero in 'Matrix::operator%()' *** ");

    RESULT.assign_loc(0, 0, v1 / dv);
    RESULT.assign_loc(1, 0, v2 / dv);
    RESULT.assign_loc(2, 0, v3 / dv);
    return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
// Alternate transpose Aji <- Aij
// Example: BMAT = ~AMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator~()
{
    Matrix RESULT(num_col, num_row);
    int i = 0;  // offset for original matrix
    int j = 0;  // offset for transposed matrix

    for (int r = 0; r < num_row; r++) {
        for (int c = 0; c < num_col; c++) {
            // offset for transposed
            j = c * num_row + r;
            *(RESULT.pbody + j) = *(pbody + i);
            i++;
            j++;
        }
    }
    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Returns polar from cartesian coordinates
// magnitude = POLAR[0] = |V|
// azimuth   = POLAR[1] = atan2(V2,V1)
// elevation = POLAR[2] = atan2(-V3,sqrt(V1^2+V2^2)
// Example: POLAR = VEC.pol_from_cart();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::pol_from_cart()
{
    double d = 0.0;
    double azimuth = 0.0;
    double elevation = 0.0;
    double denom;
    Matrix POLAR(3, 1);

    double v1 = (*pbody);
    double v2 = (*(pbody + 1));
    double v3 = (*(pbody + 2));

    d = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    azimuth = atan2(v2, v1);

    denom = sqrt(v1 * v1 + v2 * v2);
    if (denom > 0.)
        elevation = atan2(-v3, denom);
    else {
        if (v3 > 0)
            elevation = -PI / 2.;
        if (v3 < 0)
            elevation = PI / 2.;
        if (v3 == 0)
            elevation = 0.;
    }

    *POLAR.pbody = d;
    *(POLAR.pbody + 1) = azimuth;
    *(POLAR.pbody + 2) = elevation;

    return POLAR;
}
/*
///////////////////////////////////////////////////////////////////////////////
//Puts # of cols into private member Matrix::col_num of MAT
//Example: MAT.put_cols(cols);
///////////////////////////////////////////////////////////////////////////////
void Matrix::put_cols(int cols)
{
    num_col=cols;
}

///////////////////////////////////////////////////////////////////////////////
//Puts # of elems into private member Matrix::elem_num of MAT
//Example: MAT.put_elems(elems);
///////////////////////////////////////////////////////////////////////////////
void Matrix::put_elems(int elems)
{
    num_elem=elems;
}

///////////////////////////////////////////////////////////////////////////////
//Puts # of rows into private member Matrix::row_num of MAT
//Example: MAT.put_rows(rows);
///////////////////////////////////////////////////////////////////////////////
void Matrix::put_rows(int rows)
{
    num_row=rows;
}
*/
///////////////////////////////////////////////////////////////////////////////
// Returns row vector of row #
// Example: VEC = MAT.row_vec(2); (2nd row!)
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::row_vec(const int &row)
{
    assert((row > 0 && row <= num_row) &&
           " *** Error: row outside array in 'Matrix::row_vec()' *** ");

    Matrix RESULT(1, num_col);

    for (int i = 0; i < num_col; i++) {
        int offset = (row - 1) * num_col + i;
        *(RESULT.pbody + i) = (*(pbody + offset));
    }
    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Returns the skew-symmetric matrix from a 3-dim vector VEC
//            | 0 -c  b|        |a|
//            | c  0 -a| <--    |b|
//            |-b  a  0|        |c|
//
// Example: MAT = VEC.skew_sym();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::skew_sym()
{
    Matrix RESULT(3, 3);
    // check for proper dimensions
    assert((num_col == 1 && num_row == 3) &&
           " *** Error: not a 3x1 column vector in 'Matrix::skew_sym()' *** ");

    *(RESULT.pbody + 5) = -(*pbody);
    *(RESULT.pbody + 7) = (*pbody);
    *(RESULT.pbody + 2) = (*(pbody + 1));
    *(RESULT.pbody + 6) = -(*(pbody + 1));
    *(RESULT.pbody + 1) = -(*(pbody + 2));
    *(RESULT.pbody + 3) = (*(pbody + 2));

    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Returns the sub matrix after 'row'  and 'col' have been ommitted
// Example: BMAT = AMAT.sub_matrix(1,3); (deleting first row and third column!)
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::sub_matrix(const int &row, const int &col)
{
    assert(((row <= num_row) && (col <= num_col)) &&
           " *** Error: row or column outside array in 'Matrix::sub_matrix()' "
           "*** ");

    assert((row > 0 && col > 0) &&
           " *** Error: row/col are numbered not offset in "
           "'Matrix::sub_matrix()' *** ");

    // create return matrix
    Matrix RESULT(num_row - 1, num_col - 1);
    // start and stop of skipping matrix elements
    int skip_start = (row - 1) * num_col;
    int skip_end = skip_start + num_col;

    // initialize RESULT offset j
    int j = 0;

    for (int i = 0; i < num_elem; i++) {
        // skip elements of row to be removed
        if ((i < skip_start) || (i >= skip_end)) {
            // offset of column element to be removed
            int offset_col = (col - 1) + (i / num_col) * num_col;
            // skip elements of col to be removed
            if (i != offset_col) {
                *(RESULT.pbody + j) = *(pbody + i);
                j++;
            }
        }
    }
    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Transpose Aji <- Aij
// Example: BMAT = AMAT.trans();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::trans()
{
    Matrix RESULT(num_col, num_row);
    int i = 0;  // offset for original matrix
    int j = 0;  // offset for transposed matrix

    for (int r = 0; r < num_row; r++) {
        for (int c = 0; c < num_col; c++) {
            // offset for transposed
            j = c * num_row + r;
            *(RESULT.pbody + j) = *(pbody + i);
            i++;
            j++;
        }
    }
    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Returns unit vector from 3x1 vector
// Example: UVEC=VEC.univec3();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::univec3()
{
    Matrix RESULT(3, 1);
    // check for proper dimensions
    assert((num_col == 1 && num_row == 3) &&
           " *** Error: not a 3x1 column vector in 'Matrix::univec()' *** ");

    double v1 = (*pbody);
    double v2 = (*(pbody + 1));
    double v3 = (*(pbody + 2));
    double d = sqrt(v1 * v1 + v2 * v2 + v3 * v3);

    // if VEC is zero than the unit vector is also a zero vector
    if (d == 0) {
        *RESULT.pbody = 0;
        *(RESULT.pbody + 1) = 0;
        *(RESULT.pbody + 2) = 0;
    } else {
        *RESULT.pbody = v1 / d;
        *(RESULT.pbody + 1) = v2 / d;
        *(RESULT.pbody + 2) = v3 / d;
    }
    return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
// Returns 9x1 vector from 3x3 matrix row-wise
// Example: VEC=MAT.vec9_mat33();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::vec9_mat33()
{
    assert((num_row == 3 && num_col == 3) &&
           " *** Error: matrix not 3 x 3 in 'Matrix::vec9_mat33()' *** ");

    Matrix RESULT(9, 1);
    for (int i = 0; i < 9; i++) {
        *(RESULT.pbody + i) = *(pbody + i);
    }
    return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
// Forms a zero matrix MAT from object MAT(num_row,num_col)
// Example: MAT.zero();
///////////////////////////////////////////////////////////////////////////////
Matrix &Matrix::zero()
{
    for (int i = 0; i < num_elem; i++)
        *(pbody + i) = 0.0;

    return *this;
}
///////////////////////////////////////////////////////////////////////////////
////////////////// Module utility functions ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Returns the angle between two 3x1 vectors
// Example: theta=angle(VEC1,VEC2);
// 010824 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double angle(Matrix VEC1, Matrix VEC2)
{
    double argument;
    double scalar = VEC1 ^ VEC2;
    double abs1 = VEC1.absolute();
    double abs2 = VEC2.absolute();

    double dum = abs1 * abs2;
    if (abs1 * abs2 > EPS)
        argument = scalar / dum;
    else
        argument = 1.;
    if (argument > 1.)
        argument = 1.;
    if (argument < -1.)
        argument = -1.;

    return acos(argument);
}
////////////////////////////////////////////////////////////////////////////////
// Returns the T.M. of the psi->tht->phi sequence
// Euler angle transformation matrix of flight mechanics
//
// 011126 Created by Peter H Zipfel
// 170121 Add Armadillo version
////////////////////////////////////////////////////////////////////////////////

Matrix mat3tr(const double &psi, const double &tht, const double &phi)
{
    double spsi = sin(psi);
    double cpsi = cos(psi);
    double stht = sin(tht);
    double ctht = cos(tht);
    double sphi = sin(phi);
    double cphi = cos(phi);

    Matrix AMAT(3, 3);
    AMAT.assign_loc(0, 0, cpsi * ctht);
    AMAT.assign_loc(1, 0, cpsi * stht * sphi - spsi * cphi);
    AMAT.assign_loc(2, 0, cpsi * stht * cphi + spsi * sphi);
    AMAT.assign_loc(0, 1, spsi * ctht);
    AMAT.assign_loc(1, 1, spsi * stht * sphi + cpsi * cphi);
    AMAT.assign_loc(2, 1, spsi * stht * cphi - cpsi * sphi);
    AMAT.assign_loc(0, 2, -stht);
    AMAT.assign_loc(1, 2, ctht * sphi);
    AMAT.assign_loc(2, 2, ctht * cphi);

    return AMAT;
}

arma::mat33 build_euler_transform_matrix(const double &psi, const double &tht, const double &phi)
{
    double spsi = sin(psi);
    double cpsi = cos(psi);
    double stht = sin(tht);
    double ctht = cos(tht);
    double sphi = sin(phi);
    double cphi = cos(phi);

    arma::mat33 AMAT;
    AMAT(0, 0) = cpsi * ctht;
    AMAT(1, 0) = cpsi * stht * sphi - spsi * cphi;
    AMAT(2, 0) = cpsi * stht * cphi + spsi * sphi;
    AMAT(0, 1) = spsi * ctht;
    AMAT(1, 1) = spsi * stht * sphi + cpsi * cphi;
    AMAT(2, 1) = spsi * stht * cphi - cpsi * sphi;
    AMAT(0, 2) = -stht;
    AMAT(1, 2) = ctht * sphi;
    AMAT(2, 2) = ctht * cphi;

    return AMAT;
}
////////////////////////////////////////////////////////////////////////////////
// Projects initial state through 'tgo' to final state along a Keplerian
// trajectory
// Based on Ray Morth, unpublished utility
//
// Return output
//    kepler_flag = 0: good Kepler projection;
//                = 1: bad (# of iterations>20, or neg. sqrt), no new proj cal,
//                  use prev value;  - ND
// Parameter output
//    SPII = projected inertial position after tgo - m
//    VPII = projected inertial velocity after tgo - m/s
// Parameter input
//    SBII = current inertial position - m
//    VBII = current inertial velocity - m/s
//    tgo = time-to-go to projected point - sec
//
// 040319 Created from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
int cad_kepler(Matrix &SPII,
               Matrix &VPII,
               Matrix SBII,
               Matrix VBII,
               const double &tgo)
{
    // local variables
    double sde(0);
    double cde(0);
    int kepler_flag(0);

    double sqrt_GM = sqrt(GM);
    double ro = SBII.absolute();
    double vo = VBII.absolute();
    double rvo = SBII ^ VBII;
    double a1 = vo * vo / GM;
    double sa = ro / (2 - ro * a1);
    if (sa < 0) {
        // return without re-calculating SPII, VPII
        kepler_flag = 1;
        return kepler_flag;
    }
    double smua = sqrt_GM * sqrt(sa);
    double mdot = smua / (sa * sa);

    // calculating 'de'iteratively
    double dm = mdot * tgo;
    double de = dm;  // initialize eccentricity
    double a11 = rvo / smua;
    double a21 = (sa - ro) / sa;
    int count20 = 0;
    double adm(0);
    do {
        cde = 1 - cos(de);
        sde = sin(de);
        double dmn = de + a11 * cde - a21 * sde;
        double dmerr = dm - dmn;

        adm = fabs(dmerr) / mdot;
        double dmde = 1 + a11 * sde - a21 * (1 - cde);
        de = de + dmerr / dmde;
        count20++;
        if (count20 > 20) {
            // return without re-calculating SPII, VPII
            kepler_flag = 1;
            return kepler_flag;
        }
    } while (adm > SMALL);

    // projected position
    double fk = (ro - sa * cde) / ro;
    double gk = (dm + sde - de) / mdot;
    SPII = SBII * fk + VBII * gk;

    // projected velocity
    double rp = SPII.absolute();
    double fdk = -smua * sde / ro;
    double gdk = rp - sa * cde;
    VPII = SBII * (fdk / rp) + VBII * (gdk / rp);

    return kepler_flag;
}

//////////////////////////////////////////////////////////////////////////////
// Calculates geodetic longitude, latitude, and altitude from inertial
// displacement vector
// using the WGS 84 reference ellipsoid
// Reference: Britting,K.R."Inertial Navigation Systems Analysis", Wiley. 1971
//
// Parameter output
//    lon = geodetic longitude - rad
//    lat = geodetic latitude - rad
//    alt = altitude above ellipsoid - m
// Parameter input
//    lat = geodetic latitude - rad
//    SBII(3x1) = Inertial position - m
//
// 030414 Created from FORTRAN by Peter H Zipfel
// 170121 Create Armadillo Version by soncyang
///////////////////////////////////////////////////////////////////////////////
void cad_geo84_in(double &lon,
                  double &lat,
                  double &alt,
                  Matrix SBII,
                  const double &time)
{
    int count(0);
    double lat0(0);
    double alamda(0);

    // initializing geodetic latitude using geocentric latitude
    double dbi = SBII.absolute();
    double latg = asin(SBII.get_loc(2, 0) / dbi);
    lat = latg;

    // iterating to calculate geodetic latitude and altitude
    do {
        lat0 = lat;
        double r0 =
            SMAJOR_AXIS *
            (1. - FLATTENING * (1. - cos(2. * lat0)) / 2. +
             5. * pow(FLATTENING, 2) * (1. - cos(4. * lat0)) / 16.);  // eq 4-21
        alt = dbi - r0;
        double dd = FLATTENING * sin(2. * lat0) *
                    (1. - FLATTENING / 2. - alt / r0);  // eq 4-15
        lat = latg + dd;
        count++;
        assert(count <= 100 &&
               " *** Stop: Geodetic latitude does not "
               "converge,'cad_geo84_in()' *** ");

    } while (fabs(lat - lat0) > SMALL);

    // longitude
    double sbii1 = SBII.get_loc(0, 0);
    double sbii2 = SBII.get_loc(1, 0);
    double dum4 = asin(sbii2 / sqrt(sbii1 * sbii1 + sbii2 * sbii2));
    // Resolving the multi-valued arcsin function
    if ((sbii1 >= 0.0) && (sbii2 >= 0.0))
        alamda = dum4;  // quadrant I
    if ((sbii1 < 0.0) && (sbii2 >= 0.0))
        alamda = (180. * RAD) - dum4;  // quadrant II
    if ((sbii1 < 0.0) && (sbii2 < 0.0))
        alamda = (180. * RAD) - dum4;  // quadrant III
    if ((sbii1 > 0.0) && (sbii2 < 0.0))
        alamda = (360. * RAD) + dum4;  // quadrant IV
    lon = alamda - WEII3 * time - GW_CLONG;
    if ((lon) > (180. * RAD))
        lon = -((360. * RAD) - lon);  // east positive, west negative
}

///////////////////////////////////////////////////////////////////////////////
////////////////////  Integration functions  //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Integration of Matrix MAT(r,c)
//
// 030424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Matrix integrate(Matrix &DYDX_NEW,
                 Matrix &DYDX,
                 Matrix &Y,
                 const double int_step)
{
    int nrow = Y.get_rows();
    int nrow1 = DYDX_NEW.get_rows();
    int nrow2 = DYDX.get_rows();
    int ncol = Y.get_cols();
    int ncol1 = DYDX_NEW.get_cols();
    int ncol2 = DYDX.get_cols();

    assert((nrow == nrow1 && nrow == nrow2) &&
           " *** Error: incompatible row-dimensions in 'integrate()' *** ");
    assert((ncol == ncol1 && ncol == ncol2) &&
           " *** Error: incompatible column-dimensions in 'integrate()' *** ");

    Matrix RESULT(nrow, ncol);
    for (int r = 0; r < nrow; r++)
        for (int c = 0; c < ncol; c++)
            RESULT.assign_loc(
                r, c, integrate(DYDX_NEW.get_loc(r, c), DYDX.get_loc(r, c),
                                Y.get_loc(r, c), int_step));

    return RESULT;
}
