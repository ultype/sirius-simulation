/********************************* TRICK HEADER *******************************
PURPOSE:
      (Helper Functions migrated from CADAC++)
LIBRARY DEPENDENCY:
      ((../src/aux/utility_header.cpp))
PROGRAMMERS:
      ((() () () () ))
ICG: (No)
*******************************************************************************/

/**
 * \file utility_header.hpp
 */

//'Matrix' class member functions
// Module utility functions:
//    angle
//    sign
//    mat2tr
//    mat3tr
//    cad_distance
//    cad_geo84_in
//    cad_geo84vel_in
//    cad_geoc_in
//    cad_geoc_ine
//    cad_grav84
//    cad_in_geo84
//    cad_in_geoc
//    cad_in_orb
//    cad_kepler (based on Morth)
//    cad_kepler1 (based on Bate, Mueller & White)
//    cad_orb_in
//    cad_tdi84
//    cad_tei
//    cad_tge
//    cad_tgi84
//    cad_tip
// Stochastic functions
//    exponential
//    gauss
//    markov
//    rayleigh
//    uniform
//    unituni
// Integration
// US76 Atmosphere
// US76 Atmosphere extended to 1000km (NASA Marshall)
//
// 030319 Added US76 atmosphere, PZi
// 040311 US76 Atmosphere extended to 1000km (NASA Marshall), PZi

#define _CRT_SECURE_NO_DEPRECATE
#ifndef utility_header__HPP
#define utility_header__HPP

#include <armadillo>
#include <iostream>
#include <cmath>
#include "global_constants.hh"

using namespace std;

template<class _Matrix>
void print_matrix(_Matrix matrix) {
    matrix.print(std::cout);
}

template void print_matrix<arma::mat>(arma::mat matrix);

/**
 * \brief One dimensional and two dimensional arrays of any size of type
 *        'double'.
 *  dynamically allocated matrix size
 *  pointer arithmetic
 */
class Matrix
{
private:
    /// number of rows
    int num_row;
    /// number of columns
    int num_col;
    /// total number of elements
    int num_elem;
    /// pointer to array
    double *pbody;

public:
    /// default constructors
    Matrix();
    /// overloaded constructor
    Matrix(int row_size, int col_size);
    /// copy constructor
    Matrix(const Matrix &MAT);

    Matrix(const double v[3]);
    Matrix(const double v[][3]);
    /// deconstructor
    ~Matrix();

    /// printing matrix array to console
    void print();

    /// Returns absolute value of a vector VEC
    /// Example: value=VEC.absolute();
    double absolute();

    /// Returns the adjoint
    /// Example: BMAT = AMAT.adjoint();
    Matrix adjoint();

    /// Assigns a value to a matrix element (offset!)
    /// Example: MAT.assign_loc(r,c,val); ((r+1)th-row, (c+1)th-col)
    void assign_loc(const int &r, const int &c, const double &val);

    /// Builds a 3x1 vector from three parameters
    /// Example: VEC.build_vec3(v1,v2,v3)
    Matrix &build_vec3(const double &v1, const double &v2, const double &v3);
    Matrix &build_vec3(const double v[3]);

    /// Builds a 3x3 matrix from nine paramters arranged in rows
    /// Example: MAT.build_mat33(v11,v12,v13,v21,v22,v23,v31,v32,v33)
    Matrix &build_mat33(const double &v11,
                        const double &v12,
                        const double &v13,
                        const double &v21,
                        const double &v22,
                        const double &v23,
                        const double &v31,
                        const double &v32,
                        const double &v33);

    /// Builds a 3x3 matrix from double array[3][3]
    Matrix &build_mat33(const double v[][3]);

    // Fill in the Matrix data into intrinsic array;
    Matrix &fill(double v[3]);
    Matrix &fill(double m[][3]);

    /** Calculates Cartesian vector from polar coordinates.
     *  |V1|             | cos(elevation)*cos(azimuth)|
     *  |V2| = magnitude*|cos(elevation)*sin(azimuth) |
     *  |V3|             |      -sin(elevation)        |
     * Example: VEC.cart_from_pol(magnitude,azimuth,elevation);
     */
    Matrix &cart_from_pol(const double &magnitude,
                          const double &azimuth,
                          const double &elevation);

    /// Returns square root matrix of square matrix MAT
    /// Example: SQRTMAT=MAT.cholesky();
    Matrix cholesky();

    /// Returns column vector of column #
    /// Example: VEC = MAT.col_vec(2); (2nd column!)
    Matrix col_vec(const int &col);

    /// Returns the determinant
    /// Example: det = MAT.determinant();
    double determinant();

    /// Returns nxn diagonal matrix  from nx1 vector
    /// Example: DIAMAT=VEC.diamat_vec()
    Matrix diamat_vec();

    /// Returns nx1 diagonal vector from nxn matrix
    /// Example: VEC=MAT.diavec_mat();
    Matrix diavec_mat();

    /// Dimensions a matrix of size row x col
    /// Example: MAT.dimension(3,3);
    void dimension(int row, int col);

    /** Bi-variate ellipse.
     *  calculating major and minor semi-axes of ellipse and rotation angle
     *  from the symmetrical pos semi-definite MAT(2x2) matrix
     *  major_semi_axis = ELLIPSE.get_loc(0,0);
     *  minor_semi_axis = ELLIPSE.get_loc(1,0);
     *  angle      = ELLIPSE.get_loc(2,0);
     *
     * Example: ELLIPSE = MAT.ellipse();
     */
    Matrix ellipse();

    /// Returns the number of columns of matrix MAT
    /// Example: nc = MAT.get_cols();
    int get_cols();

    /// Returns offset index given row# and col#
    /// Example: i = MAT.get_index(2,3); (2nd row, 3rd coloumn)
    int get_index(const int &row, const int &col);

    /// Returns the value at offset-row 'r' offset-col 'c' of MAT
    /// Example: value = MAT.get_loc(2,1); (3rd row, 2nd column)
    double get_loc(const int &r, const int &c);

    /// Returns the pointer to MAT
    /// Example: ptr = MAT.get_pbody();
    double *get_pbody();

    /// Returns the number of rows in the matrix
    /// Example: nr = MAT.get_rows();
    int get_rows();

    /// Builds a square identity matrix of object 'Matrix MAT'
    Matrix &identity();

    /// Returns the inverse of a square matrix AMAT
    /// Example: INVERSE = AMAT.inverse();
    Matrix inverse();

    /// Returns 3x3 matrix row-wise from 9x1 vector
    /// Example: MAT=VEC.    mat33_vec9();
    Matrix mat33_vec9();

    /// Forms  matrix MAT with all elements '1.' from object
    /// MAT(num_row,num_col)
    /// Example: MAT.ones();
    Matrix &ones();

    /** Inequality relational operator.
     * @return true if elements differ by more than EPS
     * Example: if(AMAT!=BMAT){...};
     */
    bool operator!=(const Matrix &B);

    /** Scalar multiplication operator (scalar element by element
     *  multiplication).
     * \note scalar must be the second operand
     * Example: CMAT = AMAT * b;
     */
    Matrix operator*(const double &b);

    /** Multiplication operator, returns matrix product.
     *  Example: CMAT = AMAT * BMAT;
     */
    Matrix operator*(const Matrix &B);

    /** Scalar multiplication assignment operator (scalar element by
     *  element multiplication).
     * Example: AMAT *= b; meaning: AMAT = AMAT * b
     */
    Matrix &operator*=(const double &b);

    /** Scalar division operator.
     * Example: CMAT = AMAT / b;
     */
    Matrix operator/(const double &b);

    /// Multiplication assignment operator
    /// Example: AMAT *= BMAT; meaning: AMAT = AMAT * BMAT;
    Matrix &operator*=(const Matrix &B);

    /** Scalar Addition operator (scalar element by element addition).
     * \note: scalar must be the second operand
     *  Example: CMAT = AMAT + b;
     */
    Matrix operator+(const double &b);

    /** Addition operator, returns matrix addition
     * Example: CMAT = AMAT + BMAT;
     */
    Matrix operator+(const Matrix &B);

    /** Scalar addition assignment operator (scalar element by
     *  element addition).
     * Example: AMAT += b; meaning: AMAT = AMAT + b
     */
    Matrix &operator+=(const double &b);

    /** Matrix addition assignment operator.
     * Example: AMAT += BMAT; meaning: AMAT = AMAT + BMAT;
     */
    Matrix &operator+=(const Matrix &B);

    /** Scalar substraction operator (scalar element by
     *  element substraction).
     * \note scalar must be the second operand
     * Example: CMAT = AMAT - b;
     */
    Matrix operator-(const double &b);

    /** Substraction operator, returns matrix substraction.
     * Example: CMAT = AMAT - BMAT;
     */
    Matrix operator-(const Matrix &B);

    /** Scalar substraction assignment operator (scalar element by
     * element substraction).
     * Example: AMAT -= b; meaning: AMAT = AMAT - b
     */
    Matrix &operator-=(const double &b);

    /** Matrix subtraction assignment operator.
     * Example: AMAT -= BMAT; meaning: AMAT = AMAT - BMAT;
     */
    Matrix &operator-=(const Matrix &B);

    /** Assignment operator (deep copy).
     * Example: AMAT = BMAT; also: AMAT = BMAT = CMAT;
     */
    Matrix &operator=(const Matrix &B);

    /** Equality relational operator.
     * @return true if elements differ by less than EPS
     * Example: if(AMAT==BMAT){...};
     */
    bool operator==(const Matrix &B);

    /** Returns the component(i) from vector VEC[i] or assigns a value to
     *  component VEC[i]
     * Examples: comp_i=VEC[i]; VEC[i]=comp_i;
     */
    double &operator[](const int &r);

    /** Scalar product operator (any combination of row or column vectors).
     * Example: value = AMAT ^ BMAT;
     */
    double operator^(const Matrix &B);

    /** Unit vector cross product from two 3x1 vectors.
     * Example: UNIT = VEC1%VEC2;
     */
    Matrix operator%(const Matrix &B);

    /** Alternate transpose Aij=>Aji.
     * Example: BMAT = ~AMAT;
     */
    Matrix operator~();

    /** Returns polar from cartesian coordinates.
     * magnitude = POLAR(0,0) = |V|
     * azimuth   = POLAR(1,0) = atan2(V2,V1)
     * elevation = POLAR(2,0) = atan2(-V3,sqrt(V1^2+V2^2)
     * Example: POLAR = VEC.pol_from_cart();
     */
    Matrix pol_from_cart();

    /** Returns row vector of row #.
     * Example: VEC = MAT.row_vec(2); (2nd row!)
     */
    Matrix row_vec(const int &row);

    /** Returns the skew-symmetric matrix from a 3-dim vector VEC.
     *            | 0 -c  b|        |a|
     *            | c  0 -a| <--    |b|
     *            |-b  a  0|        |c|
     *
     * Example: MAT = VEC.skew_sym();
     */
    Matrix skew_sym();

    /** Returns the sub matrix after 'row'  and 'col' have been ommitted.
     * Example: BMAT = AMAT.sub_matrix(1,3); (deleting first row and third
     *                                        column!)
     */
    Matrix sub_matrix(const int &row, const int &col);

    /** Returns the stranspose of a matrix Aji <- Aij.
     * Example: BMAT = AMAT.trans();
     */
    Matrix trans();

    /** Returns unit vector from 3x1 vector.
     * Example: UVEC=VEC.univec3();
     */
    Matrix univec3();

    /** Forms a zero matrix MAT from object MAT(num_row,num_col).
     * Example: MAT.zero();
     */
    Matrix &zero();

    /** Returns 9x1 vector from 3x3 matrix row-wise.
     * Example: VEC=MAT.vec9_mat33();
     */
    Matrix vec9_mat33();
};

// Module utility functions

/// @return the sign of the function
int sign(const double &variable);

/// @return the angle between two 3x1 vectors
double angle(Matrix VEC1, Matrix VEC2);

/// @return skew symmetric matrix of a Vector3
arma::mat33 skew_sym(arma::vec3 vec);

/// @return the T.M. of the psivg -> thtvg sequence
Matrix mat2tr(const double &psivg, const double &thtvg);
arma::mat33 build_transform_matrix(const double &psivg, const double &thtvg);

/// @return the Euler T.M. of the psi->tht->phi sequence
Matrix mat3tr(const double &psi, const double &tht, const double &phi);
arma::mat33 build_euler_transform_matrix(const double &psi, const double &tht, const double &phi);

/// @return great circle distance between two point on a spherical Earth
double cad_distance(const double &lon1,
                    const double &lat1,
                    const double &lon2,
                    const double &lat2);

/** Calculates geodetic longitude, latitude, and altitude from inertial
 *  displacment vector.
 */
void cad_geo84_in(double &lon,
                  double &lat,
                  double &alt,
                  const Matrix SBII,
                  const double &time);

void arma_cad_geo84_in(double &lon,
                  double &lat,
                  double &alt,
                  arma::vec3 SBII,
                  const double &time);

/// Calculates geodetic velocity vector from inertial states
void cad_geo84vel_in(double &dvbe,
                     double &psivdx,
                     double &thtvdx,
                     Matrix SBII,
                     Matrix VBII,
                     const double &time);

/** Calculates geocentric lon, lat, alt from inertial displacement vector,
 *  spherical Earth
 */
void cad_geoc_in(double &lonc,
                 double &latc,
                 double &altc,
                 Matrix SBII,
                 const double &time);

void arma_cad_geoc_in(double &lonc,
                 double &latc,
                 double &altc,
                 arma::vec3 SBII,
                 const double &time);

/// @return geocentric lon, lat, alt from displacement vector in Earth coord
Matrix cad_geoc_ine(Matrix SBIE);

/// @return Earth gravitational acceleration in geocentric coord, using the
///         WGS 84 ellipsoid
Matrix cad_grav84(Matrix SBII, const double &time);

arma::vec3 arma_cad_grav84(arma::vec3 SBII, const double &time);

/// @return the inertial diplacement vector from longitude, latitude and
///         altitude (WGS84)
Matrix cad_in_geo84(const double lon,
                    const double lat,
                    const double alt,
                    const double &time);

arma::vec3 arma_cad_in_geo84(const double lon,
                             const double lat,
                             const double alt,
                             const double &time);

/// @return the inertial displacement vector from geocentric longitude,
///         latitude and altitude
Matrix cad_in_geoc(const double &lon,
                   const double &lat,
                   const double &alt,
                   const double &time);

/// Calculates inertial displacement and velocity vectors from orbital elements
int cad_in_orb(Matrix &SBII,
               Matrix &VBII,
               const double &semi,
               const double &ecc,
               const double &inclx,
               const double &lon_anodex,
               const double &arg_perix,
               const double &true_anomx);

/** Projects initial state through 'tgo' to final state along a
 *  Keplerian trajectory
 */
int cad_kepler(Matrix &SPII,
               Matrix &VPII,
               Matrix SBII,
               Matrix VBII,
               const double &tgo);

/** Projects initial state through 'tgo' to final state along a
 *  Keplerian trajectory
 */
int cad_kepler1(Matrix &SPII,
                Matrix &VPII,
                const Matrix SBII,
                const Matrix VBII,
                const double &tgo);

/// Calculates utility functions c(z) and s(z) for cad_kepler()
void cadkepler1_ucs(double &c, double &s, const double &z);

/// Calculates the orbital elements from inertial displacement and velocity
int cad_orb_in(double &semi,
               double &ecc,
               double &inclx,
               double &lon_anodex,
               double &arg_perix,
               double &true_anomx,
               Matrix &SBII,
               Matrix &VBII);

int arma_cad_orb_in(double &semi,
                   double &ecc,
                   double &inclx,
                   double &lon_anodex,
                   double &arg_perix,
                   double &true_anomx,
                   arma::vec3 &SBII,
                   arma::vec3 &VBII);

/// @return the T.M. of geodetic wrt inertial coordinates
Matrix cad_tdi84(const double &lon,
                 const double &lat,
                 const double &alt,
                 const double &time);

arma::mat33 arma_cad_tdi84(const double &lon,
                           const double &lat,
                           const double &alt,
                           const double &time);

/// @return the T.M. of earth wrt inertial coordinates
Matrix cad_tei(const double &time);

/// @return the T.M. of geographic wrt earth coordinates
Matrix cad_tge(const double &lon, const double &lat);

/// @return the T.M. of geographic (geocentric) wrt inertial
Matrix cad_tgi84(const double &lon,
                 const double &lat,
                 const double &alt,
                 const double &time);

arma::mat33 arma_cad_tgi84(const double &lon,
                           const double &lat,
                           const double &alt,
                           const double &time);

/// @return the transformation matrix of inertial wrt perifocal coordinates
Matrix cad_tip(const double &incl,
               const double &lon_anode,
               const double &arg_peri);

arma::vec Matrix2Quaternion(arma::mat Matrix_in);
arma::mat Quaternion2Matrix(arma::vec Quaternion_in);
/* Stochastic functions */

/**
 * \brief Generating an exponential distribution with a given mean density.
 * Ref:
 *   Tybrin, "CADAC Program documentation", June 2000 and source code CADX3.FOR
 *   Numerical Recipies, p 287, 1992 Cambridge University Press
 * Function unituni() is a CADAC++ utility
 *   parameter input:
 *     density = # of events per unit of variable (in the mean)
 * return output:
 *     value = units of variable to be traversed until next event occurs
 *
 * The variance is density^2
 */
double exponential(double density);

/**
 * \brief Generating a standard distribution with 'mean' and 'sig' std
 *        deviation.
 * Ref Numerical Recipies, p 289, 1992 Cambridge University Press
 * Function unituni() is a CADAC++ utility
 *
 * parameter input:
 *   min = standard deviation of Gaussian distribution - unit of variable
 *   mean = mean value of Gaussian distribution - unit of variable
 * return output:
 *   value = value of variable - unit of variable
 */
double gauss(double mean, double sig);

/**
 * \brief Generating a time-correlated Gaussian variable with zero mean.
 * Ref: CADAC Subroutine CNT_GAUSS
 * Function gauss() is CADAC++ utility
 *
 * parameter input:
 *   sigma = standard deviation of Gaussian distribution - unit of variable
 *   bcor = beta time correlation coefficient - 1/s (Hz)
 *   time = simulation time - s
 *   intstep = integration step size - s
 * return output:
 *   value = value of variable - unit of variable
 */
double markov(double sigma,
              double bcor,
              double time,
              double intstep,
              double &value_saved);

/**
 * \brief Generating a Rayleigh distribution with peak value of pdf = 'mode'.
 * Ref: Tybrin, "CADAC Program documentation", June 2000 and source code
 *      CADX3.FOR
 * Function unituni() is a CADAC++ utility
 *
 * parameter input:
 *   mode= mode (peak value of pdf) of Rayleigh distribution - unit of variable
 * return output:
 *   value=value of variable - unit of variable
 *
 * The mean of the distribution is: mean = mode * (pi/2)
 * The variance is: variance = mode^2 * (2 - pi/2)
 */
double rayleigh(double mode);

/**
 * \brief Generating uniform random distribution between 'min' and 'max'.
 */
double uniform(double min, double max);

/**
 * \brief Generating uniform random distribution between 0-1 based on
 *        C function rand().
 */
double unituni();

/**
 * \brief Integration of scalar state variable.
 * Modified Euler Midpoint method
 * Example first order lag:
 *   phid_new=(phic-phi)/tphi;
 *   phi=integrate(phid_new,phid,phi,int_step);
 *   phid=phid_new;
 */
double integrate(const double &dydx_new,
                 const double &dydx,
                 const double &y,
                 const double &int_step);

/**
 * \brief Integration of Matrix MAT(r,c).
 */
Matrix integrate(Matrix &DYDX_NEW,
                 Matrix &DYDX,
                 Matrix &Y,
                 const double int_step);

arma::mat integrate(arma::mat &DYDX_NEW,
                 arma::mat &DYDX,
                 arma::mat &Y,
                 const double int_step);

arma::vec integrate(arma::vec &DYDX_NEW, 
                arma::vec &DYDX, arma::vec &Y, const double int_step);
/**
 * \brief US Standard Atmosphere 1976.
 *
 * Calculates the atmospheric properties density, pressure and temperature
 * up to 85 km.
 */
void atmosphere76(double &rho, double &press, double &tempk, const double balt);

/**
 * \brief Calculates the atmospheric properties density, pressure and
 *        temperature up to 1000 km (NASA Marshall).
 */
int us76_nasa2002(double alt_km,
                  double *rho,
                  double *press,
                  double *tempk,
                  double *vsound);


class Table
{
    private:
        string name;    //table name
        int dim;    //table dimension (1,2, or 3)
        int var1_dim;   // variable 1 dimension
        int var2_dim;   // variable 2 dimension
        int var3_dim;   // variable 3 dimension
    public:
        double *var1_values;  //values of variable 1
        double *var2_values;  //values of variable 2
        double *var3_values;  //values of variable 3
        double *data; // table data values packaged in one array

        Table(){}
        virtual ~Table()
        {
            delete [] var1_values;
            delete [] var2_values;
            delete [] var3_values;
            delete [] data;
        }

        ///////////////////////////////////////////////////////////////////////////
        //Getting dimension of table
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        int get_dim(){return dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Getting name of table
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        string get_name(){return name;}

        ///////////////////////////////////////////////////////////////////////////
        //Getting 1. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        int get_var1_dim(){return var1_dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Getting 2. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        int get_var2_dim(){return var2_dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Getting 3. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        int get_var3_dim(){return var3_dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting dimension of table
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_dim(int table_dim){dim=table_dim;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting name of table
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_name(string tbl_name){name=tbl_name;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting 1. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var1_dim(int size){var1_dim=size;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting 2. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var2_dim(int size){var2_dim=size;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting 3. independent variable dimension
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var3_dim(int size){var3_dim=size;}

        ///////////////////////////////////////////////////////////////////////////
        //Setting 1. independent variable values
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var1_value(int offset,double value){
            var1_values[offset]=value;
        }
        ///////////////////////////////////////////////////////////////////////////
        //Setting 2. independent variable values
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var2_value(int offset,double value){
            var2_values[offset]=value;
        }
        ///////////////////////////////////////////////////////////////////////////
        //Setting 3. independent variable values
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_var3_value(int offset,double value){
            var3_values[offset]=value;
        }
        ///////////////////////////////////////////////////////////////////////////
        //Setting tablular data values
        //030710 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////
        void set_data(int offset,double value){
            data[offset]=value;
        }
};

class Datadeck
{
    private:
        string title; //title of data deck
        int capacity; //total number of tables
        int tbl_counter; //table counter
        Table **table_ptr; //table_ptr is pointer to a pointer array of type 'Table'

    public:

        Datadeck(){}
        virtual ~Datadeck(){}

        ///////////////////////////////////////////////////////////////////////////////
        //Allocating memory  table deck title
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void alloc_mem(){table_ptr=new Table *[capacity];}

        ///////////////////////////////////////////////////////////////////////////////
        //Setting table deck title
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void set_title(string deck_title){title=deck_title;}

        ///////////////////////////////////////////////////////////////////////////////
        //Getting table deck title
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        string get_title(){return title;}

        ///////////////////////////////////////////////////////////////////////////////
        //Setting total number of tables
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void set_capacity(int table_numbers){capacity=table_numbers;}

        ///////////////////////////////////////////////////////////////////////////////
        //Getting total number of tables
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        int get_capacity(){return capacity;}

        ///////////////////////////////////////////////////////////////////////////////
        //Setting table counter
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void set_counter(int count){tbl_counter=count;}

        ///////////////////////////////////////////////////////////////////////////////
        //Getting table counter
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        int get_counter(){return tbl_counter;}

        ///////////////////////////////////////////////////////////////////////////////
        //Adding a table pointer to the table list
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        void add_table(Table &pt)
        {
            if(tbl_counter<capacity){
                table_ptr[tbl_counter]=&pt;
            }
        }
        ///////////////////////////////////////////////////////////////////////////////
        //Overloaded operator [] returns a 'Table' pointer
        //030711 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        Table * operator[](int slot)
        {
            if(slot>=0 && slot<capacity)
                return table_ptr[slot];
            else
            {
                cout<<"*** Bad pointer value of table deck: "<<slot<<'\n';
                return 0;
            }
        }

        ///////////////////////////////////////////////////////////////////////////////
        //Getting table pointer
        //030717 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        Table * get_tbl(int slot){return table_ptr[slot];}

        ///////////////////////////////////////////////////////////////////////////////
        //Single independent variable look-up
        //constant extrapolation at the upper end, slope extrapolation at the lower end
        //
        //030717 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double look_up(string name,double value1);

        ///////////////////////////////////////////////////////////////////////////////
        //Two independent variables look-up
        // Constant extrapolation at the upper end, slope extrapolation at the lower end
        //
        //030717 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double look_up(string name,double value1,double value2);

        ///////////////////////////////////////////////////////////////////////////////
        //Three independent variables look-up
        //constant extrapolation at the upper end, slope extrapolation at the lower end
        //
        //030723 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double look_up(string name,double value1,double value2,double value3);

        ///////////////////////////////////////////////////////////////////////////////
        //Table index finder
        //This is a binary search method it is O(lgN)
        //* Returns array locater of the discrete_variable just below variable
        //* Keeps max or min array locater if variable is outside those max or min
        //
        //010628 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        int find_index(int max,double value,double *list);

        ///////////////////////////////////////////////////////////////////////////////
        //Linear one-dimensional interpolation
        // Constant extrapolation beyond max values of X1
        // Slope extrapolation beyond min values of X1
        //
        //030717 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double interpolate(int ind,int ind2,int slot,double val);

        ///////////////////////////////////////////////////////////////////////////////
        //Linear, two-dimensional interpolation
        // Constant extrapolation beyond max values of X1 and X2
        // Slope extrapolation beyond min values of X1 and X2
        //
        //030718 Created by Peter H Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double interpolate(int ind10,int ind11,int ind20,int ind21,int slot,double value1,
                            double value2);

        ///////////////////////////////////////////////////////////////////////////////
        //Linear, three-dimensional interpolation
        //Constant extrapolation beyond max values of X1, X2 and X3
        //Slope extrapolation beyond min values of X1, X2 and X3
        //
        //030723 Created by Peter Zipfel
        ///////////////////////////////////////////////////////////////////////////////
        double interpolate(int ind10,int ind11,int ind20,int ind21,int ind30,int ind31,
                                     int slot,double value1,double value2,double value3);

};

extern void read_tables(char *file_name,Datadeck &datatable);
#endif
