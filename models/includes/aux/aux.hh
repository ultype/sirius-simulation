/********************************* TRICK HEADER *******************************
PURPOSE:
      (Helper Functions and MARCOs)
LIBRARY DEPENDENCY:
      ()
PROGRAMMERS:
      ((() () () () ))
ICG: (No)
*******************************************************************************/

#ifndef aux_header__HPP
#define aux_header__HPP

#define TRICK_INTERFACE(class_name) \
        friend class InputProcessor; \
        friend void init_attr ## class_name();

#define MATRIX_INIT(mat_name, n, m) \
        mat_name(&_ ## mat_name[0][0], n, m, false, true)

#define VECTOR_INIT(vec_name, n) \
        vec_name(&_ ## vec_name[0], n, false, true)

#define INTEGRATE_d(in, diff) \
        do{ \
            arma::mat in##d_new = diff; \
            in = integrate(in##d_new, in##d, in, int_step); \
            in##d = in##d_new; \
        }while(0)

#define INTEGRATE_D(in, diff) \
        do{ \
            arma::mat in##D_NEW = diff; \
            in = integrate(in##D_NEW, in##D, in, int_step); \
            in##D = in##D_NEW; \
        }while(0)

#endif// utility_header__HPP
