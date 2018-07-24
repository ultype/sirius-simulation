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

#include <type_traits>
#include "sim_services/include/simtime.h"


#define TRICK_INTERFACE(class_name) \
        friend class InputProcessor; \
        friend void init_attr ## class_name();

#define MATRIX_INIT(mat_name, n, m) \
        mat_name(&_ ## mat_name[0][0], n, m, false, true)

#define VECTOR_INIT(vec_name, n) \
        vec_name(&_ ## vec_name[0], n, false, true)

#define LINK(model, func) \
        std::bind(&std::remove_reference<decltype(model)>::type::func, &model)

#define LINKARG(model, func, arg) \
        std::bind(&std::remove_reference<decltype(model)>::type::func, &model, arg)

#define EXPORT(model, func) \
        #model, #func, std::bind(&std::remove_reference<decltype(model)>::type::func, &model)

#define ECIO_EXPORT(model, func) \
        #model, #func, std::bind(&Ecio::func, &ecio)

#define IMPORT(model, func) \
        #model, #func

inline double get_elapsed_time() { return get_rettime(); }

#define MATRIX_BIND(mat_name, n, m) \
        mat_name = gsl_matrix_view_array(_ ## mat_name, n, m)

#define VECTOR_BIND(vec_name, n) \
        vec_name = gsl_vector_view_array(_ ## vec_name, n)
#endif  // utility_header__HPP
