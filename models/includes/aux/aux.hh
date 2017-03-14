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

#define TRICK_INTERFACE(class_name) \
        friend class InputProcessor; \
        friend void init_attr ## class_name();

#define MATRIX_INIT(mat_name, n, m) \
        mat_name(&_ ## mat_name[0][0], n, m, false, true)

#define VECTOR_INIT(vec_name, n) \
        vec_name(&_ ## vec_name[0], n, false, true)

#define LINK(model, func) \
        std::bind(&std::remove_reference<decltype(model)>::type::func, &model)

#endif// utility_header__HPP
