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

#endif// utility_header__HPP
