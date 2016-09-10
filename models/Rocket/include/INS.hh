#ifndef __INS_HH__
#define __INS_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the INS Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Ins.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

class INS {
    public:
    INS() {}

    void default_data();
    void initialize();

    double IPos[3];     /* *i (m)       Body in Inertial Coordinate */
    double IVel[3];     /* *i (m)       Body in Inertial Coordinate */
    double IW[3];       /* *i (m)       Body in Inertial Coordinate */
};

#endif  // __INS_HH__
