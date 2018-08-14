#ifndef __CAD_UTILC_HH__
#define __CAD_UTILC_HH__

/********************************* TRICK HEADER *******************************
PURPOSE:
      (CAD Utility functions C Version)
LIBRARY DEPENDENCY:
      ((../src/cad_utility_c.c)
        (../src/global_constants.c))
*******************************************************************************/

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif
/** Global constants **/
extern const double __FLATTENING;
extern const double __SMAJOR_AXIS;
extern const double __SMALL;
extern const double __RAD;
extern const double __GM;

/** Functions **/
int cad_in_geo84(const double lon, const double lat
                    , const double alt, const gsl_matrix *TEI, gsl_vector *SBII);
int cad_tdi84(const double lon, const double lat, const double alt, const gsl_matrix *TEI, gsl_matrix *TDI);
int cad_geo84_in(const gsl_vector *SBII, const gsl_matrix *TEI, double *lon, double *lat, double *alt);
int cad_kepler(const gsl_vector *SBII, const gsl_vector *VBII, const double tgo
                , gsl_vector *SPII, gsl_vector *VPII);
#ifdef __cplusplus
}
#endif
#endif  //  __CAD_UTILC_HH__