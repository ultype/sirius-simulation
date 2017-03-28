#ifndef GPS_CONSTELLATION_HH
#define GPS_CONSTELLATION_HH

/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the newton Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/rocket/GPS_constellation.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/

#include <armadillo>

#include "aux/aux.hh"

#include "Time_management.hh"
#include "aux/global_constants.hh"
#include "cad/utility.hh"

struct range_t
{
	GPS g;
	double range;
	double rate;
	double d;
	arma::vec2 azel;
	double iono_delay;
};

struct ephem_t
{
	int vflg;	/*!< Valid Flag */
	CALDATE t;
	GPS toc;	/*!< Time of Clock */
	GPS toe;	/*!< Time of Ephemeris */
	int iodc;	/*!< Issue of Data, Clock */
	int iode;	/*!< Isuse of Data, Ephemeris */
	double deltan;	/*!< Delta-N (radians/sec) */
	double cuc;	/*!< Cuc (radians) */
	double cus;	/*!< Cus (radians) */
	double cic;	/*!< Correction to inclination cos (radians) */
	double cis;	/*!< Correction to inclination sin (radians) */
	double crc;	/*!< Correction to radius cos (meters) */
	double crs;	/*!< Correction to radius sin (meters) */
	double ecc;	/*!< e Eccentricity */
	double sqrta;	/*!< sqrt(A) (sqrt(m)) */
	double m0;	/*!< Mean anamoly (radians) */
	double omg0;	/*!< Longitude of the ascending node (radians) */
	double inc0;	/*!< Inclination (radians) */
	double aop;
	double omgdot;	/*!< Omega dot (radians/s) */
	double idot;	/*!< IDOT (radians/s) */
	double af0;	/*!< Clock offset (seconds) */
	double af1;	/*!< rate (sec/sec) */
	double af2;	/*!< acceleration (sec/sec^2) */
	double tgd;	/*!< Group delay L2 bias */
	int svhlth;
	int codeL2;
	// Working variables follow
	double n; 	/*!< Mean motion (Average angular velocity) */
	double sq1e2;	/*!< sqrt(1-e^2) */
	double A;	/*!< Semi-major axis */
	double omgkdot; /*!< OmegaDot-OmegaEdot */
};

struct ionoutc_t
{
	int enable;
	int vflg;
	double alpha0,alpha1,alpha2,alpha3;
	double beta0,beta1,beta2,beta3;
	double A0,A1;
	int dtls,tot,wnt;
	int dtlsf,dn,wnlsf;
};

struct channel_t
{
	int prn;	/*< PRN Number */
	int ca[CA_SEQ_LEN]; /*< C/A Sequence */
	double f_carr;	/*< Carrier frequency */
	double f_code;	/*< Code frequency */
	unsigned int carr_phase; /*< Carrier phase */
	int carr_phasestep;	/*< Carrier phasestep */
	double code_phase; /*< Code phase */
	GPS g0;	/*!< GPS time at start */
	unsigned long sbf[5][N_DWRD_SBF]; /*!< current subframe */
	unsigned long dwrd[N_DWRD]; /*!< Data words of sub-frame */
	int iword;	/*!< initial word */
	int ibit;	/*!< initial bit */
	int icode;	/*!< initial code */
	int dataBit;	/*!< current data bit */
	int codeCA;	/*!< current C/A code */
	arma::vec2 azel;
	range_t rho0;
};


class GPS_constellation{
	
	TRICK_INTERFACE(GPS_constellation);
	public:
		
		GPS_constellation(time_management &time_ma, Newton &newt, Environment &env, Kinematics &kine);
		GPS_constellation(const GPS_constellation& other);
		GPS_constellation& operator= (const GPS_constellation& other);

		void readfile(const char *fname);
		void initialize();
		void compute();
		void show();

		template<class Archive>
	    void serialize(Archive & ar, const unsigned int version){
	        ar & neph;
	        ar & ieph;
	        ar & nsat;
	        ar & chan;
	        ar & ionoutc;
	        ar & eph;
	        // ar & _xyz;
	    }

	private:
		
		int readRinexNavAll(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname);
		int allocateChannel(channel_t *chan, ephem_t *eph, ionoutc_t ionoutc, GPS grx, arma::vec3 XYZ, double elvMask);
		int checkSatVisibility(ephem_t eph, GPS g, arma::vec3 xyz, double elvMask, arma::vec2 &azel);
		void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, GPS g, arma::vec3 XYZ);
		int replaceExpDesignator(char *str, int len);
		void satpos(ephem_t eph, GPS g, arma::vec3 &pos, arma::vec3 &vel, arma::vec2 &clk);
		void xyz2llh(const arma::vec3 xyz, arma::vec3 &llh);
		arma::vec3 llh2xyz(const arma::vec3 llh);
		void neu2azel(arma::vec2 &azel, const arma::vec3 neu);
		double ionosphericDelay(const ionoutc_t *ionoutc, GPS g, arma::vec3 llh, arma::vec2 azel);
		double subGpsTime(GPS g1, GPS g0);
		arma::mat33 ltcmat(const arma::vec3 llh);
		arma::mat33 enu2ecef(arma::vec3 llh);
		arma::mat33 enu2atenna(double phi, double tht, double psi);

		time_management *time;
		Newton *newton;
		Environment *environment;
		Kinematics *kinematics;
		
		int neph;   /* *io  (--)	number of ephemeris */
		int ieph;   /* *io  (--)	which ephemeris been select */
		
		int nsat;	/*	*io  (--)								*/
		channel_t chan[MAX_CHAN];	
		ionoutc_t ionoutc;			
		ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];	

		int allocatedSat[MAX_SAT]; /* *o (--)						*/

		// arma::vec azel;
		// double _azel[2];

		// arma::vec clk;
		// double _clk[2];

		// arma::vec llh;
		// double _llh[3];

		// arma::vec neu;
		// double _neu[3];

		// arma::vec pos;
		// double _pos[3];

		// arma::vec vel;
		// double _vel[3];

		// arma::vec los;
		// double _los[3];
};

#endif