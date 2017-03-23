#include "rocket/GPS_constellation.hh"

GPS_constellation::GPS_constellation(time_management &time_ma, Newton &newt, Environment &env)
:	time(&time_ma),
	newton(&newt),
	environment(&env)
{}
GPS_constellation::GPS_constellation(const GPS_constellation& other)
:	time(other.time),
	newton(other.newton),
	environment(other.environment)
{	
	this->neph = other.neph;
	this->ieph = other.ieph;
	this->nsat = other.nsat;
}
GPS_constellation& GPS_constellation::operator= (const GPS_constellation& other){
	if(&other == this)
    return *this;
	this->time = other.time;
	this->newton = other.newton;
	this->environment = other.environment;
	this->neph = other.neph;
	this->ieph = other.ieph;
	this->nsat = other.nsat;	

	return *this;
}
//***********************************************************************************************/
// https://cddis.nasa.gov/Data_and_Derived_Products/GNSS/broadcast_ephemeris_data.html#GPShourly
//***********************************************************************************************/
void GPS_constellation::readfile(const char *fname)
{
	neph = readRinexNavAll(eph, &ionoutc, fname);
}

void GPS_constellation::initialize()
{
	ieph = -1;
	int sv(0);
	double dt;
	//Select Which ephemeris will be used
    for(int i = 0;i<neph;i++){
        for(sv = 0;sv<MAX_SAT;sv++){
            if(eph[i][sv].vflg == 1){
                dt = subGpsTime(time->gpstime, eph[i][sv].toc);
                if(dt>=-SECONDS_IN_HOUR && dt<SECONDS_IN_HOUR){
                    ieph = i;
                    break;
                }
            }
        }
        if(ieph>=0){
            break;
        }
    }
    // Clear all channels
	for (int i=0; i<MAX_CHAN; i++)
		chan[i].prn = 0;

	// Clear satellite allocation flag
	for (sv=0; sv<MAX_SAT; sv++)
		allocatedSat[sv] = -1;

	ionoutc.enable = TRUE;
}

void GPS_constellation::compute()
{
	unsigned int sv(0);
	range_t rho;
	arma::vec3 SBEE = newton->get_SBEE();
	nsat = allocateChannel(chan, eph[ieph], ionoutc, time->gpstime, SBEE, 10.0);
                for(int i = 0;i<MAX_CHAN;i++){
                    if(chan[i].prn>0){
                        sv = chan[i].prn-1;
                        computeRange(&rho, eph[ieph][sv], &ionoutc, time->gpstime, SBEE);
                        chan[i].rho0=rho;
                        chan[i].azel(0)=rho.azel(0);
                        chan[i].azel(1)=rho.azel(1);
                    }
                }
}

void GPS_constellation::show()
{
	cout<<"Lock number ="<<nsat<<'\t'<<"GPS SOW ="<<time->gpstime.SOW<<endl;
                cout<<"/*****************************************************************************/"<<endl;
                for(int i = 0;i<MAX_CHAN;i++){
                    if(chan[i].prn>0){
                        cout<<chan[i].prn<<'\t'<<(chan[i].azel(0))*DEG<<'\t'<<(chan[i].azel(1))*DEG<<'\t'<<chan[i].rho0.d
                        <<'\t'<<chan[i].rho0.iono_delay<<'\t'<<chan[i].rho0.range<<'\t'<<chan[i].rho0.rate<<endl;
                    }
                }
                cout<<"/*****************************************************************************/"<<endl;
}

int GPS_constellation::readRinexNavAll(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname)
{
	FILE *fp;
	int ieph;
	
	int sv;
	char str[MAX_CHAR];
	char tmp[20];

	CALDATE t;
	GPS g;
	GPS g0;
	double dt;

	int flags = 0x0;

	if (NULL==(fp=fopen(fname, "rt")))
		return(-1);

	// Clear valid flag
	for (ieph=0; ieph<EPHEM_ARRAY_SIZE; ieph++)
		for (sv=0; sv<MAX_SAT; sv++)
			eph[ieph][sv].vflg = 0;

	// Read header lines
	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		if (strncmp(str+60, "END OF HEADER", 13)==0)
			break;
		else if (strncmp(str+60, "ION ALPHA", 9)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->alpha3 = atof(tmp);

			flags |= 0x1;
		}
		else if (strncmp(str+60, "ION BETA", 8)==0)
		{
			strncpy(tmp, str+2, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta0 = atof(tmp);

			strncpy(tmp, str+14, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta1 = atof(tmp);

			strncpy(tmp, str+26, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta2 = atof(tmp);

			strncpy(tmp, str+38, 12);
			tmp[12] = 0;
			replaceExpDesignator(tmp, 12);
			ionoutc->beta3 = atof(tmp);

			flags |= 0x1<<1;
		}
		else if (strncmp(str+60, "DELTA-UTC", 9)==0)
		{
			strncpy(tmp, str+3, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A0 = atof(tmp);

			strncpy(tmp, str+22, 19);
			tmp[19] = 0;
			replaceExpDesignator(tmp, 19);
			ionoutc->A1 = atof(tmp);

			strncpy(tmp, str+41, 9);
			tmp[9] = 0;
			ionoutc->tot = atoi(tmp);

			strncpy(tmp, str+50, 9);
			tmp[9] = 0;
			ionoutc->wnt = atoi(tmp);

			if (ionoutc->tot%4096==0)
				flags |= 0x1<<2;
		}
		else if (strncmp(str+60, "LEAP SECONDS", 12)==0)
		{
			strncpy(tmp, str, 6);
			tmp[6] = 0;
			ionoutc->dtls = atoi(tmp);

			flags |= 0x1<<3;
		}
	}

	ionoutc->vflg = FALSE;
	if (flags==0xF) // Read all Iono/UTC lines
		ionoutc->vflg = TRUE;

	// Read ephemeris blocks
	g0.Week = -1;
	ieph = 0;

	while (1)
	{
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		// PRN
		strncpy(tmp, str, 2);
		tmp[2] = 0;
		sv = atoi(tmp)-1;

		// EPOCH
		strncpy(tmp, str+3, 2);
		tmp[2] = 0;
		t.Year = atoi(tmp) + 2000;

		strncpy(tmp, str+6, 2);
		tmp[2] = 0;
		t.Month = atoi(tmp);

		strncpy(tmp, str+9, 2);
		tmp[2] = 0;
		t.Day = atoi(tmp);

		strncpy(tmp, str+12, 2);
		tmp[2] = 0;
		t.Hour = atoi(tmp);

		strncpy(tmp, str+15, 2);
		tmp[2] = 0;
		t.Min = atoi(tmp);

		strncpy(tmp, str+18, 4);
		tmp[2] = 0;
		t.Sec = atof(tmp);

		time->utc_to_gps(&t, &g);
		
		if (g0.Week==-1)
			g0 = g;

		// Check current time of clock
		dt = subGpsTime(g, g0);
		
		if (dt>SECONDS_IN_HOUR)
		{
			g0 = g;
			ieph++; // a new set of ephemerides

			if (ieph>=EPHEM_ARRAY_SIZE)
				break;
		}

		// Date and time
		eph[ieph][sv].t = t;

		// SV CLK
		eph[ieph][sv].toc = g;

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19); // tmp[15]='E';
	eph[ieph][sv].af0 = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af1 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].af2 = atof(tmp);

		// BROADCAST ORBIT - 1
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iode = (int)atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crs = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].deltan = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].m0 = atof(tmp);

		// BROADCAST ORBIT - 2
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cuc = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].ecc = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cus = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].sqrta = atof(tmp);

		// BROADCAST ORBIT - 3
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.SOW = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cic = atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omg0 = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].cis = atof(tmp);

		// BROADCAST ORBIT - 4
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].inc0 = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].crc = atof(tmp);
		
		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].aop = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].omgdot = atof(tmp);

		// BROADCAST ORBIT - 5
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+3, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].idot = atof(tmp);

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].codeL2 = (int)atof(tmp);

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].toe.Week = (int)atof(tmp);

		// BROADCAST ORBIT - 6
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		strncpy(tmp, str+22, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].svhlth = (int)atof(tmp);
		if ((eph[ieph][sv].svhlth>0) && (eph[ieph][sv].svhlth<32))
			eph[ieph][sv].svhlth += 32; // Set MSB to 1

		strncpy(tmp, str+41, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].tgd = atof(tmp);

		strncpy(tmp, str+60, 19);
		tmp[19] = 0;
		replaceExpDesignator(tmp, 19);
		eph[ieph][sv].iodc = (int)atof(tmp);

		// BROADCAST ORBIT - 7
		if (NULL==fgets(str, MAX_CHAR, fp))
			break;

		// Set valid flag
		eph[ieph][sv].vflg = 1;

		// Update the working variables
		eph[ieph][sv].A = eph[ieph][sv].sqrta * eph[ieph][sv].sqrta;
		eph[ieph][sv].n = sqrt(GM/(eph[ieph][sv].A*eph[ieph][sv].A*eph[ieph][sv].A)) + eph[ieph][sv].deltan;
		eph[ieph][sv].sq1e2 = sqrt(1.0 - eph[ieph][sv].ecc*eph[ieph][sv].ecc);
		eph[ieph][sv].omgkdot = eph[ieph][sv].omgdot - WEII3;
	}

	fclose(fp);
	
	if (g0.Week>=0)
		ieph += 1; // Number of sets of ephemerides

	return(ieph);
}

int GPS_constellation::allocateChannel(channel_t *chan, ephem_t *eph, ionoutc_t ionoutc, GPS grx, arma::vec3 XYZ, double elvMask)
{
	nsat = 0;
	int i,sv;
	arma::vec2 azel;

	range_t rho;

	double r_ref,r_xyz;
	double phase_ini;

	for (sv=0; sv<MAX_SAT; sv++)
	{
		if(checkSatVisibility(eph[sv], grx, XYZ, elvMask, azel)==1)
		{
			nsat++; // Number of visible satellites

			if (allocatedSat[sv]==-1) // Visible but not allocated
			{
				// Allocated new satellite
				for (i=0; i<MAX_CHAN; i++)
				{
					if (chan[i].prn==0)
					{
						// Initialize channel
						chan[i].prn = sv+1;
						chan[i].azel(0) = azel(0);
						chan[i].azel(1) = azel(1);

						// // C/A code generation
						// codegen(chan[i].ca, chan[i].prn);

						// // Generate subframe
						// eph2sbf(eph[sv], ionoutc, chan[i].sbf);

						// // Generate navigation message
						// generateNavMsg(grx, &chan[i], 1);

						// Initialize pseudorange
						computeRange(&rho, eph[sv], &ionoutc, grx, XYZ);
						chan[i].rho0 = rho;	
						// Initialize carrier phase
						r_xyz = rho.range;

						// computeRange(&rho, eph[sv], &ionoutc, grx, ref);
						// r_ref = rho.range;

						// phase_ini = (2.0*r_ref - r_xyz)/LAMBDA_L1;
						// phase_ini -= floor(phase_ini);
						// chan[i].carr_phase = (unsigned int)(512 * 65536.0 * phase_ini);

						// Done.
						break;
					}
				}

				// Set satellite allocation channel
				if (i<MAX_CHAN)
					allocatedSat[sv] = i;
			}
		}
		else if (allocatedSat[sv]>=0) // Not visible but allocated
		{
			// Clear channel
			chan[allocatedSat[sv]].prn = 0;

			// Clear satellite allocation flag
			allocatedSat[sv] = -1;
		}
	}

	return(nsat);
}

int GPS_constellation::checkSatVisibility(ephem_t eph, GPS g, arma::vec3 xyz, double elvMask, arma::vec2 &azel)
{
	arma::vec3 llh;
	arma::vec3 neu;
	arma::vec3 pos;
	arma::vec3 vel;
	arma::vec2 clk;
	arma::vec3 los;
	arma::mat33 tmat;

	if (eph.vflg != 1)
		return (-1); // Invalid

	xyz2llh(xyz,llh);
	tmat = ltcmat(llh);

	satpos(eph, g, pos, vel, clk);
	los = pos - xyz;
	neu = tmat * los;
	neu2azel(azel, neu);
	if (azel(1)*DEG > elvMask)
		return (1); // Visible
	// else
	return (0); // Invisible
}

/*! \brief Compute range between a satellite and the receiver
 *  \param[out] rho The computed range
 *  \param[in] eph Ephemeris data of the satellite
 *  \param[in] g GPS time at time of receiving the signal
 *  \param[in] xyz position of the receiver
 */
void GPS_constellation::computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, GPS g, arma::vec3 xyz)
{
	arma::vec3 pos;
	arma::vec3 vel;
	arma::vec2 clk;
	arma::vec3 los;
	double tau;
	double range,rate;
	double xrot,yrot;

	arma::vec3 llh;
	arma::vec3 neu;
	arma::mat33 tmat;
	
	// SV position at time of the pseudorange observation.
	satpos(eph, g, pos, vel, clk);

	// Receiver to satellite vector and light-time.
	los = pos - xyz;
	tau = norm(los)/SPEED_OF_LIGHT;

	// Extrapolate the satellite position backwards to the transmission time.
	pos(0) -= vel(0)*tau;
	pos(1) -= vel(1)*tau;
	pos(2) -= vel(2)*tau;

	// Earth rotation correction. The change in velocity can be neglected.
	xrot = pos(0) + pos(1)*WEII3*tau;
	yrot = pos(1) - pos(0)*WEII3*tau;
	pos(0) = xrot;
	pos(1) = yrot;

	// New observer to satellite vector and satellite range.
	los =  pos - xyz;
	range = norm(los);
	rho->d = range;

	// Pseudorange.
	rho->range = range - SPEED_OF_LIGHT*clk(0);

	// Relative velocity of SV and receiver.
	rate = dot(vel, los)/range;

	// Pseudorange rate.
	rho->rate = rate; // - SPEED_OF_LIGHT*clk[1];

	// Time of application.
	rho->g = g;

	// Azimuth and elevation angles.
	xyz2llh(xyz, llh);
	tmat = ltcmat(llh);
	neu = tmat * los;
	// ecef2neu(los, tmat, neu);
	neu2azel(rho->azel, neu);

	// Add ionospheric delay
	rho->iono_delay = ionosphericDelay(ionoutc, g, llh, rho->azel);
	rho->range += rho->iono_delay;

}

int GPS_constellation::replaceExpDesignator(char *str, int len)
{
	int i,n=0;

	for (i=0; i<len; i++)
	{
		if (str[i]=='D')
		{
			n++;
			str[i] = 'E';
		}
	}
	
	return(n);
}

void GPS_constellation::satpos(ephem_t eph, GPS g, arma::vec3 &pos, arma::vec3 &vel, arma::vec2 &clk)
{
	// Computing Satellite Velocity using the Broadcast Ephemeris
	// http://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm

	double tk;
	double mk;
	double ek;
	double ekold;
	double ekdot;
	double cek,sek;
	double pk;
	double pkdot;
	double c2pk,s2pk;
	double uk;
	double ukdot;
	double cuk,suk;
	double ok;
	double sok,cok;
	double ik;
	double ikdot;
	double sik,cik;
	double rk;
	double rkdot;
	double xpk,ypk;
	double xpkdot,ypkdot;

	double relativistic, OneMinusecosE, tmp;

	tk = g.SOW - eph.toe.SOW;

	if(tk>(SEC_PER_WEEK/2))
		tk -= SEC_PER_WEEK;
	else if(tk<-(SEC_PER_WEEK/2))
		tk += SEC_PER_WEEK;

	mk = eph.m0 + eph.n*tk;
	ek = mk;
	ekold = ek + 1.0;
  
	OneMinusecosE = 0; // Suppress the uninitialized warning.
	while(fabs(ek-ekold)>1.0E-14)
	{
		ekold = ek;
		OneMinusecosE = 1.0-eph.ecc*cos(ekold);
		ek = ek + (mk-ekold+eph.ecc*sin(ekold))/OneMinusecosE;
	}

	sek = sin(ek);
	cek = cos(ek);

	ekdot = eph.n/OneMinusecosE;

	relativistic = -4.442807633E-10*eph.ecc*eph.sqrta*sek;

	pk = atan2(eph.sq1e2*sek,cek-eph.ecc) + eph.aop;
	pkdot = eph.sq1e2*ekdot/OneMinusecosE;

	s2pk = sin(2.0*pk);
	c2pk = cos(2.0*pk);

	uk = pk + eph.cus*s2pk + eph.cuc*c2pk;
	suk = sin(uk);
	cuk = cos(uk);
	ukdot = pkdot*(1.0 + 2.0*(eph.cus*c2pk - eph.cuc*s2pk));

	rk = eph.A*OneMinusecosE + eph.crc*c2pk + eph.crs*s2pk;
	rkdot = eph.A*eph.ecc*sek*ekdot + 2.0*pkdot*(eph.crs*c2pk - eph.crc*s2pk);

	ik = eph.inc0 + eph.idot*tk + eph.cic*c2pk + eph.cis*s2pk;
	sik = sin(ik);
	cik = cos(ik);
	ikdot = eph.idot + 2.0*pkdot*(eph.cis*c2pk - eph.cic*s2pk);

	xpk = rk*cuk;
	ypk = rk*suk;
	xpkdot = rkdot*cuk - ypk*ukdot;
	ypkdot = rkdot*suk + xpk*ukdot;

	ok = eph.omg0 + tk*eph.omgkdot - WEII3*eph.toe.SOW;
	sok = sin(ok);
	cok = cos(ok);

	pos(0) = xpk*cok - ypk*cik*sok;
	pos(1) = xpk*sok + ypk*cik*cok;
	pos(2) = ypk*sik;

	tmp = ypkdot*cik - ypk*sik*ikdot;

	vel(0)= -eph.omgkdot*pos(1) + xpkdot*cok - tmp*sok;
	vel(1) = eph.omgkdot*pos(0) + xpkdot*sok + tmp*cok;
	vel(2) = ypk*cik*ikdot + ypkdot*sik;

	// Satellite clock correction
	tk = g.SOW - eph.toc.SOW;

	if(tk>(SEC_PER_WEEK/2))
		tk -= SEC_PER_WEEK;
	else if(tk<-(SEC_PER_WEEK/2))
		tk += SEC_PER_WEEK;

	clk(0) = eph.af0 + tk*(eph.af1 + tk*eph.af2) + relativistic - eph.tgd;  
	clk(1) = eph.af1 + 2.0*tk*eph.af2; 

}

void GPS_constellation::xyz2llh(const arma::vec3 xyz, arma::vec3 &llh)
{
	double a,eps,e,e2;
	double x,y,z;
	double rho2,dz,zdz,nh,slat,n,dz_new;

	a = SMAJOR_AXIS;
	e = WGS84_ECCENTRICITY;

	eps = 1.0e-3;
	e2 = e*e;

	if (norm(xyz)<eps)
	{
		// Invalid ECEF vector
		llh(0) = 0.0;
		llh(1) = 0.0;
		llh(2) = -a;

		return;
	}

	x = xyz(0);
	y = xyz(1);
	z = xyz(2);

	rho2 = x*x + y*y;
	dz = e2*z;

	while (1)
	{
		zdz = z + dz;
		nh = sqrt(rho2 + zdz*zdz);
		slat = zdz / nh;
		n = a / sqrt(1.0-e2*slat*slat);
		dz_new = n*e2*slat;

		if (fabs(dz-dz_new) < eps)
			break;

		dz = dz_new;
	}

	llh(0) = atan2(zdz, sqrt(rho2));
	llh(1) = atan2(y, x);
	llh(2) = nh - n;

}

/*! \brief Convert Lat/Long/Height into Earth-centered Earth-fixed (ECEF)
 *  \param[in] llh Input Array of Latitude, Longitude and Height
 *  \param[out] xyz Output Array of X, Y and Z ECEF coordinates
 */
arma::vec3 GPS_constellation::llh2xyz(const arma::vec3 llh)
{
	double n;
	double a;
	double e;
	double e2;
	double clat;
	double slat;
	double clon;
	double slon;
	double d,nph;
	double tmp;
	arma::vec3 xyz;

	a = SMAJOR_AXIS;
	e = WGS84_ECCENTRICITY;
	e2 = e*e;

	clat = cos(llh(0));
	slat = sin(llh(0));
	clon = cos(llh(1));
	slon = sin(llh(1));
	d = e*slat;

	n = a/sqrt(1.0-d*d);
	nph = n + llh(2);

	tmp = nph*clat;
	xyz(0) = tmp*clon;
	xyz(1) = tmp*slon;
	xyz(2) = ((1.0-e2)*n + llh(2))*slat;
	
	return xyz;
}

// /*! \brief Compute the intermediate matrix for LLH to ECEF
//  *  \param[in] llh Input position in Latitude-Longitude-Height format
//  *  \param[out] t Three-by-Three output matrix
//  */
arma::mat33 GPS_constellation::ltcmat(const arma::vec3 llh)
{
	double slat, clat;
	double slon, clon;
	arma::mat33 t;
	slat = sin(llh(0));
	clat = cos(llh(0));
	slon = sin(llh(1));
	clon = cos(llh(1));

	t(0,0) = -slat*clon;
	t(0,1) = -slat*slon;
	t(0,2) = clat;
	t(1,0) = -slon;
	t(1,1) = clon;
	t(1,2) = 0.0;
	t(2,0) = clat*clon;
	t(2,1) = clat*slon;
	t(2,2) = slat;

	return t;
}

/*! \brief Convert Earth-centered Earth-Fixed to ?
 *  \param[in] xyz Input position as vector in ECEF format
 *  \param[in] t Intermediate matrix computed by \ref ltcmat
 *  \param[out] neu Output position as North-East-Up format
 */
// void gps_con::ecef2neu(const double *xyz, double t[3][3], double *neu)
// {
// 	neu[0] = t[0][0]*xyz[0] + t[0][1]*xyz[1] + t[0][2]*xyz[2];
// 	neu[1] = t[1][0]*xyz[0] + t[1][1]*xyz[1] + t[1][2]*xyz[2];
// 	neu[2] = t[2][0]*xyz[0] + t[2][1]*xyz[1] + t[2][2]*xyz[2];

// 	return;
// }

/*! \brief Convert North-Eeast-Up to Azimuth + Elevation
 *  \param[in] neu Input position in North-East-Up format
 *  \param[out] azel Output array of azimuth + elevation as double
 */
void GPS_constellation::neu2azel(arma::vec2 &azel, const arma::vec3 neu)
{
	double ne;

	azel(0) = atan2(neu(1),neu(0));
	if (azel(0)<0.0)
		azel(0) += (2.0*PI);

	ne = sqrt(neu(0)*neu(0) + neu(1)*neu(1));
	azel(1) = atan2(neu(2), ne);
}

double GPS_constellation::ionosphericDelay(const ionoutc_t *ionoutc, GPS g, arma::vec3 llh, arma::vec2 azel)
{
	double iono_delay = 0.0;
	double E,phi_u,lam_u,F;

	if (ionoutc->enable==FALSE)
		return (0.0); // No ionospheric delay

	E = azel(1)/PI;
	phi_u = llh(0)/PI;
	lam_u = llh(1)/PI;

	// Obliquity factor
	F = 1.0 + 16.0*pow((0.53 - E),3.0);

	if (ionoutc->vflg==FALSE)
		iono_delay = F*5.0e-9*SPEED_OF_LIGHT;
	else
	{
		double t,psi,phi_i,lam_i,phi_m,phi_m2,phi_m3;
		double AMP,PER,X,X2,X4;

		// Earth's central angle between the user position and the earth projection of
		// ionospheric intersection point (semi-circles)
		psi = 0.0137/(E + 0.11) - 0.022;
		
		// Geodetic latitude of the earth projection of the ionospheric intersection point
		// (semi-circles)
		phi_i = phi_u + psi*cos(azel(0));
		if(phi_i>0.416)
			phi_i = 0.416;
		else if(phi_i<-0.416)
			phi_i = -0.416;

		// Geodetic longitude of the earth projection of the ionospheric intersection point
		// (semi-circles)
		lam_i = lam_u + psi*sin(azel(0))/cos(phi_i*PI);

		// Geomagnetic latitude of the earth projection of the ionospheric intersection
		// point (mean ionospheric height assumed 350 km) (semi-circles)
		phi_m = phi_i + 0.064*cos((lam_i - 1.617)*PI);
		phi_m2 = phi_m*phi_m;
		phi_m3 = phi_m2*phi_m;

		AMP = ionoutc->alpha0 + ionoutc->alpha1*phi_m
			+ ionoutc->alpha2*phi_m2 + ionoutc->alpha3*phi_m3;
		if (AMP<0.0)
			AMP = 0.0;

		PER = ionoutc->beta0 + ionoutc->beta1*phi_m
			+ ionoutc->beta2*phi_m2 + ionoutc->beta3*phi_m3;
		if (PER<72000.0)
			PER = 72000.0;

		// Local time (sec)
		t = SECONDS_IN_DAY/2.0*lam_i + g.SOW;
		while(t>=SECONDS_IN_DAY)
			t -= SECONDS_IN_DAY;
		while(t<0)
			t += SECONDS_IN_DAY;

		// Phase (radians)
		X = 2.0*PI*(t - 50400.0)/PER;

		if(fabs(X)<1.57)
		{
			X2 = X*X;
			X4 = X2*X2;
			iono_delay = F*(5.0e-9 + AMP*(1.0 - X2/2.0 + X4/24.0))*SPEED_OF_LIGHT;
		}
		else
			iono_delay = F*5.0e-9*SPEED_OF_LIGHT;
	}

	return (iono_delay);
}


double GPS_constellation::subGpsTime(GPS g1, GPS g0)
{
	double dt;

	dt = g1.SOW - g0.SOW;
	dt += (double)(g1.Week - g0.Week) * SEC_PER_WEEK;

	return(dt);
}



// void gps_con::codegen(int *ca, int prn)
// {
// 	int delay[] = {
// 		  5,   6,   7,   8,  17,  18, 139, 140, 141, 251,
// 		252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
// 		473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
// 		861, 862};
	
// 	int g1[CA_SEQ_LEN], g2[CA_SEQ_LEN];
// 	int r1[N_DWRD_SBF], r2[N_DWRD_SBF];
// 	int c1, c2;
// 	int i,j;

// 	if (prn<1 || prn>32)
// 		return;

// 	for (i=0; i<N_DWRD_SBF; i++)
// 		r1[i] = r2[i] = -1;

// 	for (i=0; i<CA_SEQ_LEN; i++)
// 	{
// 		g1[i] = r1[9];
// 		g2[i] = r2[9];
// 		c1 = r1[2]*r1[9];
// 		c2 = r2[1]*r2[2]*r2[5]*r2[7]*r2[8]*r2[9];

// 		for (j=9; j>0; j--) 
// 		{
// 			r1[j] = r1[j-1];
// 			r2[j] = r2[j-1];
// 		}
// 		r1[0] = c1;
// 		r2[0] = c2;
// 	}

// 	for (i=0,j=CA_SEQ_LEN-delay[prn-1]; i<CA_SEQ_LEN; i++,j++)
// 		ca[i] = (1-g1[i]*g2[j%CA_SEQ_LEN])/2;
	
// 	return;
// }

// /*! \brief Compute Subframe from Ephemeris
//  *  \param[in] eph Ephemeris of given SV
//  *  \param[out] sbf Array of five sub-frames, 10 long words each
//  */
// void gps_con::eph2sbf(const ephem_t eph, const ionoutc_t ionoutc, unsigned long sbf[5][N_DWRD_SBF])
// {
// 	unsigned long wn;
// 	unsigned long toe;
// 	unsigned long toc;
// 	unsigned long iode;
// 	unsigned long iodc;
// 	long deltan;
// 	long cuc;
// 	long cus;
// 	long cic;
// 	long cis;
// 	long crc;
// 	long crs;
// 	unsigned long ecc;
// 	unsigned long sqrta;
// 	long m0;
// 	long omg0;
// 	long inc0;
// 	long aop;
// 	long omgdot;
// 	long idot;
// 	long af0;
// 	long af1;
// 	long af2;
// 	long tgd;
// 	int svhlth;
// 	int codeL2;

// 	unsigned long ura = 0UL;
// 	unsigned long dataId = 1UL;
// 	unsigned long sbf4_page25_svId = 63UL;
// 	unsigned long sbf5_page25_svId = 51UL;

// 	unsigned long wna;
// 	unsigned long toa;

// 	signed long alpha0,alpha1,alpha2,alpha3;
// 	signed long beta0,beta1,beta2,beta3;
// 	signed long A0,A1;
// 	signed long dtls,dtlsf;
// 	unsigned long tot,wnt,wnlsf,dn;
// 	unsigned long sbf4_page18_svId = 56UL;

// 	// FIXED: This has to be the "transmission" week number, not for the ephemeris reference time
// 	//wn = (unsigned long)(eph.toe.week%1024);
// 	wn = 0UL;
// 	toe = (unsigned long)(eph.toe.SOW/16.0);
// 	toc = (unsigned long)(eph.toc.SOW/16.0);
// 	iode = (unsigned long)(eph.iode);
// 	iodc = (unsigned long)(eph.iodc);
// 	deltan = (long)(eph.deltan/POW2_M43/PI);
// 	cuc = (long)(eph.cuc/POW2_M29);
// 	cus = (long)(eph.cus/POW2_M29);
// 	cic = (long)(eph.cic/POW2_M29);
// 	cis = (long)(eph.cis/POW2_M29);
// 	crc = (long)(eph.crc/POW2_M5);
// 	crs = (long)(eph.crs/POW2_M5);
// 	ecc = (unsigned long)(eph.ecc/POW2_M33);
// 	sqrta = (unsigned long)(eph.sqrta/POW2_M19);
// 	m0 = (long)(eph.m0/POW2_M31/PI);
// 	omg0 = (long)(eph.omg0/POW2_M31/PI);
// 	inc0 = (long)(eph.inc0/POW2_M31/PI);
// 	aop = (long)(eph.aop/POW2_M31/PI);
// 	omgdot = (long)(eph.omgdot/POW2_M43/PI);
// 	idot = (long)(eph.idot/POW2_M43/PI);
// 	af0 = (long)(eph.af0/POW2_M31);
// 	af1 = (long)(eph.af1/POW2_M43);
// 	af2 = (long)(eph.af2/POW2_M55);
// 	tgd = (long)(eph.tgd/POW2_M31);
// 	svhlth = (unsigned long)(eph.svhlth);
// 	codeL2 = (unsigned long)(eph.codeL2);

// 	wna = (unsigned long)(eph.toe.Week%256);
// 	toa = (unsigned long)(eph.toe.SOW/4096.0);

// 	alpha0 = (signed long)round(ionoutc.alpha0/POW2_M30);
// 	alpha1 = (signed long)round(ionoutc.alpha1/POW2_M27);
// 	alpha2 = (signed long)round(ionoutc.alpha2/POW2_M24);
// 	alpha3 = (signed long)round(ionoutc.alpha3/POW2_M24);
// 	beta0 = (signed long)round(ionoutc.beta0/2048.0);
// 	beta1 = (signed long)round(ionoutc.beta1/16384.0);
// 	beta2 = (signed long)round(ionoutc.beta2/65536.0);
// 	beta3 = (signed long)round(ionoutc.beta3/65536.0);
// 	A0 = (signed long)round(ionoutc.A0/POW2_M30);
// 	A1 = (signed long)round(ionoutc.A1/POW2_M50);
// 	dtls = (signed long)(ionoutc.dtls);
// 	tot = (unsigned long)(ionoutc.tot/4096);
// 	wnt = (unsigned long)(ionoutc.wnt%256);
// 	// TO DO: Specify scheduled leap seconds in command options
// 	// 2016/12/31 (Sat) -> WNlsf = 1929, DN = 7 (http://navigationservices.agi.com/GNSSWeb/)
// 	// Days are counted from 1 to 7 (Sunday is 1).
// 	wnlsf = 1929%256;
// 	dn = 7;
// 	dtlsf = 18;

// 	// Subframe 1
// 	sbf[0][0] = 0x8B0000UL<<6;
// 	sbf[0][1] = 0x1UL<<8;
// 	sbf[0][2] = ((wn&0x3FFUL)<<20) | ((codeL2&0x3UL)<<18) | ((ura&0xFUL)<<14) | ((svhlth&0x3FUL)<<8) | (((iodc>>8)&0x3UL)<<6);
// 	sbf[0][3] = 0UL;
// 	sbf[0][4] = 0UL;
// 	sbf[0][5] = 0UL;
// 	sbf[0][6] = (tgd&0xFFUL)<<6;
// 	sbf[0][7] = ((iodc&0xFFUL)<<22) | ((toc&0xFFFFUL)<<6);
// 	sbf[0][8] = ((af2&0xFFUL)<<22) | ((af1&0xFFFFUL)<<6);
// 	sbf[0][9] = (af0&0x3FFFFFUL)<<8;

// 	// Subframe 2
// 	sbf[1][0] = 0x8B0000UL<<6;
// 	sbf[1][1] = 0x2UL<<8;
// 	sbf[1][2] = ((iode&0xFFUL)<<22) | ((crs&0xFFFFUL)<<6);
// 	sbf[1][3] = ((deltan&0xFFFFUL)<<14) | (((m0>>24)&0xFFUL)<<6);
// 	sbf[1][4] = (m0&0xFFFFFFUL)<<6;
// 	sbf[1][5] = ((cuc&0xFFFFUL)<<14) | (((ecc>>24)&0xFFUL)<<6);
// 	sbf[1][6] = (ecc&0xFFFFFFUL)<<6;
// 	sbf[1][7] = ((cus&0xFFFFUL)<<14) | (((sqrta>>24)&0xFFUL)<<6);
// 	sbf[1][8] = (sqrta&0xFFFFFFUL)<<6;
// 	sbf[1][9] = (toe&0xFFFFUL)<<14;

// 	// Subframe 3
// 	sbf[2][0] = 0x8B0000UL<<6;
// 	sbf[2][1] = 0x3UL<<8;
// 	sbf[2][2] = ((cic&0xFFFFUL)<<14) | (((omg0>>24)&0xFFUL)<<6);
// 	sbf[2][3] = (omg0&0xFFFFFFUL)<<6;
// 	sbf[2][4] = ((cis&0xFFFFUL)<<14) | (((inc0>>24)&0xFFUL)<<6);
// 	sbf[2][5] = (inc0&0xFFFFFFUL)<<6;
// 	sbf[2][6] = ((crc&0xFFFFUL)<<14) | (((aop>>24)&0xFFUL)<<6);
// 	sbf[2][7] = (aop&0xFFFFFFUL)<<6;
// 	sbf[2][8] = (omgdot&0xFFFFFFUL)<<6;
// 	sbf[2][9] = ((iode&0xFFUL)<<22) | ((idot&0x3FFFUL)<<8);

// 	if (ionoutc.vflg==TRUE)
// 	{
// 		// Subframe 4, page 18
// 		sbf[3][0] = 0x8B0000UL<<6;
// 		sbf[3][1] = 0x4UL<<8;
// 		sbf[3][2] = (dataId<<28) | (sbf4_page18_svId<<22) | ((alpha0&0xFFUL)<<14) | ((alpha1&0xFFUL)<<6);
// 		sbf[3][3] = ((alpha2&0xFFUL)<<22) | ((alpha3&0xFFUL)<<14) | ((beta0&0xFFUL)<<6);
// 		sbf[3][4] = ((beta1&0xFFUL)<<22) | ((beta2&0xFFUL)<<14) | ((beta3&0xFFUL)<<6);
// 		sbf[3][5] = (A1&0xFFFFFFUL)<<6;
// 		sbf[3][6] = ((A0>>8)&0xFFFFFFUL)<<6;
// 		sbf[3][7] = ((A0&0xFFUL)<<22) | ((tot&0xFFUL)<<14) | ((wnt&0xFFUL)<<6);
// 		sbf[3][8] = ((dtls&0xFFUL)<<22) | ((wnlsf&0xFFUL)<<14) | ((dn&0xFFUL)<<6);
// 		sbf[3][9] = (dtlsf&0xFFUL)<<22;
	
// 	}
// 	else
// 	{
// 		// Subframe 4, page 25
// 		sbf[3][0] = 0x8B0000UL<<6;
// 		sbf[3][1] = 0x4UL<<8;
// 		sbf[3][2] = (dataId<<28) | (sbf4_page25_svId<<22);
// 		sbf[3][3] = 0UL;
// 		sbf[3][4] = 0UL;
// 		sbf[3][5] = 0UL;
// 		sbf[3][6] = 0UL;
// 		sbf[3][7] = 0UL;
// 		sbf[3][8] = 0UL;
// 		sbf[3][9] = 0UL;
// 	}

// 	// Subframe 5, page 25
// 	sbf[4][0] = 0x8B0000UL<<6;
// 	sbf[4][1] = 0x5UL<<8;
// 	sbf[4][2] = (dataId<<28) | (sbf5_page25_svId<<22) | ((toa&0xFFUL)<<14) | ((wna&0xFFUL)<<6);
// 	sbf[4][3] = 0UL;
// 	sbf[4][4] = 0UL;
// 	sbf[4][5] = 0UL;
// 	sbf[4][6] = 0UL;
// 	sbf[4][7] = 0UL;
// 	sbf[4][8] = 0UL;
// 	sbf[4][9] = 0UL;

// 	return;
// }

// int gps_con::generateNavMsg(GPS g, channel_t *chan, int init)
// {
// 	int iwrd,isbf;
// 	GPS g0;
// 	unsigned long wn,tow;
// 	unsigned sbfwrd;
// 	unsigned long prevwrd;
// 	int nib;

// 	g0.Week = g.Week;
// 	g0.SOW = (double)(((unsigned long)(g.SOW+0.5))/30UL) * 30.0; // Align with the full frame length = 30 sec
// 	chan->g0 = g0; // Data bit reference time

// 	wn = (unsigned long)(g0.Week%1024);
// 	tow = ((unsigned long)g0.SOW)/6UL;

// 	if (init==1) // Initialize subframe 5
// 	{
// 		prevwrd = 0UL;

// 		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
// 		{
// 			sbfwrd = chan->sbf[4][iwrd];

// 			// Add TOW-count message into HOW
// 			if (iwrd==1)
// 				sbfwrd |= ((tow&0x1FFFFUL)<<13);

// 			// Compute checksum
// 			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
// 			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
// 			chan->dwrd[iwrd] = computeChecksum(sbfwrd, nib);

// 			prevwrd = chan->dwrd[iwrd];
// 		}
// 	}
// 	else // Save subframe 5
// 	{
// 		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
// 		{
// 			chan->dwrd[iwrd] = chan->dwrd[N_DWRD_SBF*N_SBF+iwrd];

// 			prevwrd = chan->dwrd[iwrd];
// 		}
// 		/*
// 		// Sanity check
// 		if (((chan->dwrd[1])&(0x1FFFFUL<<13)) != ((tow&0x1FFFFUL)<<13))
// 		{
// 			printf("\nWARNING: Invalid TOW in subframe 5.\n");
// 			return(0);
// 		}
// 		*/
// 	}

// 	for (isbf=0; isbf<N_SBF; isbf++)
// 	{
// 		tow++;

// 		for (iwrd=0; iwrd<N_DWRD_SBF; iwrd++)
// 		{
// 			sbfwrd = chan->sbf[isbf][iwrd];

// 			// Add transmission week number to Subframe 1
// 			if ((isbf==0)&&(iwrd==2))
// 				sbfwrd |= (wn&0x3FFUL)<<20;

// 			// Add TOW-count message into HOW
// 			if (iwrd==1)
// 				sbfwrd |= ((tow&0x1FFFFUL)<<13);

// 			// Compute checksum
// 			sbfwrd |= (prevwrd<<30) & 0xC0000000UL; // 2 LSBs of the previous transmitted word
// 			nib = ((iwrd==1)||(iwrd==9))?1:0; // Non-information bearing bits for word 2 and 10
// 			chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd] = computeChecksum(sbfwrd, nib);

// 			prevwrd = chan->dwrd[(isbf+1)*N_DWRD_SBF+iwrd];
// 		}
// 	}

// 	return(1);
// }

// unsigned long gps_con::computeChecksum(unsigned long source, int nib)
// {
// 	/*
// 	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
// 	Bits 29 to  6 = Source data bits, d1, d2, ..., d24
// 	Bits  5 to  0 = Empty parity bits
// 	*/ 

// 	/*
// 	Bits 31 to 30 = 2 LSBs of the previous transmitted word, D29* and D30*
// 	Bits 29 to  6 = Data bits transmitted by the SV, D1, D2, ..., D24
// 	Bits  5 to  0 = Computed parity bits, D25, D26, ..., D30
// 	*/ 

// 	/*
// 	                  1            2           3
// 	bit    12 3456 7890 1234 5678 9012 3456 7890
// 	---    -------------------------------------
// 	D25    11 1011 0001 1111 0011 0100 1000 0000
// 	D26    01 1101 1000 1111 1001 1010 0100 0000
// 	D27    10 1110 1100 0111 1100 1101 0000 0000
// 	D28    01 0111 0110 0011 1110 0110 1000 0000
// 	D29    10 1011 1011 0001 1111 0011 0100 0000
// 	D30    00 1011 0111 1010 1000 1001 1100 0000
// 	*/

// 	unsigned long bmask[6] = { 
// 		0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
// 		0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL };

// 	unsigned long D;
// 	unsigned long d = source & 0x3FFFFFC0UL;
// 	unsigned long D29 = (source>>31)&0x1UL;
// 	unsigned long D30 = (source>>30)&0x1UL;

// 	if (nib) // Non-information bearing bits for word 2 and 10
// 	{
// 		/*
// 		Solve bits 23 and 24 to presearve parity check
// 		with zeros in bits 29 and 30.
// 		*/

// 		if ((D30 + countBits(bmask[4] & d)) % 2)
// 			d ^= (0x1UL<<6);
// 		if ((D29 + countBits(bmask[5] & d)) % 2)
// 			d ^= (0x1UL<<7);
// 	}

// 	D = d;
// 	if (D30)
// 		D ^= 0x3FFFFFC0UL;

// 	D |= ((D29 + countBits(bmask[0] & d)) % 2) << 5;
// 	D |= ((D30 + countBits(bmask[1] & d)) % 2) << 4;
// 	D |= ((D29 + countBits(bmask[2] & d)) % 2) << 3;
// 	D |= ((D30 + countBits(bmask[3] & d)) % 2) << 2;
// 	D |= ((D30 + countBits(bmask[4] & d)) % 2) << 1;
// 	D |= ((D29 + countBits(bmask[5] & d)) % 2);
	
// 	D &= 0x3FFFFFFFUL;
// 	//D |= (source & 0xC0000000UL); // Add D29* and D30* from source data bits

// 	return(D);
// }

// unsigned long gps_con::countBits(unsigned long v)
// {
// 	unsigned long c;
// 	const int S[] = {1, 2, 4, 8, 16};
// 	const unsigned long B[] = {
// 		0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF};

// 	c = v;
// 	c = ((c >> S[0]) & B[0]) + (c & B[0]);
// 	c = ((c >> S[1]) & B[1]) + (c & B[1]);
// 	c = ((c >> S[2]) & B[2]) + (c & B[2]);
// 	c = ((c >> S[3]) & B[3]) + (c & B[3]);
// 	c = ((c >> S[4]) & B[4]) + (c & B[4]);

// 	return(c);
// }
