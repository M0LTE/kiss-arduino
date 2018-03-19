// ctest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <string.h>
//#include <stdio.h>
#include <conio.h>
#include <stdlib.h>

//225446.12
int timeOfFixHours;
int timeOfFixMins;
int timeOfFixSecs;
int timeOfFixSecsFrac;

//A
char navRcvWarn;

//4916.45,N
int latDeg, latMin, latMinFrac;
char ns;

//12311.12,W
int lonDeg, lonMin, lonMinFrac;
char ew;

//000.5
int sogKnots, sogKnotsFrac;

//054.7
int courseTrueDeg, courseTrueDegFrac;

//191194
int utcDateOfFixD, utcDateOfFixM, utcDateOfFixY;

//020.3,E
int magVarDeg, magVarDegFrac;
char magVarEW;

//*68
char checksum[3];

char gpsBuf[255];

char tofBuf[7];
int interpret_timeOfFix(int startOfField) {

	// warning, all untested
	tofBuf[0] = gpsBuf[startOfField];
	tofBuf[1] = gpsBuf[startOfField + 1];
	tofBuf[2] = 0;
	timeOfFixHours = atoi(tofBuf);

	tofBuf[0] = gpsBuf[startOfField + 2];
	tofBuf[1] = gpsBuf[startOfField + 3];
	tofBuf[2] = 0;
	timeOfFixMins = atoi(tofBuf);

	tofBuf[0] = gpsBuf[startOfField + 4];
	tofBuf[1] = gpsBuf[startOfField + 5];
	tofBuf[2] = 0;
	timeOfFixSecs = atoi(tofBuf);

	for (int i = 0; i<sizeof(tofBuf); i++) {
		tofBuf[i] = 0;
	}
	// $GPRMC,225446.1234,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
	//           1111111111222222222233333333334444444444555555555577777777778
	// 01234567890123456789012345678901234567890123456789012345678901234567890

	char sep = gpsBuf[startOfField + 6];
	if (sep == '.') {
		int cur = 0;
		
		char c = 0;
		while(true) {
			c = gpsBuf[startOfField + 7 + cur];
			if (c == ',') {
				break;
			}
			tofBuf[cur] = c;
			cur++;
		}
		timeOfFixSecsFrac = atoi(tofBuf);
		return startOfField + 8 + cur;
	}
	else {
		timeOfFixSecsFrac = 0;
		return startOfField + 7; // hhmmss
	}
}

int interpret_sog(int startOfField) {

	int sogWholeDigits = 0;
	char buf[3] = "";
	while (true) {
		char c = gpsBuf[startOfField + sogWholeDigits];
		if (c == '.' || c==',')
			break;
		buf[sogWholeDigits] = c;
		sogWholeDigits++;
	}
	sogKnots = atoi(buf);
	
	for (int i = 0; i < sizeof(buf); i++) {
		buf[i] = 0;
	}

	if (gpsBuf[startOfField + sogWholeDigits] == '.') {
		
		int sogStartOfFrac = startOfField + sogWholeDigits + 1;

		int cnt = 0;
		while (true) {
			char c = gpsBuf[sogStartOfFrac + cnt];
			if (c == ',')
				break;
			buf[cnt] = c;
			cnt++;
		}

		sogKnotsFrac = atoi(buf);
	}
	else {
		sogKnotsFrac = 0;
	}

}

int interpret_latlon(int startOfField, bool lat) {
	//5916.45,N
	//916.456,N

	int degDigits = 0;

	while (true) {
		char c = gpsBuf[startOfField + degDigits];
		if (c == '.')
			break;

		degDigits++;
	}

	char buf[3] = "";
	for (int i = 0; i < degDigits - 2; i++) {
		buf[i] = gpsBuf[startOfField + i];
	}
	int val = atoi(buf);
	if (lat) {
		latDeg = val;
	}
	else {
		lonDeg = val;
	}

	buf[0] = buf[1] = buf[2] = 0;

	int wholeDegDigits = degDigits - 2;

	// skip that many positions

	for (int i = wholeDegDigits; i < degDigits; i++) {
		buf[i - wholeDegDigits] = gpsBuf[startOfField + i];
	}
	val = atoi(buf);
	if (lat) {
		latMin = val;
	}
	else {
		lonMin = val;
	}

	// find the fraction
	int startOfFrac = startOfField + degDigits + 1;

	char c;
	char fracArr[6] = "";
	int cnt = 0;
	while (cnt < 6) {
		c = gpsBuf[startOfFrac + cnt];
		if (c == ',') {
			break;
		}
		fracArr[cnt] = c;
		cnt++;
	}
	if (lat) {
		latMinFrac = atoi(fracArr);
	}
	else {
		lonMinFrac = atoi(fracArr);
	}

	// find the N/S / E/W
	int nsFieldLoc = 0;
	while (true) {
		char c = gpsBuf[startOfField + nsFieldLoc];
		if (c == ',')
			break;

		nsFieldLoc++;
	}

	c = gpsBuf[startOfField + nsFieldLoc + 1];
	if (lat) {
		ns = c;
	}
	else {
		ew = c;
	}

	return startOfField + nsFieldLoc + 3;
}

int interpret_navRcvWarn(int startOfField) {
	navRcvWarn = gpsBuf[startOfField];
	return startOfField + 2;
}

int startOfNextField;
void interpretGps() {
	if (strncmp(gpsBuf, "$GPRMC", 6) != 0) {
		// if the sentence is not Recommended minimum specific GPS/Transit data, ignore.
		return;
	}

	startOfNextField = 7;

	startOfNextField = interpret_timeOfFix(startOfNextField);

	startOfNextField = interpret_navRcvWarn(startOfNextField);

	startOfNextField = interpret_latlon(startOfNextField, true);

	startOfNextField = interpret_latlon(startOfNextField, false);

	startOfNextField = interpret_sog(startOfNextField);

	char nextChar = gpsBuf[startOfNextField];
}

int main()
{
	strcpy_s(gpsBuf, "$GPRMC,225446,A,5916.45,N,12311.12,W,23,054.7,191194,020.3,E*68");
	interpretGps();
	if (timeOfFixHours != 22) {
		printf("timeOfFixHours is %d not 22\n", timeOfFixHours);
	}
	if (timeOfFixMins != 54) {
		printf("timeOfFixMins is %d not 54\n", timeOfFixMins);
	}
	if (timeOfFixSecs != 46) {
		printf("timeOfFixSecs is %d not 46\n", timeOfFixSecs);
	}
	if (timeOfFixSecsFrac != 0) {
		printf("timeOfFixSecsFrac is %d not 0\n", timeOfFixSecsFrac);
	}
	if (navRcvWarn != 'A') {
		printf("navRcvWarn is %c not A\n", navRcvWarn);
	}
	if (latDeg != 59) {
		printf("latDeg is %d not 59\n", latDeg);
	}
	if (latMin != 16) {
		printf("latMin is %d not 16\n", latMin);
	}
	if (latMinFrac != 45) {
		printf("latMinFrac is %d not 45\n", latMinFrac);
	}
	if (ns != 'N') {
		printf("ns is %c not N\n", ns);
	}
	// 12311.12,W
	if (lonDeg != 123) {
		printf("lonDeg is %d not 123\n", lonDeg);
	}
	if (lonMin != 11) {
		printf("lonMin is %d not 11\n", lonMin);
	}
	if (lonMinFrac != 12) {
		printf("lonMinFrac is %d not 12\n", lonMinFrac);
	}
	if (ew != 'W') {
		printf("ew is %c not W\n", ew);
	}
	if (sogKnots != 23) {
		printf("sogKnots is %d not 23\n", sogKnots);
	}
	if (sogKnotsFrac != 0) {
		printf("sogKnotsFrac is %d not 0\n", sogKnotsFrac);
	}

	strcpy_s(gpsBuf, "$GPRMC,225446.1234,A,916.456,N,11.12,E,020.5,054.7,191194,020.3,E*68");
	interpretGps();
	if (timeOfFixHours != 22) {
		printf("timeOfFixHours is %d not 22\n", timeOfFixHours);
	}
	if (timeOfFixMins != 54) {
		printf("timeOfFixMins is %d not 54\n", timeOfFixMins);
	}
	if (timeOfFixSecs != 46) {
		printf("timeOfFixSecs is %d not 46\n", timeOfFixSecs);
	}
	if (timeOfFixSecsFrac != 1234) {
		printf("timeOfFixSecsFrac is %d not 1234\n", timeOfFixSecsFrac);
	}
	if (navRcvWarn != 'A') {
		printf("navRcvWarn is %c not A\n", navRcvWarn);
	}
	if (latDeg != 9) {
		printf("latDeg is %d not 9\n", latDeg);
	}
	if (latMin != 16) {
		printf("latMin is %d not 16\n", latMin);
	}
	if (latMinFrac != 456) {
		printf("latMinFrac is %d not 456\n", latMinFrac);
	}
	if (ns != 'N') {
		printf("ns is %c not N\n", ns);
	}
	// 11.12,E
	if (lonDeg != 0) {
		printf("lonDeg is %d not 0\n", lonDeg);
	}
	if (lonMin != 11) {
		printf("lonMin is %d not 11\n", lonMin);
	}
	if (lonMinFrac != 12) {
		printf("lonMinFrac is %d not 12\n", lonMinFrac);
	}
	if (ew != 'E') {
		printf("ew is %c not E\n", ew);
	}
	if (sogKnots != 20) {
		printf("sogKnots is %d not 20\n", sogKnots);
	}
	if (sogKnotsFrac != 5) {
		printf("sogKnotsFrac is %d not 5\n", sogKnotsFrac);
	}

	printf("Done, press any key to continue\n");
	_getch();
	return 0;
}