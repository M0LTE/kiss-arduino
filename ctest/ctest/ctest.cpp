// ctest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <string.h>
//#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include <math.h>

#define INFO_FIELD_LEN 255
char infoField[INFO_FIELD_LEN];
// 51.447326, -1.027283
long latitude_uDeg = 51447326;
long longitude_uDeg = -1027283;

void build_info_field(char msg[], int msgLen) {
	// e.g. !5126.82N/00101.68W>Arduino testing

	for (int i = 0; i<INFO_FIELD_LEN; i++) {
		infoField[i] = 0;
	}

	infoField[0] = '!';

	//Serial.println(latitude_uDeg); // 51447326 -> 51d 26.84m

	long abslatitude_uDeg;
	if (latitude_uDeg < 0) {
		abslatitude_uDeg = latitude_uDeg * -1;
	}
	else {
		abslatitude_uDeg = latitude_uDeg;
	}

	int latWholeDeg = abslatitude_uDeg / 1000000; // 51
	//Serial.println(latWholeDeg);

	long lat_whole = latWholeDeg * 1000000;
	long lat_frac = abslatitude_uDeg - lat_whole;
	//Serial.println(lat_frac); // 447326

	long latMegaMins = lat_frac * 60;
	//Serial.println(megaMins); // 26838480

	long minsPart = latMegaMins / 10000;
	//Serial.println(minsPart); // 2683

	long wholeMins = minsPart / 100;

	long decMinsPart = minsPart - wholeMins * 100;
	//Serial.println(decMinsPart); // 83

	char buf[4];
	buf[2] = 0;
	buf[3] = 0;
	
	_itoa(latWholeDeg, buf, 10);
	if (latWholeDeg < 10) {
		infoField[1] = '0';
		infoField[2] = buf[0];
	}
	else {
		infoField[1] = buf[0];
		infoField[2] = buf[1];
	}

	_itoa(wholeMins, buf, 10);
	infoField[3] = buf[0];
	infoField[4] = buf[1];
	infoField[5] = '.';
	_itoa(decMinsPart, buf, 10);
	infoField[6] = buf[0];
	infoField[7] = buf[1];
	infoField[8] = latitude_uDeg > 0 ? 'N' : 'S';

	// bookmark.
	// Lat seems to work.Need to work on longitude encoding now.

	infoField[9] = '/';

	// clear buffer
	for (int i = 0; i < sizeof(buf); i++) {
		buf[i] = 0;
	}

	// 00101.68W
	// long longitude_uDeg = -1027283;
	long absLongUdeg;
	if (longitude_uDeg < 0) {
		absLongUdeg = longitude_uDeg * -1;
	}
	else {
		absLongUdeg = longitude_uDeg;
	}

	// 1027283
	int lonWholeuDeg = (absLongUdeg / 1000000) * 1000000; // 1000000
	printf("%d\n", lonWholeuDeg);

	int lonWholeDeg = lonWholeuDeg / 1000000;
	_itoa(lonWholeDeg, buf, 10);
	
	// pad
	if (lonWholeDeg < 10) {
		infoField[10] = '0';
		infoField[11] = '0';
		infoField[12] = buf[0];
	}
	else if (lonWholeDeg < 100) {
		infoField[10] = '0';
		infoField[11] = buf[0];
		infoField[12] = buf[1];
	}
	else {
		infoField[10] = buf[0];
		infoField[11] = buf[1];
		infoField[12] = buf[2];
	}

	int lonFracUDeg = absLongUdeg - lonWholeuDeg; // 27283
	printf("%d\n", lonFracUDeg);

	int lonuFrac = lonFracUDeg * 60; // = 1636980
	printf("%d\n", lonuFrac);

	int preDP = lonuFrac / 1000000; // 1
	int postDP = lonuFrac - preDP * 1000000;
	float postDPRounded_f = postDP / 10000.;
	int postDPRounded = lround(postDPRounded_f); // 64

	// 1.027283d
	// 1027283      <- absLongUdeg
	// 0.027283 deg = 1.63698 mins, get by multiplying by 60
	// 1d 1.63698m
	// 001 01.64

	// clear buffer
	for (int i = 0; i < sizeof(buf); i++) {
		buf[i] = 0;
	}

	// 001 01.68W
	_itoa(preDP, buf, 10);
	if (preDP < 10) {
		infoField[13] = '0';
		infoField[14] = buf[0];
	}
	else {
		infoField[13] = buf[0];
		infoField[14] = buf[1];
	}

	infoField[15] = '.';
	
	_itoa(postDPRounded, buf, 10);
	if (postDPRounded < 10) {
		infoField[16] = '0';
		infoField[17] = buf[0];
	}
	else {
		infoField[16] = buf[0];
		infoField[17] = buf[1];
	}

	if (longitude_uDeg < 0) {
		infoField[18] = 'W';
	}
	else {
		infoField[18] = 'E';
	}

	infoField[19] = '>';

	for (int i = 0; i < msgLen; i++) {
		infoField[20 + i] = msg[i];
	}

	for (int i = 0; i<INFO_FIELD_LEN; i++) {
		if (infoField[i] == 0) {
			break;
		}
		printf("%c", infoField[i]);
	}
	printf("\n");
}

int main()
{
	char msg[] = "Arduino testing";

	build_info_field(msg, sizeof(msg));
	printf("Done, press any key to continue\n");
	_getch();
	return 0;
}