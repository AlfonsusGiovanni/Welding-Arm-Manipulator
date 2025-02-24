/*
	MATHEMATICAL EQUATION FOR 6-DOF ARM ROBOT
	Author	: Alfonsus Giovanni Mahendra Putra
	Date		: Februari 2025
*/

#ifndef ARMROBOT_MATH_H
#define ARMROBOT_MATH_H

#include "math.h"

typedef struct{
	float
	rot_angle[6],			// Sudut rotasi Zi-1 dengan Z1 searah rotasi Zi
	distance[6],			// Jarak translasi sepanjang sumbu Z yang menhubungkan link satu dengan link yang lain
	link_length[6],		// Jarak translasi sepanjang sumbu X antara sumubu Zi+1 dengan Zi
	alpha[6];					// Sudut rotasi antara sumbu Zi-1 dengan Z1 searah rotasi Xi
}DH_Param_t;

#endif