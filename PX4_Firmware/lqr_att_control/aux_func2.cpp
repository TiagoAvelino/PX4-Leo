#include <px4_defines.h>
#include <px4_posix.h>
#include <stdio.h>
#include <lib/mathlib/mathlib.h>
#include "aux_func2.hpp"


void __EXPORT mount_k_n(math::Matrix<4, 8> &m)
{
	FILE *f = fopen("dados_k_n.txt", "r");
	if (f == NULL)
	{
    		PX4_INFO("Error opening file!\n");
	}
	char line[100];
	int i;
	double trash[8];
	for (i=0; i<4; i++)
	{
		if (fgets(line, sizeof(line), f) == NULL)
		{
			PX4_INFO("Erro na leitura com fgets\n");
			break;
		}
		sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf\n",&trash[0],&trash[1],&trash[2],&trash[3],&trash[4],&trash[5],&trash[6],&trash[7]);
		m(i,0) = (float)trash[0];
		m(i,1) = (float)trash[1];
		m(i,2) = (float)trash[2];
		m(i,3) = (float)trash[3];
		m(i,4) = (float)trash[4];
		m(i,5) = (float)trash[5];
		m(i,6) = (float)trash[6];
		m(i,7) = (float)trash[7];
	}
	
}
