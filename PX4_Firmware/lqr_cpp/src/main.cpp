#include <iostream>
#include "definicoes.hpp"
#include "lqrdados.hpp"
#include "funcoes_auxiliares.hpp"
#include <Eigen/Dense>

int
main(int argc, char **argv)
{
// Construct LQR 
	LQRDados LQR;
	
	std::cout << ("Classe LQR contruida!") << std::endl;
	
	LQR.imprime_LQR();
	
	return 0;	
}
