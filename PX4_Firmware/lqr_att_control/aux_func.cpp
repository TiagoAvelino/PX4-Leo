
#include <px4_defines.h>
#include <px4_posix.h>
#include <stdio.h>
#include <fstream>
#include <lib/mathlib/mathlib.h>
#include <sstream>
#include "aux_func.hpp"

void __EXPORT mount_g_n(math::Matrix<3, 6*N_PRED> &m)
{
    /* open file  */
    std::ifstream data_file;
    data_file.open ("dados_g_n.txt");

    std::string l;
    std::stringstream ss;
    float buf, c;
    for (int i = 0; i < 3; i++)
    {
        getline(data_file,l);
        ss.str(l);
        c = 0;
        while (ss >> buf) {
                m(i,c) = buf;
                c++;
        }
        ss.str(std::string());
        ss.clear();
        l.clear();
    }

    data_file.close();

}


void __EXPORT mount_k_n(math::Matrix<3, 6> &m)
{
    /* open file  */
    std::ifstream data_file;
    data_file.open ("dados_k_n.txt");

    std::string l;
    std::stringstream ss;
    float buf, c;
    for (int i = 0; i < 3; i++)
    {
        getline(data_file,l);
        ss.str(l);
        c = 0;
        while (ss >> buf) {
                m(i,c) = buf;
                c++;
        }
        ss.str(std::string());
        ss.clear();
        l.clear();
    }


    data_file.close();

}
