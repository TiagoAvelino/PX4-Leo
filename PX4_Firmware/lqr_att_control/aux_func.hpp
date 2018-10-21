#include <platforms/px4_defines.h>

#ifndef AUX_FUNC_H
#define AUX_FUNC_H

#define N_PRED                  10000

void __EXPORT mount_g_n(math::Matrix<3, 6*N_PRED> &m);

void __EXPORT mount_k_n(math::Matrix<3, 6> &m);

#endif
