#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include "types.h"
#include <cstddef>

// Linear Quadratic Regulator (LQR) Controller
// Closes the loop:
//      x[k+1] = A * x[k] + B * u[k]
//        u[k] = -K * x[k]
class LQRController
{
private:
    Matrix K;
    size_t num_inputs;
    size_t num_states;

public:
    LQRController(Matrix k_matrix);

    Vector computeControl(const Vector &x) const;
};

#endif // LQR_CONTROLLER_H
