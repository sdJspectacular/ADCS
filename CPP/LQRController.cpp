#include "LQRController.h"

LQRController::LQRController(Matrix k_matrix) : K(std::move(k_matrix))
{
    num_inputs = K.size();
    num_states = K.empty() ? 0 : K[0].size();
}

Vector LQRController::computeControl(const Vector &x) const
{
    Vector u(num_inputs, 0.0);
    // u = -K * x
    for (size_t i = 0; i < num_inputs; ++i)
    {
        for (size_t j = 0; j < num_states; ++j)
        {
            u[i] -= K[i][j] * x[j];
        }
    }
    return u;
}
