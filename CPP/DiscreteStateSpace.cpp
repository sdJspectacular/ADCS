#include "DiscreteStateSpace.h"
#include <utility>

DiscreteStateSpace::DiscreteStateSpace(Matrix a, Matrix b, Matrix c, Matrix d, Matrix bd)
    : A(std::move(a)), B(std::move(b)), C(std::move(c)), D(std::move(d)), Bd(std::move(bd))
{
    num_states = A.size();
    num_inputs = B.empty() ? 0 : B[0].size();
    num_outputs = C.empty() ? 0 : C.size();
    num_disturbances = Bd.empty() ? 0 : Bd[0].size();

    // Validate dimensions
    if (A.empty() || A.size() != A[0].size())
    {
        throw std::invalid_argument("A matrix must be square.");
    }
    if (B.size() != num_states)
    {
        throw std::invalid_argument("B matrix row count must match A row count.");
    }
    if (Bd.size() != num_states)
    {
        throw std::invalid_argument("Bd row count mismatch.");
    }
    if (!C.empty() && C[0].size() != num_states)
    {
        throw std::invalid_argument("C matrix column count must match A row count.");
    }
    if (!D.empty() && (D.size() != num_outputs || D[0].size() != num_inputs))
    {
        throw std::invalid_argument("D matrix dimensions must match C rows and B columns.");
    }

    // Default state vector initialization
    x.assign(num_states, 0.0);
}

void DiscreteStateSpace::setInitialState(const Vector &x0)
{
    if (x0.size() != num_states)
    {
        throw std::invalid_argument("Initial state size mismatch.");
    }
    x = x0;
}

Vector DiscreteStateSpace::lsim(const Vector &u, const Vector &d)
{
    // Check input sizes (values can change at each update)
    if (u.size() != num_inputs)
    {
        throw std::invalid_argument("Input vector size mismatch.");
    }
    if (d.size() != num_disturbances)
    {
        throw std::invalid_argument("Disturbance size mismatch.");
    }

    Vector y(num_outputs, 0.0);
    for (size_t i = 0; i < num_outputs; ++i)
    {
        // y[k] = C * x[k]
        for (size_t j = 0; j < num_states; ++j)
        {
            y[i] += C[i][j] * x[j];
        }
        // y[k] += D * u[k]
        for (size_t j = 0; j < num_inputs; ++j)
        {
            y[i] += D[i][j] * u[j];
        }
    }

    Vector x_next(num_states, 0.0);
    for (size_t i = 0; i < num_states; ++i)
    {
        // x[k+1] = A * x[k]
        for (size_t j = 0; j < num_states; ++j)
        {
            x_next[i] += A[i][j] * x[j];
        }
        // x[k+1] += B * u[k]
        for (size_t j = 0; j < num_inputs; ++j)
        {
            x_next[i] += B[i][j] * u[j];
        }
        //x[k+1] += Bd * d[k]
        for (size_t j = 0; j < num_disturbances; ++j)
        {
            x_next[i] += Bd[i][j] * d[j];
        }
    }

    // x[k] <-- x[k+1]
    x = std::move(x_next);
    return y;
}

const Vector &DiscreteStateSpace::getState() const { return x; }
size_t DiscreteStateSpace::getNumStates() const { return num_states; }
size_t DiscreteStateSpace::getNumInputs() const { return num_inputs; }
size_t DiscreteStateSpace::getNumDisturbances() const { return num_disturbances; }
size_t DiscreteStateSpace::getNumOutputs() const { return num_outputs; }
