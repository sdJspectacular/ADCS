#ifndef DISCRETE_STATE_SPACE_H
#define DISCRETE_STATE_SPACE_H

#include "types.h"
#include <cstddef>
#include <stdexcept>

// State Space: x[k+1] = A * x[k] + B * u[k] + Bd * d[k]
//      Output:   y[k] = C * x[k] + D * u[k]
//
//   Bd is state disturbance matrix
//
// In well behaved systems:
//   (A, B) is controllable
//   (C, A) is observable
class DiscreteStateSpace
{
private:
    Matrix A, B, C, D, Bd;
    Vector x;
    size_t num_states;
    size_t num_inputs;
    size_t num_disturbances;
    size_t num_outputs;

public:
    // Constructor
    DiscreteStateSpace(Matrix a, Matrix b, Matrix c, Matrix d, Matrix bd);

    // Set initial state
    void setInitialState(const Vector &x0);

    // Simulate one time step
    Vector lsim(const Vector &u, const Vector &d);

    // Utility functions
    const Vector &getState() const;
    size_t getNumStates() const;
    size_t getNumInputs() const;
    size_t getNumDisturbances() const;
    size_t getNumOutputs() const;
};

#endif // DISCRETE_STATE_SPACE_H
