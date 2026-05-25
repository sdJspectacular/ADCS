#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <stdexcept>

// Aliases for readability
using Vector = std::vector<double>;
using Matrix = std::vector<std::vector<double>>;

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
    // Matrices computed using ss(c2d(ss(A, B, C, D), Ts))
    Matrix A, B, C, D, Bd;

    // State vector
    Vector x;

    // length(x)
    size_t num_states;

    // length(u)
    size_t num_inputs;

    // length(d)
    size_t num_disturbances;

    // length(y)
    size_t num_outputs;

public:
    DiscreteStateSpace(Matrix a, Matrix b, Matrix c, Matrix d, Matrix bd)
        : A(std::move(a)), B(std::move(b)), C(std::move(c)), D(std::move(d)),
          Bd(std::move(bd))
    {

        num_states = A.size();
        num_inputs = B.empty() ? 0 : B[0].size();
        num_outputs = C.empty() ? 0 : C.size();
        num_disturbances = Bd.empty() ? 0 : Bd[0].size();

        // If Bd or Dd are not provided, default them to dimensions matching zero matrices
        num_disturbances = Bd.empty() ? 0 : Bd[0].size();

        // Basic validation
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

        // Initialize state vector to zeros
        x.assign(num_states, 0.0);
    }

    // Set or reset the initial state conditions
    void setInitialState(const Vector &x0)
    {
        if (x0.size() != num_states)
        {
            throw std::invalid_argument("Initial state size mismatch.");
        }
        x = x0;
    }

    // Updates the state and returns the current output: y[k] = C*x[k] + D*u[k]
    // Note: no time scaling, it's already implicit in the coefficients
    Vector lsim(const Vector &u, const Vector &d)
    {

        // Check for a bad number of inputs
        if (u.size() != num_inputs)
        {
            throw std::invalid_argument("Input vector size mismatch.");
        }
        if (d.size() != num_disturbances)
        {
            throw std::invalid_argument("Disturbance size mismatch.");
        }

        // 1. Update output vector: y = C*x + D*u
        Vector y(num_outputs, 0.0);
        for (size_t i = 0; i < num_outputs; ++i)
        {
            for (size_t j = 0; j < num_states; ++j)
            {
                y[i] += C[i][j] * x[j];
            }
            for (size_t j = 0; j < num_inputs; ++j)
            {
                y[i] += D[i][j] * u[j];
            }
        }

        // 2. Propagate state for next step: x[k+1] = A*x[k] + B*u[k]
        Vector x_next(num_states, 0.0);
        for (size_t i = 0; i < num_states; ++i)
        {
            for (size_t j = 0; j < num_states; ++j)
            {
                x_next[i] += A[i][j] * x[j];
            }
            for (size_t j = 0; j < num_inputs; ++j)
            {
                x_next[i] += B[i][j] * u[j];
            }
            for (size_t j = 0; j < num_disturbances; ++j)
            {
                x_next[i] += Bd[i][j] * d[j];
            }
        }

        x = std::move(x_next);
        return y;
    }

    const Vector &getState() const { return x; }
    size_t getNumStates() const { return num_states; }
    size_t getNumInputs() const { return num_inputs; }
    size_t getNumDisturbances() const { return num_disturbances; }
    size_t getNumOutputs() const { return num_outputs; }
};

// Apply optimal control, u = -k * x
// This is P-D.  To use integral control the state must be augmented
// The optimal gain matrix is computed off-line
// LQR is guaranteed to be stable, but can't directly dial GM/PM
class LQRController
{
private:
    Matrix K;
    size_t num_inputs;
    size_t num_states;

public:
    LQRController(Matrix k_matrix) : K(std::move(k_matrix))
    {
        num_inputs = K.size();
        num_states = K.empty() ? 0 : K[0].size();
    }

    // u = - k * x
    Vector computeControl(const Vector &x) const
    {
        Vector u(num_inputs, 0.0);
        for (size_t i = 0; i < num_inputs; ++i)
        {
            for (size_t j = 0; j < num_states; ++j)
            {
                u[i] -= K[i][j] * x[j];
            }
        }
        return u;
    }
};

int main()
{
    // Open loop discrete-time dynamic system
    Matrix A = {{0.9995, 0.0010},
                {-0.9995, 0.9985}};
    Matrix B = {{0.0000},
                {0.0010}};
    Matrix C = {{1.0, 0.0},
                {0.0, 1.0}};
    Matrix D = {{0.0},
                {0.0}};

    Matrix Bd = {{0.0},  
                 {0.0010}};

    // Example LQR Optimal Gain Matrix K (1 row x 2 columns for 1 input, 2 states)
    // Derived offline assuming Q = diag(1000, 1), R = 0.1
    Matrix K = {{12.34, 1.56}};

    // File to save to
    const std::string filename = "simout.csv";

    try
    {
        DiscreteStateSpace sys(A, B, C, D, Bd); // default x0 = [0; ...; 0]
        sys.setInitialState({1.0, -0.5});       // Important: initial conditions
        LQRController controller(K);

        double Tend = 0.05; // Total simulation time
        double Ts = 0.001;  // 1 kHz
        size_t steps = static_cast<size_t>(Tend / Ts);

        // Initialize CSV file stream
        std::ofstream csv_file(filename);
        if (!csv_file.is_open())
        {
            throw std::runtime_error("Failed to open file for writing: " + filename);
        }

        // Write CSV log in format:
        // time[sec], u[0], ..., [num_inputs-1], x[0], ..., x[num_states-1], y[0], ..., y[num_outputs-1]

        csv_file << "Time";
        for (size_t i = 0; i < sys.getNumInputs(); ++i)
        {
            csv_file << ",u[" << i << "]";
        }
        for (size_t i = 0; i < sys.getNumStates(); ++i)
        {
            csv_file << ",x[" << i << "]";
        }
        for (size_t i = 0; i < sys.getNumOutputs(); ++i)
        {
            csv_file << ",y[" << i << "]";
        }
        csv_file << "\n";

        // Now run up to Tend
        std::cout << "Simulating and logging data to " << filename << "...\n";
        std::cout << "Time(s)\tOutput(y)\n";
        std::cout << "-----------------\n";

        // Simulation Loop
        for (size_t k = 0; k <= steps; ++k)
        {
            double tsim = k * Ts;
            Vector u;

            // Define control input u[k] (Step input of 1.0)
            if (tsim < 0.03)
                u = {0.0};
            else
                u = {1.0};

            // Capture states x[k] *before* the update modifies them to x[k+1]
            const Vector &x = sys.getState();

            // Step the simulation forward
            Vector y = sys.lsim(u);

            // Print results to stdout
            std::cout << tsim << "\t" << y[0] << "\n";

            // Write to log
            csv_file << tsim;

            // Log all inputs
            for (double input_val : u)
            {
                csv_file << "," << input_val;
            }

            // Log all states matching the current time step
            for (double state_val : x)
            {
                csv_file << "," << state_val;
            }

            for (double output_val : y)
            {
                csv_file << "," << output_val;
            }
            csv_file << "\n";
        }

        csv_file.close();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}
