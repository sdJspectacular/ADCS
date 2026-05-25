#include <iostream>
#include <random>
#include "types.h"
#include "DiscreteStateSpace.h"
#include "LQRController.h"
#include "Log.h"

enum Scenario
{
    DEFAULT,
    NOISE
};

int main()
{
    Scenario scenario = DEFAULT;

    // Open loop discrete-time dynamic system
    Matrix A = {{1.0, 0.000999500166625, 0.0},
                {0.0, 0.999000499833375, 0.0},
                {0.0, 0.0, 1.0}};

    Matrix B = {{4.99833374991668e-07},
                {0.000999500166625008},
                {0.0}};

    Matrix Bd = {{4.99833374991668e-07},
                 {0.000999500166625008},
                 {0.001}};

    Matrix C = {{1.0, 0.732050807568877, 0.0}};
    Matrix D = {{0.0}};
    Matrix K = {{1.0, 0.732050807568877, 0.0}};

    // File to save to
    const std::string filename = "./data/simout.csv";

    // Setup C++ Pseudo-Random Number Generator (PRNG)
    std::random_device rd;
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    // Define a Normal (Gaussian) distribution: Mean = 0.0, Standard Deviation = 2.0
    double noise_mean = 0.0;
    double noise_stddev = 2.0;
    std::normal_distribution<double> disturbance_dist(noise_mean, noise_stddev);

    try
    {
        DiscreteStateSpace sys(A, B, C, D, Bd); // default x0 = [0; ...; 0]
        LQRController lqr(K);

        // Important: initial conditions
        switch (scenario)
        {
        case NOISE:
            sys.setInitialState({0.0, 0.0, 0.0});
            break;
        default:
            sys.setInitialState({0.0, 1.0, 1.0});
            break;
        }

        // Total simulation time
        double Tend = 10.0;
        // Sample time (must match sys(A,B,C,D))
        double Ts = 0.001; // 1 kHz
        size_t steps = static_cast<size_t>(Tend / Ts);

        // Initialize CSV file stream
        Log log(filename, sys);

        // Now run up to Tend
        std::cout << "Simulating and logging data to " << filename << "...\n";
        std::cout << "Time(s)\tOutput(y)\n";
        std::cout << "-----------------\n";

        // Simulation Loop
        for (size_t k = 0; k <= steps; ++k)
        {
            double tsim = k * Ts;

            Vector Qdist = {0.0};
            switch (scenario)
            {
            case NOISE:
                Qdist[0] = disturbance_dist(gen);
                if (k == 1000)
                    Qdist[0] += 500.0;
                else if (k == 3000)
                    Qdist[0] -= 500.0;
                break;
            default:
                Qdist[0] = 0.0;
                break;
            }
            //  Capture states x[k] *before* the update modifies them to x[k+1]
            const Vector &x = sys.getState();
            Vector u = lqr.computeControl(x);

            // Step the simulation forward
            Vector y = sys.lsim(u, Qdist);

            // Print results to stdout
            std::cout << tsim << "\t" << y[0] << "\n";

            // Write to log
            log.writeLog(tsim, u, Qdist, x, y);
        }

        log.close();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}
