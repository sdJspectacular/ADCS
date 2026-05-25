#include "Log.h"

Log::Log(const std::string &fname, const DiscreteStateSpace &sys)
{
    csv_file.open(fname, std::ios::out);
    if (!csv_file.is_open())
    {
        throw std::runtime_error("Failed to open file for writing: " + fname);
    }

    // Write header row
    csv_file << "Time";
    for (size_t i = 0; i < sys.getNumInputs(); ++i)
    {
        csv_file << ",u[" << i << "]";
    }
    for (size_t i = 0; i < sys.getNumDisturbances(); ++i)
    {
        csv_file << ",d[" << i << "]";
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
}

void Log::close()
{
    if (csv_file.is_open())
    {
        csv_file.close();
    }
}

void Log::writeLog(const double tsim, const Vector &u, const Vector &d, const Vector &x, const Vector &y)
{
    // Write CSV log in format:
    // time[sec], u[0], ..., [num_inputs-1], d[0], ..., d[num_disturbances-1], x[0], ..., x[num_states-1], y[0], ..., y[num_outputs-1]
    csv_file << tsim;
    for (double input_val : u)
    {
        csv_file << "," << input_val;
    }
    for (double dist_val : d)
    {
        csv_file << "," << dist_val;
    }
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
