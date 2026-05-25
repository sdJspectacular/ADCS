#ifndef LOG_H
#define LOG_H

#include "types.h"
#include "DiscreteStateSpace.h"
#include <fstream>
#include <string>

// Write simulation data to a CSV file
class Log
{
private:
    std::ofstream csv_file;

public:
    Log(const std::string &fname, const DiscreteStateSpace &sys);
    void close();
    void writeLog(const double tsim, const Vector &u, const Vector &d, const Vector &x, const Vector &y);
};

#endif // LOG_H
