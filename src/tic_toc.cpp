#include "tic_toc.h"
#include <stack>
#include <chrono>

using std::stack;
using namespace std::chrono;

std::stack<high_resolution_clock::time_point> tictoc_stack;

void tic() {
    tictoc_stack.push(high_resolution_clock::now());
}

double toc() {
    double dt = duration_cast<microseconds>(high_resolution_clock::now() - tictoc_stack.top()).count() / 1.0e3;
    tictoc_stack.pop();
    return dt;
}
