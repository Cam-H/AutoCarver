//
// Created by Cam on 2024-09-21.
//

#ifndef AUTOCARVER_TIMER_H
#define AUTOCARVER_TIMER_H

#include <chrono>
#include <string>

#include <iostream>
#include <iomanip>

class Timer {
public:

    Timer();

    void reset();

    double getElapsedSeconds();

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_StartTime;
};

class ScopedTimer : public Timer {
public:
    explicit ScopedTimer(std::string message);
    ~ScopedTimer();
private:
    std::string m_Message;

    const std::streamsize default_precision = std::cout.precision();
};


#endif //AUTOCARVER_TIMER_H
