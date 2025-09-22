//
// Created by Cam on 2024-09-21.
//

#include "Timer.h"

#include <utility>

Timer::Timer() : m_StartTime(std::chrono::high_resolution_clock::now()) {}

void Timer::reset(){
    m_StartTime = std::chrono::high_resolution_clock::now();
}

double Timer::getElapsedSeconds(){
    std::chrono::duration<double> delta = std::chrono::high_resolution_clock::now() - m_StartTime;
    return delta.count();
}


ScopedTimer::ScopedTimer(std::string message) : Timer(), m_message(std::move(message)) {}

void ScopedTimer::setMessage(const std::string& message)
{
    m_message = message;
}

ScopedTimer::~ScopedTimer(){
    std::cout << m_message << " took " << std::setprecision(5) << getElapsedSeconds() << std::setprecision(default_precision) << "s\n";
}