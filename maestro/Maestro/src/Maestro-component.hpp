#ifndef OROCOS_MAESTRO_COMPONENT_HPP
#define OROCOS_MAESTRO_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>

class Maestro
    : public RTT::TaskContext
{
 public:
    Maestro(string const& name)
        : TaskContext(name)
    {
        std::cout << "Maestro constructed !" <<std::endl;
    }

    bool configureHook() {
        std::cout << "Maestro configured !" <<std::endl;
        return true;
    }

    bool startHook() {
        std::cout << "Maestro started !" <<std::endl;
        return true;
    }

    void updateHook() {
        std::cout << "Maestro executes updateHook !" <<std::endl;
    }

    void stopHook() {
        std::cout << "Maestro executes stopping !" <<std::endl;
    }

    void cleanupHook() {
        std::cout << "Maestro cleaning up !" <<std::endl;
    }
};

#endif
