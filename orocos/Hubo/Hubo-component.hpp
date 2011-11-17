#ifndef OROCOS_HUBO_COMPONENT_HPP
#define OROCOS_HUBO_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>

class Hubo
    : public RTT::TaskContext
{
 public:
    Hubo(string const& name)
        : TaskContext(name)
    {
        std::cout << "Hubo constructed !" <<std::endl;
    }

    bool configureHook() {
        std::cout << "Hubo configured !" <<std::endl;
        return true;
    }

    bool startHook() {
        std::cout << "Hubo started !" <<std::endl;
        return true;
    }

    void updateHook() {
        std::cout << "Hubo executes updateHook !" <<std::endl;
    }

    void stopHook() {
        std::cout << "Hubo executes stopping !" <<std::endl;
    }

    void cleanupHook() {
        std::cout << "Hubo cleaning up !" <<std::endl;
    }
};

#endif
