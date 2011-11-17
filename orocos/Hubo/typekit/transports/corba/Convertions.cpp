/* Generated from orogen/lib/orogen/templates/typekit/corba/Convertions.cpp */

#include "Convertions.hpp"
#include <memory>







bool orogen_typekits::toCORBA( orogen::Corba::HuboData& corba, HuboData const& value )
{
    if (!toCORBA(corba.samples, value.samples)) return false;

    return true;
}
bool orogen_typekits::fromCORBA( HuboData& value, orogen::Corba::HuboData const& corba )
{
    if (!fromCORBA(value.samples, corba.samples)) return false;

    return true;
}

bool orogen_typekits::toCORBA( char const*& corba, ::std::string const& value )
{
    corba = value.c_str();

    return true;
}
bool orogen_typekits::fromCORBA( ::std::string& value, char const* corba )
{
    value = corba;

    return true;
}

bool orogen_typekits::toCORBA( orogen::Corba::vector__double_& corba, ::std::vector< double > const& value )
{
    corba.length(value.size());
        size_t i0 = 0;
        for(::std::vector< double >::const_iterator it = value.begin(); it != value.end(); ++it, ++i0)
        {
    corba[i0] = (*it);
    }

    return true;
}
bool orogen_typekits::fromCORBA( ::std::vector< double >& value, orogen::Corba::vector__double_ const& corba )
{
    size_t const size_i0 = corba.length();
    value.resize(size_i0);
        for(size_t i0 = 0; i0 < size_i0; ++i0)
        {
    value[i0] = corba[i0];
    }

    return true;
}



