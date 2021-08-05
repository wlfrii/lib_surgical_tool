#ifndef TEST_KINEMATICS_H
#define TEST_KINEMATICS_H
#include <gtest/gtest.h>
#include <lib_instrument/instrument_handle.h>

TEST(kinematics, single_continuum_RT)
{
    InstrumentHandle ins;

    ins.libtest();
}

#endif // TEST_KINEMATICS_H