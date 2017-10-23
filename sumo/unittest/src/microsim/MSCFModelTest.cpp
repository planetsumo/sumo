/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSCFModelTest.cpp
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    2013-06-05
/// @version $Id$
///
// Tests the cfmodel functions 
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <gtest/gtest.h>
#include <utils/vehicle/SUMOVTypeParameter.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSVehicleType.h>
#include <microsim/cfmodels/MSCFModel.h>
#include <microsim/cfmodels/MSCFModel_Krauss.h>


class MSCFModelTest : public testing::Test {
    protected :
        MSVehicleType* type;
        MSCFModel* m;
        double accel;
        double decel;
        double dawdle;
        double tau; // headway time

        virtual void SetUp(){
            accel = 2;
            decel = 4;
            dawdle = 0;
            tau = 1;
            type = new MSVehicleType(SUMOVTypeParameter("0"));
            m = new MSCFModel_Krauss(type,
                    accel, decel, decel, decel, dawdle, tau);
            MSGlobals::gSemiImplicitEulerUpdate = true;
        }

        virtual void TearDown(){
            delete m;
            delete type;
        }
};

/* Test the method 'brakeGap'.*/

TEST_F(MSCFModelTest, test_method_brakeGap) {
    // discrete braking model. keep driving for 1 s
    const double v = 3;
    EXPECT_DOUBLE_EQ(tau * v, m->brakeGap(v));
}

TEST_F(MSCFModelTest, test_method_static_brakeGap) {
    const double v = 3;
    const double b = 4;
    const double tau = 1;
    EXPECT_DOUBLE_EQ(3, MSCFModel::brakeGap(v, b, tau));
    EXPECT_DOUBLE_EQ(0, MSCFModel::brakeGap(v, b, 0));
    EXPECT_DOUBLE_EQ(3, MSCFModel::brakeGap(7, b, 0));
    EXPECT_DOUBLE_EQ(28.6, MSCFModel::brakeGap(18.4, 4.5, 0));
    EXPECT_DOUBLE_EQ(47., MSCFModel::brakeGap(22.9, 4.5, 0));
    EXPECT_DOUBLE_EQ(44., MSCFModel::brakeGap(22.25, 4.5, 0));
}

TEST_F(MSCFModelTest, test_method_static_freeSpeed) {
    const double vCur = 10;
    const double b = 4;
    const double v = 0;
    const double g = 4;
    EXPECT_DOUBLE_EQ(4, MSCFModel::freeSpeed(vCur, b, g, v, false, 1.0));
    EXPECT_DOUBLE_EQ(8, MSCFModel::freeSpeed(vCur, b, g, v, true, 1.0));
    EXPECT_DOUBLE_EQ(17.5, MSCFModel::freeSpeed(vCur, 5, 30, 10, false, 1.0));
    EXPECT_DOUBLE_EQ(18.4, MSCFModel::freeSpeed(vCur, 4.5, 20, 13.9, false, 1.0));
    EXPECT_DOUBLE_EQ(18.4, MSCFModel::freeSpeed(vCur, 4.5, 30, 13.9, false, 1.0));
    EXPECT_DOUBLE_EQ(22.9, MSCFModel::freeSpeed(vCur, 4.5, 30, 13.9, true, 1.0));
    EXPECT_DOUBLE_EQ(22.25, MSCFModel::freeSpeed(vCur, 4.5, 40, 13.9, false, 1.0));
}

#ifdef HAVE_SUBSECOND_TIMESTEPS
TEST_F(MSCFModelTest, test_method_static_freeSpeed_half) {
    DELTA_T = 500;
    const double b = 4;
    const double v = 0;
    const double g = 4;
    EXPECT_DOUBLE_EQ(14./3., MSCFModel::freeSpeed(b, g, v, false, DELTA_T*0.001));
    EXPECT_DOUBLE_EQ(6., MSCFModel::freeSpeed(b, 6, v, false, DELTA_T*0.001));
    EXPECT_DOUBLE_EQ(18.75, MSCFModel::freeSpeed(5, 30, 10, false, DELTA_T*0.001));
    EXPECT_DOUBLE_EQ(18.4, MSCFModel::freeSpeed(4.5, 20, 13.9, false, DELTA_T*0.001));
    EXPECT_DOUBLE_EQ(20.65, MSCFModel::freeSpeed(4.5, 20, 13.9, true, DELTA_T*0.001));
    EXPECT_DOUBLE_EQ(20.65, MSCFModel::freeSpeed(4.5, 30, 13.9, false, DELTA_T*0.001));
    EXPECT_DOUBLE_EQ(22.9, MSCFModel::freeSpeed(4.5, 30, 13.9, true, DELTA_T*0.001));
    EXPECT_DOUBLE_EQ(22.9, MSCFModel::freeSpeed(4.5, 40, 13.9, false, DELTA_T*0.001));
}
#endif
