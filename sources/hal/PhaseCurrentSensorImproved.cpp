/* Copyright (C) 2015  Nils Weiss
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include "PhaseCurrentSensorImproved.h"
#include "trace.h"
#include <algorithm>

static const int __attribute__((unused)) g_DebugZones = ZONE_ERROR | ZONE_WARNING | ZONE_VERBOSE | ZONE_INFO;

using hal::Adc;
using hal::Factory;
using hal::HalfBridge;
using hal::PhaseCurrentSensorImproved;
using hal::Tim;

void PhaseCurrentSensorImproved::setPulsWidthForTriggerPerMill(uint32_t value) const
{
    if (value > HalfBridge::MAXIMAL_PWM_IN_MILL) {
        value = HalfBridge::MAXIMAL_PWM_IN_MILL;
    }

    if (value < HalfBridge::MINIMAL_PWM_IN_MILL) {
        value = HalfBridge::MINIMAL_PWM_IN_MILL;
    }

    float scale = static_cast<float>(mHBridge.mTim.getPeriode()) /
                  static_cast<float>(HalfBridge::MAXIMAL_PWM_IN_MILL);

    //static const uint32_t sampleTime = 2 ;//<< mAdc1.getAdcSampleTime();
    //half high dutty
    //value = static_cast<uint32_t>(static_cast<float>(value) * scale) * (0.45); // + value / 5000.0);
    //value += sampleTime;

    //    TIM_SetCompare4(mHBridge.mTim.getBasePointer(),
    //                    static_cast<uint32_t>(std::max(
    //                                                   static_cast<int32_t>(value + HalfBridge::DEFAULT_DEADTIME),
    //                                                   static_cast<int32_t>(1))));

    //Modify variable value for off part of duty cycle -> half low dutty
    value = ((HalfBridge::MAXIMAL_PWM_IN_MILL - (value * scale)) * 0.5) + (value * scale);
    TIM_SetCompare4(mHBridge.mTim.getBasePointer(),
                    static_cast<uint32_t>(std::max(
                                                   static_cast<int32_t>(value + HalfBridge::DEFAULT_DEADTIME),
                                                   static_cast<int32_t>(1))));

    // not working
    //    TIM_SetCompare5(mHBridge.mTim.getBasePointer(),
    //                        static_cast<uint32_t>(std::max(
    //                                                        static_cast<int32_t>(value + HalfBridge::DEFAULT_DEADTIME),
    //                                                        static_cast<int32_t>(1))));
}

void PhaseCurrentSensorImproved::setNumberOfMeasurementsForPhaseCurrentValue(uint32_t value) const
{
    if (value > MAX_NUMBER_OF_MEASUREMENTS) {
        value = MAX_NUMBER_OF_MEASUREMENTS;
    }
    if (value == 0) {
        value = 1;
    }
    mNumberOfMeasurementsForPhaseCurrentValue = value;
    //mAdc1.stopConversion();
    //mAdc1.startConversion(
    //                                MeasurementValueBuffer[mDescription].data(), mNumberOfMeasurementsForPhaseCurrentValue,
    //                                [&] {this->updateCurrentValue();
    //                                });
}

size_t PhaseCurrentSensorImproved::getNumberOfMeasurementsForPhaseCurrentValue(void) const
{
    return mNumberOfMeasurementsForPhaseCurrentValue;
}

void PhaseCurrentSensorImproved::updateCurrentValue(void) const
{
    auto& array = MeasurementValueBuffer[mDescription];

    for (size_t i = 0; i < mNumberOfMeasurementsForPhaseCurrentValue; i++) {
        mPhaseCurrentValue -= mPhaseCurrentValue / FILTERWIDTH;
        //use mNumberOfMeasurementsForPhaseCurrentValue many values back from mMeasureCounter, remember its a ringbuffer
        mPhaseCurrentValue +=
            static_cast<float>(array[((mMeasureCounter - i) + MAX_NUMBER_OF_MEASUREMENTS) %
                                     MAX_NUMBER_OF_MEASUREMENTS]) /
            FILTERWIDTH;
    }

    //only inform about new value after 10 measurements
    if (mValueAvailableSemaphore) {
        mValueAvailableSemaphore->giveFromISR();
        TraceLight("Current: %5.5f \n", mPhaseCurrentValue);
    }
}

void PhaseCurrentSensorImproved::registerValueAvailableSemaphore(os::Semaphore* valueAvailable) const
{
    mValueAvailableSemaphore = valueAvailable;
}

void PhaseCurrentSensorImproved::unregisterValueAvailableSemaphore(void) const
{
    mValueAvailableSemaphore = nullptr;
}

void PhaseCurrentSensorImproved::enable(void) const
{
    mAdc1.startConversion();
    mAdc2.startConversion();
    //mAdc1.startConversion(MeasurementValueBuffer[mDescription], [&] {this->updateCurrentValue();
    //                            });
}

void PhaseCurrentSensorImproved::disable(void) const
{
    mAdc1.stopConversion();
    mAdc2.stopConversion();
}

void PhaseCurrentSensorImproved::reset(void) const
{
    mPhaseCurrentValue = 2 * mOffsetValue - mPhaseCurrentValue;
}

void PhaseCurrentSensorImproved::calibrate(void) const
{
    os::ThisTask::sleep(std::chrono::milliseconds(250));
    mOffsetValue = mPhaseCurrentValue;
}

float PhaseCurrentSensorImproved::getPhaseCurrent(void) const
{
    static constexpr const float A_PER_DIGITS = 1.0 / 53.8;

    return static_cast<float>(mOffsetValue - mPhaseCurrentValue) *
           A_PER_DIGITS;
}

float PhaseCurrentSensorImproved::getCurrentVoltage(void) const
{
    return MeasurementValueBuffer[mDescription][mMeasureCounter];
    //mAdc1.getVoltage(mPhaseCurrentValue);
    //return mAdcWithDma.getVoltage(mPhaseCurrentValue);
}

void PhaseCurrentSensorImproved::interpretPhaseA(const uint16_t value) const
{
    //if transistor A is open, use his value, else use value from B
    if (mHBridge.getCurrentTransistorState((uint16_t)0) == false) { //if A is closed use value from B
        MeasurementValueBuffer[mDescription][(mMeasureCounter++) % MAX_NUMBER_OF_MEASUREMENTS] = value * (2);
    }

    //calc and notify about new values only everty 10th time
    if (mMeasureCounter%10 == 0){
        updateCurrentValue();
    }
    //TraceLight("A: %d \n",value);
}

void PhaseCurrentSensorImproved::interpretPhaseB(const uint16_t value) const
{
    //only use value from transistor B if A is closed
    if (mHBridge.getCurrentTransistorState((uint16_t)0) == true) { //if A is closed use value from B
        MeasurementValueBuffer[mDescription][(mMeasureCounter++) % MAX_NUMBER_OF_MEASUREMENTS] = value * (2);
    }

    /*if (mMeasureCounter%10 == 0){
        updateCurrentValue();
    }
    */
    //TraceLight("        B: %d \n",value);
}

void PhaseCurrentSensorImproved::initialize(void) const
{
    TIM_OC4Init(mHBridge.mTim.getBasePointer(), &mAdcTrgoConfiguration);
    TIM_OC4PreloadConfig(mHBridge.mTim.getBasePointer(), TIM_OCPreload_Enable);

    //TIM_OC5Init(mHBridge.mTim.getBasePointer(), &mAdcTrgoConfiguration);
    //TIM_OC5PreloadConfig(mHBridge.mTim.getBasePointer(), TIM_OCPreload_Enable);

    TIM_SelectMasterSlaveMode(mHBridge.mTim.getBasePointer(), TIM_MasterSlaveMode_Enable);

    /* Channel 4 output compare signal is connected to TRGO */
    TIM_SelectOutputTrigger(mHBridge.mTim.getBasePointer(), (uint16_t)TIM_TRGOSource_OC4Ref);

    //Channel 5 Output Compare signal is connected to TRG022 which is connected to ADC23
    //TIM_SelectOutputTrigger2(mHBridge.mTim.getBasePointer(), (uint16_t)TIM_TRGO2Source_OC5Ref);

    setPulsWidthForTriggerPerMill(1);
    mAdc1.registerInterruptCallback([&](uint16_t value){
        this->interpretPhaseA(value);
    });
    mAdc2.registerInterruptCallback([&](uint16_t value){
        this->interpretPhaseB(value);
    });
}

constexpr const std::array<const PhaseCurrentSensorImproved,
                           PhaseCurrentSensorImproved::Description::__ENUM__SIZE> Factory<PhaseCurrentSensorImproved>::
Container;
std::array<std::array<uint16_t,
                      PhaseCurrentSensorImproved::MAX_NUMBER_OF_MEASUREMENTS>,
           PhaseCurrentSensorImproved::Description::__ENUM__SIZE> PhaseCurrentSensorImproved::MeasurementValueBuffer;
