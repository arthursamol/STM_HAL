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

#ifndef SOURCES_PMD_PHASE_CURRENT_IMPROVED_SENSOR_H_
#define SOURCES_PMD_PHASE_CURRENT_IMPROVED_SENSOR_H_

#include <cstdint>
#include <array>
#include "hal_Factory.h"
#include "stm32f30x_syscfg.h"
#include "TimHalfBridge.h"
#include "AdcChannel.h"
#include "Adc.h"

namespace hal
{
struct PhaseCurrentSensorImproved {
#include "PhaseCurrentSensorImproved_config.h"

    PhaseCurrentSensorImproved() = delete;
    PhaseCurrentSensorImproved(const PhaseCurrentSensorImproved&) = delete;
    PhaseCurrentSensorImproved(PhaseCurrentSensorImproved&&) = default;
    PhaseCurrentSensorImproved& operator=(const PhaseCurrentSensorImproved&) = delete;
    PhaseCurrentSensorImproved& operator=(PhaseCurrentSensorImproved&&) = delete;

    float getPhaseCurrent(void) const;
    float getCurrentVoltage(void) const;
    void registerValueAvailableSemaphore(os::Semaphore* valueAvailable) const;
    void unregisterValueAvailableSemaphore(void) const;
    void calibrate(void) const;
    void reset(void) const;
    void setPulsWidthForTriggerPerMill(uint32_t) const;
    void setNumberOfMeasurementsForPhaseCurrentValue(uint32_t) const;
    void enable(void) const;
    void disable(void) const;
    size_t getNumberOfMeasurementsForPhaseCurrentValue(void) const;
    void interpretPhaseA(const uint16_t value) const;
    void interpretPhaseB(const uint16_t value) const;

private:
    constexpr PhaseCurrentSensorImproved(const enum Description&  desc,
                                         const HalfBridge&        hBridge,
                                         const Adc::Channel&      adc1,
                                         const Adc::Channel&      adc2,
                                         const TIM_OCInitTypeDef& adcTrgoConf) :
        mDescription(desc), mHBridge(hBridge), mAdc1(adc1), mAdc2(adc2), mAdcTrgoConfiguration(adcTrgoConf){}

    void updateCurrentValue(void) const;
    void initialize(void) const;

    const enum Description mDescription;
    const HalfBridge& mHBridge;
    const Adc::Channel& mAdc1;
    const Adc::Channel& mAdc2;
    const TIM_OCInitTypeDef mAdcTrgoConfiguration;

    mutable float mPhaseCurrentValue = 0;
    mutable uint16_t mOffsetValue = 2000;
    mutable size_t mNumberOfMeasurementsForPhaseCurrentValue = MAX_NUMBER_OF_MEASUREMENTS;
    mutable size_t mMeasureCounter = 0;

    mutable os::Semaphore* mValueAvailableSemaphore = nullptr;

    friend class Factory<PhaseCurrentSensorImproved>;

    static std::array<
                      std::array<uint16_t, MAX_NUMBER_OF_MEASUREMENTS>,
                      Description::__ENUM__SIZE> MeasurementValueBuffer;
};

template<>
class Factory<PhaseCurrentSensorImproved>
{
#include "PhaseCurrentSensorImproved_config.h"

    Factory(void)
    {
        for (const auto& obj : Container) {
            obj.initialize();
        }

        // if TIM20 provides trigger signal for ADC34, this trigger has to be remaped
        SYSCFG_ADCTriggerRemapConfig(REMAPADCTRIGGER_ADC34_EXT6, ENABLE);
        SYSCFG_ADCTriggerRemapConfig(REMAPADCTRIGGER_ADC34_EXT5, ENABLE);

        SYSCFG_ADCTriggerRemapConfig(REMAPADCTRIGGER_ADC12_EXT3, ENABLE);
        SYSCFG_ADCTriggerRemapConfig(REMAPADCTRIGGER_ADC12_EXT2, ENABLE);
    }
public:

    template<enum PhaseCurrentSensorImproved::Description index>
    static constexpr const PhaseCurrentSensorImproved& get(void)
    {
        static_assert(Container[index].mHBridge.mDescription != hal::HalfBridge::Description::__ENUM__SIZE,
                      "Invalid Tim Object");
        static_assert(index != PhaseCurrentSensorImproved::Description::__ENUM__SIZE, "__ENUM__SIZE is not accessible");
        static_assert(Container[index].mDescription == index, "Wrong mapping between Description and Container");

        return Container[index];
    }

    template<typename U>
    friend const U& getFactory(void);
};
}

#endif /* SOURCES_PMD_PHASE_CURRENT_SENSOR_H_ */
