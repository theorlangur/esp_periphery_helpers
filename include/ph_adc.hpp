#ifndef PH_ADC_HPP_
#define PH_ADC_HPP_

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "lib_misc_helpers.hpp"
#include <utility>

namespace adc
{
    class Calibration: public NonCopyable
    {
    public:
        Calibration() = default;
        Calibration(Calibration &&rhs): m_Handle(rhs.m_Handle){ rhs.m_Handle = nullptr; }
        Calibration(adc_channel_t channel, adc_unit_t unit, adc_atten_t atten);
        ~Calibration();

        Calibration& operator=(Calibration &&rhs){ close(); m_Handle = std::exchange(rhs.m_Handle, nullptr); return *this; }

        bool open(adc_channel_t channel, adc_unit_t unit, adc_atten_t atten);
        void close();

        operator adc_cali_handle_t() const { return m_Handle; }
        bool valid() const { return m_Handle != nullptr; }
    private:
        adc_cali_handle_t m_Handle = nullptr;
    };

    class OneShot: public NonCopyable
    {
    public:
        using result_mv_t = int;

        OneShot() = default;
        OneShot(OneShot &&rhs): m_Handle(rhs.m_Handle), m_Channel(rhs.m_Channel), m_Calibration(std::move(rhs.m_Calibration)) { rhs.m_Handle = nullptr; }
        OneShot(adc_channel_t channel, adc_unit_t unit = ADC_UNIT_1, adc_atten_t atten = ADC_ATTEN_DB_12);
        ~OneShot();

        OneShot& operator=(OneShot &&rhs)
        {
            m_Handle = std::exchange(rhs.m_Handle, nullptr);
            m_Channel = rhs.m_Channel;
            m_Calibration = std::move(rhs.m_Calibration);
            return *this;
        }

        bool open(adc_channel_t channel, adc_unit_t unit = ADC_UNIT_1, adc_atten_t atten = ADC_ATTEN_DB_12);
        void close();

        result_mv_t read();
        bool valid() const { return m_Handle != nullptr; }
    private:
        adc_oneshot_unit_handle_t m_Handle = nullptr;
        adc_channel_t m_Channel;
        Calibration m_Calibration;
    };
}
#endif
