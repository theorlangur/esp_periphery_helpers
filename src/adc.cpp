#include "ph_adc.hpp"
#include "lib_misc_helpers.hpp"
#include "esp_adc/adc_cali_scheme.h"

namespace adc
{

    /**********************************************************************/
    /* Calibration                                                        */
    /**********************************************************************/
    Calibration::Calibration(adc_channel_t channel, adc_unit_t unit, adc_atten_t atten)
    {
        open(channel, unit, atten);
    }

    bool Calibration::open(adc_channel_t channel, adc_unit_t unit, adc_atten_t atten)
    {
        close();
        adc_cali_handle_t handle = NULL;
        esp_err_t ret = ESP_FAIL;
        bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if (!calibrated) {
            FMT_PRINT("calibration scheme version is Curve Fitting\n");
            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = unit,
                .chan = channel,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if (!calibrated) {
            ESP_LOGI(TAG, "calibration scheme version is Line Fitting\n");
            adc_cali_line_fitting_config_t cali_config = {
                .unit_id = unit,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
#endif

        m_Handle = handle;
        if (ret == ESP_OK) {
            FMT_PRINT("Calibration Success\n");
            return true;
        } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
            FMT_PRINT("eFuse not burnt, skip software calibration\n");
        } else {
            FMT_PRINT("Invalid arg or no memory\n");
        }
        return false;
    }

    void Calibration::close()
    {
        if (m_Handle)
        {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
            //ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
            ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(m_Handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
            //ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
            ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(m_Handle));
#endif
            m_Handle = nullptr;
        }
    }

    Calibration::~Calibration()
    {
        close();
    }


    /**********************************************************************/
    /* OneShot                                                            */
    /**********************************************************************/
    OneShot::OneShot(adc_channel_t channel, adc_unit_t unit, adc_atten_t atten):
        m_Calibration(channel, unit, atten)
    {
        open(channel, unit, atten);
    }

    OneShot::result_mv_t OneShot::read()
    {
        int val;
        ESP_ERROR_CHECK(adc_oneshot_read(m_Handle, m_Channel, &val));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
        if (m_Calibration.valid()) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(m_Calibration, val, &val));
            //ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][0]);
        }
        return val;
    }

    bool OneShot::open(adc_channel_t channel, adc_unit_t unit, adc_atten_t atten)
    {
        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = unit,
            .clk_src = {},
            .ulp_mode = adc_ulp_mode_t::ADC_ULP_MODE_DISABLE
        };
        if (adc_oneshot_new_unit(&init_config1, &m_Handle) != ESP_OK)
            return false;
        adc_oneshot_chan_cfg_t config = {
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_oneshot_config_channel(m_Handle, channel, &config) != ESP_OK)
            return false;

        m_Channel = channel;
        return true;
    }

    void OneShot::close()
    {
        if (m_Handle)
        {
            ESP_ERROR_CHECK(adc_oneshot_del_unit(m_Handle));
            m_Handle = nullptr;
        }
    }

    OneShot::~OneShot()
    {
        close();
    }
}
