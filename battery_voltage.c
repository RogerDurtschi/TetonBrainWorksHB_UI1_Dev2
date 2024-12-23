#include "sdk_config.h"
#include "battery_voltage.h"
#include "nrf_drv_saadc.h"
#include "sdk_macros.h"
#include "nrf_log.h"

#define NRF_SAADC_INPUT_VDDHDIV5        10  //!< adc input is VDDH
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  600  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS 0  //!< diode not installed
#define ADC_RES_12BIT                  4095 //!< Maximum digital value for 12-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   6    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_12BIT) * ADC_PRE_SCALING_COMPENSATION)

//static nrf_saadc_value_t  adc_buf;                   //!< Buffer used for storing ADC value.
//static uint16_t           m_batt_lvl_in_milli_volts; //!< Current battery level.



/**@brief Function handling events from 'nrf_drv_saadc.c'.
 *
 * @param[in] p_evt SAADC event.
 */
/* Create an empty handler and pass this handler in the saadc initialization function
  > Normally this handler deals with the adc events but we are using blocking mode
  > In blocking mode the functions are called and the processor waits for the adc to finish taking samples from the respective channels
  > Event handler will not be called in this method
*/

void saadc_callback_handler(nrf_drv_saadc_evt_t const * p_event)
{
 // Empty handler function
}

// Create a function which configures the adc input pins and channels as well as the mode of operation of adc

void saadc_init(void)
{
	// A variable to hold the error code
  ret_code_t err_code;

  // Create a config struct and assign it default values along with the Pin number for ADC Input
  // Configure the input as Single Ended
  // allocate pin#9, AIN1
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);

  // Initialize the saadc 
  // first parameter is for configuring the adc resolution and other features
  err_code = nrf_drv_saadc_init(NULL, saadc_callback_handler);
  APP_ERROR_CHECK(err_code);

// Initialize the Channel, this case CHANNEL 0, to connect to pin9, AIN1.
  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);
}

//void battery_voltage_get(uint16_t * p_vbatt)
//{
//    VERIFY_PARAM_NOT_NULL_VOID(p_vbatt);

//    *p_vbatt = m_batt_lvl_in_milli_volts;
//    if (!nrfx_saadc_is_busy())
//    {
//        ret_code_t err_code = nrf_drv_saadc_buffer_convert(&adc_buf, 1);
//        APP_ERROR_CHECK(err_code);

//        err_code = nrf_drv_saadc_sample();
//        APP_ERROR_CHECK(err_code);
//    }
//}
