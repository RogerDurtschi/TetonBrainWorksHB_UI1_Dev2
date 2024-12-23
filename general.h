#include "nrf_delay.h"



// prototypes

void blink_led(void);

void adj_saddle_tilt(uint16_t);

uint16_t rd_rst_tilt_angle_from_flash(void);

void write_rst_tilt_angle_to_flash(uint16_t);

void control_motor(uint8_t);

void drive_saddle_up(void);

void drive_saddle_down(void);

void motor_stop(void);

void send_byte_to_tilter(uint32_t cmd);

void accel_smpl_timer_stop(void);

void accel_smpl_timer_start(void);

void start_sleep_tmr(int);

void stop_sleep_tmr();

static void button_event_handler(uint8_t, uint8_t);

int twoComplToInt16(int twoComplValue);

float calc_std_dev(float, float, int16_t dat[]); 

/**@brief   Nordic UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
//struct ble_nus_s
//{
//    uint8_t                         uuid_type;          /**< UUID type for Nordic UART Service Base UUID. */
//    uint16_t                        service_handle;     /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
//    ble_gatts_char_handles_t        tx_handles;         /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
//    ble_gatts_char_handles_t        rx_handles;         /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
//    //blcm_link_ctx_storage_t * const p_link_ctx_storage; /**< Pointer to link context storage with handles of all current connections and its context. */
//   // ble_nus_data_handler_t          data_handler;       /**< Event handler to be called for handling received data. */
//};
//void write_cal_tilt_angle_to_flash(uint16_t tilt_angle);