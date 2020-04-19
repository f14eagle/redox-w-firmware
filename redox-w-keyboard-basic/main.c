/*
 * Either COMPILE_RIGHT or COMPILE_LEFT has to be defined from the make call to allow proper functionality
 */

#include "redox-w.h"
#include "nrf_drv_config.h"
#include "nrf_gzll.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf51_bitfields.h"
#include "nrf51.h"


/*****************************************************************************/
/** Configuration */
/*****************************************************************************/

const nrf_drv_rtc_t rtc_maint = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */
const nrf_drv_rtc_t rtc_deb = NRF_DRV_RTC_INSTANCE(1); /**< Declaring an instance of nrf_drv_rtc for RTC1. */

#ifdef COMPILE_LEFT
const uint32_t COL_PINS[COLUMNS] = { C01, C02, C03, C04, C05, C06, C07, C08, C09, C10, C11, C12, C13, C14 };
#endif
#ifdef COMPILE_RIGHT
const uint32_t COL_PINS[COLUMNS] = { C01, C02, C03, C04, C05 };
#endif
const unsigned short REMAINING_POSITIONS = 8 - 7;

// Define payload length
#define TX_PAYLOAD_LENGTH DROWS ///< 5 byte payload length when transmitting
// Data and acknowledgement payloads
static uint8_t data_payload[TX_PAYLOAD_LENGTH];                ///< Payload to send to Host.
static uint8_t ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; ///< Placeholder for received ACK payloads from Host.

// Debounce time (dependent on tick frequency)
#define DEBOUNCE 5
#define ACTIVITY 500

// Key buffers
static uint8_t keys[DROWS], keys_snapshot[DROWS], keys_buffer[DROWS];
static uint32_t debounce_ticks, activity_ticks;
static volatile bool debouncing = false;

// Debug helper variables
static volatile bool init_ok, enable_ok, push_ok, pop_ok, tx_success;

#ifdef COMPILE_LEFT
static uint8_t channel_table[3]={6, 44, 79};
#endif
#ifdef COMPILE_RIGHT
static uint8_t channel_table[3]={27, 65, 35};
#endif

// Setup switch pins with pullups
static void gpio_config(void)
{
    nrf_gpio_cfg_sense_input(R01, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R02, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R03, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R04, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(R05, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);

    #ifdef COMPILE_LEFT
    nrf_gpio_cfg_output(C01);
    nrf_gpio_cfg_output(C02);
    nrf_gpio_cfg_output(C03);
    nrf_gpio_cfg_output(C04);
    nrf_gpio_cfg_output(C05);
    nrf_gpio_cfg_output(C06);
    nrf_gpio_cfg_output(C07);
    nrf_gpio_cfg_output(C08);
    nrf_gpio_cfg_output(C09);
    nrf_gpio_cfg_output(C10);
    nrf_gpio_cfg_output(C11);
    nrf_gpio_cfg_output(C12);
    nrf_gpio_cfg_output(C13);
    nrf_gpio_cfg_output(C14);
    #endif
    #ifdef COMPILE_RIGHT
    nrf_gpio_cfg_output(C01);
    nrf_gpio_cfg_output(C02);
    nrf_gpio_cfg_output(C03);
    nrf_gpio_cfg_output(C04);
    nrf_gpio_cfg_output(C05);
    #endif
}

// Return the key states
static void read_keys(void)
{
    unsigned short c;
    volatile uint32_t input = 0;

    #ifdef COMPILE_LEFT
    uint8_t row_stat[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // scan matrix by columns
    for (c = 0; c < COLUMNS; ++c) {
        nrf_gpio_pin_set(COL_PINS[c]);
        input = NRF_GPIO->IN;
        if(c < 7){
            row_stat[0] = (row_stat[0] << 1) | ((input >> R01) & 1);
            row_stat[1] = (row_stat[1] << 1) | ((input >> R02) & 1);
            row_stat[2] = (row_stat[2] << 1) | ((input >> R03) & 1);
            row_stat[3] = (row_stat[3] << 1) | ((input >> R04) & 1);
            row_stat[4] = (row_stat[4] << 1) | ((input >> R05) & 1);
        }else{
            row_stat[5] = (row_stat[5] << 1) | ((input >> R01) & 1);
            row_stat[6] = (row_stat[6] << 1) | ((input >> R02) & 1);
            row_stat[7] = (row_stat[7] << 1) | ((input >> R03) & 1);
            row_stat[8] = (row_stat[8] << 1) | ((input >> R04) & 1);
            row_stat[9] = (row_stat[9] << 1) | ((input >> R05) & 1);
        }
        nrf_gpio_pin_clear(COL_PINS[c]);
    }

    keys_buffer[0] = row_stat[0] << REMAINING_POSITIONS;
    keys_buffer[1] = row_stat[1] << REMAINING_POSITIONS;
    keys_buffer[2] = row_stat[2] << REMAINING_POSITIONS;
    keys_buffer[3] = row_stat[3] << REMAINING_POSITIONS;
    keys_buffer[4] = row_stat[4] << REMAINING_POSITIONS;
    keys_buffer[5] = row_stat[5] << REMAINING_POSITIONS;
    keys_buffer[6] = row_stat[6] << REMAINING_POSITIONS;
    keys_buffer[7] = row_stat[7] << REMAINING_POSITIONS;
    keys_buffer[8] = row_stat[8] << REMAINING_POSITIONS;
    keys_buffer[9] = row_stat[9] << REMAINING_POSITIONS;
    #endif

    #ifdef COMPILE_RIGHT
    uint8_t row_stat[5] = {0, 0, 0, 0, 0};

    // scan matrix by columns
    for (c = 0; c < COLUMNS; ++c) {
        nrf_gpio_pin_set(COL_PINS[c]);
        input = NRF_GPIO->IN;
            row_stat[0] = (row_stat[0] << 1) | ((input >> R01) & 1);
            row_stat[1] = (row_stat[1] << 1) | ((input >> R02) & 1);
            row_stat[2] = (row_stat[2] << 1) | ((input >> R03) & 1);
            row_stat[3] = (row_stat[3] << 1) | ((input >> R04) & 1);
            row_stat[4] = (row_stat[4] << 1) | ((input >> R05) & 1);
        nrf_gpio_pin_clear(COL_PINS[c]);
    }

    keys_buffer[0] = row_stat[0] << REMAINING_POSITIONS;
    keys_buffer[1] = row_stat[1] << REMAINING_POSITIONS;
    keys_buffer[2] = row_stat[2] << REMAINING_POSITIONS;
    keys_buffer[3] = row_stat[3] << REMAINING_POSITIONS;
    keys_buffer[4] = row_stat[4] << REMAINING_POSITIONS;
    #endif

    return;
}

static bool compare_keys(uint8_t* first, uint8_t* second, uint32_t size)
{
    for(int i=0; i < size; i++)
    {
        if (first[i] != second[i])
        {
          return false;
        }
    }
    return true;
}

static bool empty_keys(void)
{
    for(int i=0; i < DROWS; i++)
    {
        if (keys_buffer[i])
        {
          return false;
        }
    }
    return true;
}

// Assemble packet and send to receiver
static void send_data(void)
{
    for(int i=0; i < DROWS; i++)
    {
        data_payload[i] = keys[i];
    }
    nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, data_payload, TX_PAYLOAD_LENGTH);
}

// 8Hz held key maintenance, keeping the reciever keystates valid
static void handler_maintenance(nrf_drv_rtc_int_type_t int_type)
{
    send_data();
}

// 1000Hz debounce sampling
static void handler_debounce(nrf_drv_rtc_int_type_t int_type)
{
    read_keys();

    // debouncing, waits until there have been no transitions in 5ms (assuming five 1ms ticks)
    if (debouncing)
    {
        // if debouncing, check if current keystates equal to the snapshot
        if (compare_keys(keys_snapshot, keys_buffer, DROWS))
        {
            // DEBOUNCE ticks of stable sampling needed before sending data
            debounce_ticks++;
            if (debounce_ticks == DEBOUNCE)
            {
                for(int j=0; j < DROWS; j++)
                {
                    keys[j] = keys_snapshot[j];
                }
                send_data();
            }
        }
        else
        {
            // if keys change, start period again
            debouncing = false;
        }
    }
    else
    {
        // if the keystate is different from the last data
        // sent to the receiver, start debouncing
        if (!compare_keys(keys, keys_buffer, DROWS))
        {
            for(int k=0; k < DROWS; k++)
            {
                keys_snapshot[k] = keys_buffer[k];
            }
            debouncing = true;
            debounce_ticks = 0;
        }
    }

    // looking for 500 ticks of no keys pressed, to go back to deep sleep
    if (empty_keys())
    {
        activity_ticks++;
        if (activity_ticks > ACTIVITY)
        {
            nrf_drv_rtc_disable(&rtc_maint);
            nrf_drv_rtc_disable(&rtc_deb);
            #ifdef COMPILE_LEFT
            nrf_gpio_pin_set(C01);
            nrf_gpio_pin_set(C02);
            nrf_gpio_pin_set(C03);
            nrf_gpio_pin_set(C04);
            nrf_gpio_pin_set(C05);
            nrf_gpio_pin_set(C06);
            nrf_gpio_pin_set(C07);
            nrf_gpio_pin_set(C08);
            nrf_gpio_pin_set(C09);
            nrf_gpio_pin_set(C10);
            nrf_gpio_pin_set(C11);
            nrf_gpio_pin_set(C12);
            nrf_gpio_pin_set(C13);
            nrf_gpio_pin_set(C14);
            #endif
            #ifdef COMPILE_RIGHT
            nrf_gpio_pin_set(C01);
            nrf_gpio_pin_set(C02);
            nrf_gpio_pin_set(C03);
            nrf_gpio_pin_set(C04);
            nrf_gpio_pin_set(C05);
            #endif
        }

    }
    else
    {
        activity_ticks = 0;
    }

}

// Low frequency clock configuration
static void lfclk_config(void)
{
    nrf_drv_clock_init();

    nrf_drv_clock_lfclk_request(NULL);
}

// RTC peripheral configuration
static void rtc_config(void)
{
    //Initialize RTC instance
    nrf_drv_rtc_init(&rtc_maint, NULL, handler_maintenance);
    nrf_drv_rtc_init(&rtc_deb, NULL, handler_debounce);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc_maint,true);
    nrf_drv_rtc_tick_enable(&rtc_deb,true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc_maint);
    nrf_drv_rtc_enable(&rtc_deb);
}

int main()
{
    // Initialize Gazell
    nrf_gzll_init(NRF_GZLL_MODE_DEVICE);

    // Attempt sending every packet up to 100 times
    nrf_gzll_set_max_tx_attempts(100);
    nrf_gzll_set_timeslots_per_channel(4);
    nrf_gzll_set_channel_table(channel_table,3);
    nrf_gzll_set_datarate(NRF_GZLL_DATARATE_1MBIT);
    nrf_gzll_set_timeslot_period(900);

    // Addressing
    nrf_gzll_set_base_address_0(0x04030201);
    nrf_gzll_set_base_address_1(0x08070605);

    // Enable Gazell to start sending over the air
    nrf_gzll_enable();

    // Configure 32kHz xtal oscillator
    lfclk_config();

    // Configure RTC peripherals with ticks
    rtc_config();

    // Configure all keys as inputs with pullups
    gpio_config();

    // Set the GPIOTE PORT event as interrupt source, and enable interrupts for GPIOTE
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_EnableIRQ(GPIOTE_IRQn);


    // Main loop, constantly sleep, waiting for RTC and gpio IRQs
    while(1)
    {
        __SEV();
        __WFE();
        __WFE();
    }
}

// This handler will be run after wakeup from system ON (GPIO wakeup)
void GPIOTE_IRQHandler(void)
{
    if(NRF_GPIOTE->EVENTS_PORT)
    {
        //clear wakeup event
        NRF_GPIOTE->EVENTS_PORT = 0;

        //enable rtc interupt triggers
        nrf_drv_rtc_enable(&rtc_maint);
        nrf_drv_rtc_enable(&rtc_deb);

        #ifdef COMPILE_LEFT
        nrf_gpio_pin_clear(C01);
        nrf_gpio_pin_clear(C02);
        nrf_gpio_pin_clear(C03);
        nrf_gpio_pin_clear(C04);
        nrf_gpio_pin_clear(C05);
        nrf_gpio_pin_clear(C06);
        nrf_gpio_pin_clear(C07);
        nrf_gpio_pin_clear(C08);
        nrf_gpio_pin_clear(C09);
        nrf_gpio_pin_clear(C10);
        nrf_gpio_pin_clear(C11);
        nrf_gpio_pin_clear(C12);
        nrf_gpio_pin_clear(C13);
        nrf_gpio_pin_clear(C14);
        #endif
        #ifdef COMPILE_RIGHT
        nrf_gpio_pin_clear(C01);
        nrf_gpio_pin_clear(C02);
        nrf_gpio_pin_clear(C03);
        nrf_gpio_pin_clear(C04);
        nrf_gpio_pin_clear(C05);
        #endif

        //TODO: proper interrupt handling to avoid fake interrupts because of matrix scanning
        //debouncing = false;
        //debounce_ticks = 0;
        activity_ticks = 0;
    }
}



/*****************************************************************************/
/** Gazell callback function definitions  */
/*****************************************************************************/

void  nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    uint32_t ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (tx_info.payload_received_in_ack)
    {
        // Pop packet and write first byte of the payload to the GPIO port.
        nrf_gzll_fetch_packet_from_rx_fifo(pipe, ack_payload, &ack_payload_length);
    }
}

// no action is taken when a packet fails to send, this might need to change
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{

}

// Callbacks not needed
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{}
void nrf_gzll_disabled()
{}
