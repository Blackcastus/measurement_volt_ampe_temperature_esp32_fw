/*
* @Author: 				duchien 
* @Date:   				2024-12-23
* @Last Modified by:   	duchien
* @Last Modified time: 	2024-12-23
*/

/* ------------- INCLUDE ------------- */
// include system
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_event.h"
// include wifi
// #include "esp_wifi.h"
#include "connect_wifi.h"
// include gpio
#include "driver/gpio.h"
// include rtos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
// include modbus
#include "modbus_params.h"  // for modbus parameters structures
#include "mbcontroller.h"

// include mqtt
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"
#include "protocol_examples_common.h"



/* ------------- DEFINE ------------- */
#define GPIO_LED 2
#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED    (CONFIG_MB_UART_BAUD_RATE)  // The communication speed of the UART

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters
// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 10

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS          (500)
#define UPDATE_CIDS_TIMEOUT_TICS        (UPDATE_CIDS_TIMEOUT_MS / portTICK_PERIOD_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS                 (1)
#define POLL_TIMEOUT_TICS               (POLL_TIMEOUT_MS / portTICK_PERIOD_MS)

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char*)( fieldname ))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

/* ------------- DECLARE TASK HANDLE ------------- */
TaskHandle_t TASK_LED_Handle = NULL;
TaskHandle_t TASK_MODBUS_Handle = NULL;

/* ------------- DECLARE VARIABLE ------------- */
static const char* TAG_MAIN = "APP_MAIN";
static const char* TAG_MQTT = "MQTT";
static const char* TAG_MODBUS = "MODBUS";
// Enumeration of modbus device addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1 // Only one slave device used for the test (add other slave addresses here)
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_INP_DATA_0 = 0,
    CID_HOLD_DATA_0,
    CID_INP_DATA_1,
    CID_HOLD_DATA_1,
    CID_INP_DATA_2,
    CID_HOLD_DATA_2,
    CID_HOLD_TEST_REG,
    CID_RELAY_P1,
    CID_RELAY_P2,
    CID_DISCR_P1,
    CID_COUNT
};

// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    { CID_INP_DATA_0, STR("Volt"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_INPUT, 0, 10,
            INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 20, OPTS( -10, 2000, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    // { CID_INP_DATA_1, STR("Ampe"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_INPUT, 1, 3,
            // INPUT_OFFSET(input_data1), PARAM_TYPE_FLOAT, 4, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    // { CID_INP_DATA_1, STR("Ampe"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_INPUT, 1, 2,
    //         INPUT_OFFSET(input_data1), PARAM_TYPE_FLOAT, 4, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER }
    // { CID_INP_DATA_0, STR("Data_channel_2"), STR("Ampe"), MB_DEVICE_ADDR1, MB_PARAM_INPUT, 1, 4,
    //         INPUT_OFFSET(data_block1), PARAM_TYPE_FLOAT, 4, OPTS( -10, 10, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

/* ------------- DECLARE FUNCTION ------------- */

void Init_LED(void)
{
	ESP_LOGI(TAG_MAIN, "Example configured to blink GPIO LED!");
	gpio_reset_pin(GPIO_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
}

void Toggle_LED(uint8_t stt)
{
	gpio_set_level(GPIO_LED, stt);
}

void TaskLedHandle(void *params)
{
	uint8_t m_Led_Stt = 0;
	while (1)
	{
		Toggle_LED(m_Led_Stt);
		m_Led_Stt = !m_Led_Stt;
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG_MQTT, "Last error %s: 0x%x", message, error_code);
    }
}

void periodic_publish_task(void *pvParameters) {
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameters;
    char payload[16];

    while (1) {

        // int random_number = esp_random() % 100; 
        snprintf(payload, sizeof(payload), "%d", 10);


        int msg_id = esp_mqtt_client_publish(client, "iot", payload, 0, 1, 0);
        ESP_LOGI(TAG_MQTT, "Published random number: %s, msg_id=%d", payload, msg_id);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "iot", "data_3", 0, 1, 0);
        ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "iot", 0);
        ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_publish(client, "iot", "hello blackmeo", sizeof("hello blackmeo"), 0, 0);
        
        xTaskCreate(periodic_publish_task, "periodic_publish_task",2*1024, client, 2, NULL);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "iot", "hello blackmeo", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        ESP_LOGI(TAG_MQTT, "sent publish successful, len_msg = %d, msg_id = %d", sizeof("hello blackmeo"), msg_id);

        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG_MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL, //mqtt://34.97.119.55
        .broker.address.port = 1883,
        .credentials.client_id = "D01",
        .credentials.username = "duchien",
        .credentials.authentication.password = "duchien",
        .session.keepalive = 60,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// The function to get pointer to parameter storage (instance) according to parameter description table
static void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor)
{
    assert(param_descriptor != NULL);
    void* instance_ptr = NULL;
    if (param_descriptor->param_offset != 0) {
        printf("DEBUG: Slave addr = %d\n", param_descriptor->mb_slave_addr);
        printf("DEBUG: param_size = %d | mb_size = %d\n", param_descriptor->param_size, param_descriptor->mb_size);
       switch(param_descriptor->mb_param_type)
       {
           case MB_PARAM_HOLDING:
               instance_ptr = ((void*)&holding_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_INPUT:
               instance_ptr = ((void*)&input_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_COIL:
               instance_ptr = ((void*)&coil_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_DISCRETE:
               instance_ptr = ((void*)&discrete_reg_params + param_descriptor->param_offset - 1);
               break;
           default:
               instance_ptr = NULL;
               break;
       }
    } else {
        ESP_LOGE(TAG_MODBUS, "Wrong parameter offset for CID #%u", (unsigned)param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

// User operation function to read slave values and check alarm
static void TaskModbusHandle(void *arg)
{
    esp_err_t err = ESP_OK;
    float value = 0;
    bool alarm_state = false;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    ESP_LOGI(TAG_MODBUS, "Start modbus RTU...");

    while (1)
    {
        // Read all found characteristics from slave(s)
        for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++)
        {
            // Get data from parameters description table
            // and use this information to fill the characteristics description table
            // and having all required fields in just one table
            ESP_LOGI(TAG_MODBUS, "cid: %d\n", cid);
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                void* mb_data_ptr = master_get_param_data(param_descriptor);

                assert(mb_data_ptr);
                uint8_t type = 0;
                if ((param_descriptor->param_type == PARAM_TYPE_ASCII) &&
                        (param_descriptor->cid == CID_HOLD_TEST_REG)) 
                {
                            ESP_LOGI(TAG_MODBUS, "PARAM_TYPE_ASCII | CID_HOLD_TEST_REG");
                } 
                else 
                {
                    ESP_LOGI(TAG_MODBUS, "MB_PARAM_HOLDING | MB_PARAM_INPUT");
                    err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)mb_data_ptr, &type);
                    printf("DEBUG:");
                    if (err == ESP_OK) {
                        if (param_descriptor->mb_param_type == MB_PARAM_HOLDING) 
                        {

                        } 
                        else if (param_descriptor->mb_param_type == MB_PARAM_INPUT)
                        {
                            // value = *(float*)(mb_data_ptr);
                            uint16_t Voltage = *(uint16_t*)(mb_data_ptr);
                            uint32_t Current = (*((uint64_t*)mb_data_ptr) >> 16) & 0xFFFF;;
                            // uint32_t Power = (*(uint32_t*)mb_data_ptr >> 0xffffff);
                            // uint32_t Energy ;
                            // uint16_t Frequency ;
                            // uint16_t Power_factor ;
                            // uint16_t Alarm_status;
                            printf("Debug: data raw: %16llx\n", *(uint64_t*)mb_data_ptr);
                            printf("Debug: Volt: %d | Ampe: %ld\n", Voltage, Current);
                            ESP_LOGI(TAG_MODBUS, "Volt = %f (V), Ampe = %f (A)",
                                            (Voltage/10.0), Current/1000.0);
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG_MODBUS, "Characteristic #%u (%s) read fail, err = 0x%x (%s).",
                                        param_descriptor->cid,
                                        param_descriptor->param_key,
                                        (int)err,
                                        (char*)esp_err_to_name(err));
                    }
                }
                vTaskDelay(2000/portTICK_PERIOD_MS); // timeout between polls
            }
        }
        vTaskDelay(5000/portTICK_PERIOD_MS); // timeout between polls
    }
}

// Modbus master initialization
static esp_err_t Mobus_Master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
            .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
            .mode = MB_MODE_RTU,
#endif
            .baudrate = MB_DEV_SPEED,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG_MODBUS,
                                "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MODBUS,
                            "mb controller initialization fail, returns(0x%x).", (int)err);
    err = mbc_master_setup((void*)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MODBUS,
                            "mb controller setup fail, returns(0x%x).", (int)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MODBUS,
        "mb serial set pin failure, uart_set_pin() returned (0x%x).", (int)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MODBUS,
                            "mb controller start fail, returned (0x%x).", (int)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MODBUS,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MODBUS,
                                "mb controller set descriptor fail, returns(0x%x).", (int)err);
    ESP_LOGI(TAG_MODBUS, "Modbus master stack initialized...");
    return err;
}

void app_main(void)
{
	Init_LED();

	ESP_LOGI(TAG_MAIN, "[APP] Startup..");
    ESP_LOGI(TAG_MAIN, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG_MAIN, "[APP] IDF version: %s", esp_get_idf_version());
    // esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    // esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    // esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    // esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    // esp_log_level_set("transport", ESP_LOG_VERBOSE);
    // esp_log_level_set("outbox", ESP_LOG_VERBOSE);
	// ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
	// ESP_ERROR_CHECK(example_connect());
	// mqtt_app_start();

    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(Mobus_Master_init());
    vTaskDelay(10);

    xTaskCreate(&TaskModbusHandle, "TaskModbusHandle", 2048, NULL, 1, &TASK_LED_Handle);
    xTaskCreate(&TaskLedHandle, "TaskLedHandle", 2048, NULL, 2, &TASK_LED_Handle);
	
    while (1)
    {
        // eMBMasterReqReadInputRegister(1, 0, 1, 1000);
        vTaskDelay(3000/portTICK_PERIOD_MS);
    }
    

}