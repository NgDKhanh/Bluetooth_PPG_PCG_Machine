#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"
#include "esp_timer.h"
#include <freertos/ringbuf.h>
#include "esp_log.h"
#include "esp_timer.h"

// Include I2S driver
#include <driver/i2s.h>

// Incluse SD card driver
#include "sdcard.h"

// Incluse MAX30102 driver
#include "max30102.h"

#define BUTTON_PIN 36

#define MAX30102_MEASURING_TASK_ID 0
#define INMP441_MEASURING_TASK_ID  1

static bool stopMeasuringFlag = false;
static uint8_t stopSavingDataToSDCardFlag = 0x00;

static bool notify_state = false;

#define SPP_TAG "SPP_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "ESP32_SPP_SERVER"
#define TEST_DATA_SIZE 1024  // 1 KB

#define MAX_FILE_NUM 25

static uint32_t spp_handle = 0;  // SPP handle for sending data

static int8_t indexSDCardFileName;
static char nameFilePPGSDCard[6];
static char nameFilePCGSDCard[6];

int8_t getIndexSDCardFileName(void)
{
    char nameFileToCheck[6];
    for (uint8_t indexFileName = 0; indexFileName < MAX_FILE_NUM; indexFileName++)
    {
        memset(nameFileToCheck, 0, sizeof(nameFileToCheck));
        sprintf(nameFileToCheck, "%s%hhd", "ppg", indexFileName);
        if (sdcard_checkFileNameExist(nameFileToCheck) == ESP_OK)
        {
            ESP_LOGI(__func__, "File name %s.txt has existed.", nameFileToCheck);
            continue;
        }
        else
        {
            ESP_LOGI(__func__, "File name %s.txt has not existed.", nameFileToCheck);
            return indexFileName;
            break;
        }   
    }
    ESP_LOGW(__func__, "SD card full!");
    return -1;
}

void send_spp_data(const char *data, size_t size) {
    if (spp_handle != 0) {
        esp_spp_write(spp_handle, size, (uint8_t *)data);
    } else {
        ESP_LOGE(SPP_TAG, "No SPP connection available to send data");
    }
}

static void data_send_task(void *pvParameters) {
    char data[TEST_DATA_SIZE];
    memset(data, 'A', TEST_DATA_SIZE);

    while (1) {
        if (spp_handle != 0) {
            int64_t start_time = esp_timer_get_time();
            send_spp_data(data, TEST_DATA_SIZE);
            int64_t end_time = esp_timer_get_time();

            double time_taken_sec = (end_time - start_time) / 1000000.0;
            double data_rate_kbps = (TEST_DATA_SIZE * 8) / (time_taken_sec * 1024);
            ESP_LOGI(SPP_TAG, "Data rate: %.2f Kbps", data_rate_kbps);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Send data every second
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32, param->srv_open.status, param->srv_open.handle);
        spp_handle = param->srv_open.handle;  // Store the handle for later use
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_PIN_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    default:
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
}

static void startBluetoothTask()
{
    bool ret;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_cb));
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(DEVICE_NAME));
    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    if (ret == ESP_OK)
    {
        notify_state = true;
    }
    else
    {
        ESP_LOGE(__func__, "Fail to start Bluetooth");
    }
}

// Connections to INMP441 I2S microphone
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

#define bufferCount 6
#define bufferLen 32
#define receiveBufferLen ((bufferLen * 32 / 8)  * bufferCount / 2) 

// Buffers to store data read from dma buffers
int16_t buffer16[receiveBufferLen / 4] = {0};
uint8_t buffer32[receiveBufferLen] = {0};

// Buffer for data to save to SD card
RingbufHandle_t buf_handle_max;
RingbufHandle_t buf_handle_inm;

// Data buffer to send to ringbuffer
static char data_max[400] = "";
static char data_inm[receiveBufferLen / 4 * 6] = ""; // Should not be to big. For some reason, I set its size 1536B and it fails ???

TaskHandle_t readMAXTask_handle = NULL;
TaskHandle_t readINMTask_handle = NULL;
TaskHandle_t saveToSDTask_handle = NULL;

SemaphoreHandle_t semaphoreFromButton_handle = NULL;
SemaphoreHandle_t semaphoreAfterMearsuring_handle = NULL;

/* Timer ISR */
void IRAM_ATTR vButtonInterruptISR( void * pvParameters )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Is it time for vATask() to run? */
    xHigherPriorityTaskWoken = pdFALSE;
    /* Unblock the task by releasing the semaphore. */
    xSemaphoreGiveFromISR( semaphoreFromButton_handle, &xHigherPriorityTaskWoken );

    /* Yield if xHigherPriorityTaskWoken is true.  The 
    actual macro used here is port specific. */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 * @brief Read data from MAX30102 and send to ring buffer
 * 
 * @param pvParameters 
 */
void max30102_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(max30102_initDesc(&dev, 0, 21, 22));

    struct max30102_record record;
    struct max30102_data data;

    if (max30102_readPartID(&dev) == ESP_OK) {
        ESP_LOGI(__func__, "Found MAX30102!");
    }
    else {
        ESP_LOGE(__func__, "Not found MAX30102");
    }

    if (max30102_init(0x1F, 4, 2, 1000, 118, 4096, &record, &dev) == ESP_OK) {
        ESP_LOGI(__func__, "Init OK");
    }
    else {
        ESP_LOGE(__func__, "Init fail!");
    }

    uint16_t samplesTaken = 0;
    char data_temp[16] = "";
    unsigned long red;
    unsigned long ir;
    while (1)
    {

        max30102_check(&record, &dev); //Check the sensor, read up to 3 samples

        while (max30102_available(&record)) //do we have new data?
        {
            samplesTaken++;

            // printf("%d,", max30102_getFIFORed(&record));
            // printf("%d", max30102_getFIFOIR(&record));
            // printf("\n");
            red = max30102_getFIFORed(&record);
            ir = max30102_getFIFOIR(&record);

            memset(data_temp, 0, sizeof(data_temp));
            sprintf(data_temp, "%lu,%lu\n", red, ir);
            strcat(data_max, data_temp);

            max30102_nextSample(&record); //We're finished with this sample so move to next sample
        }
        if (samplesTaken >= 25) {
            xRingbufferSend(buf_handle_max, data_max, sizeof(data_max), pdMS_TO_TICKS(5));
            samplesTaken = 0;
            memset(data_max, 0, sizeof(data_max));
        }

        if (true == stopMeasuringFlag)
        {
            stopSavingDataToSDCardFlag |= 1 << MAX30102_MEASURING_TASK_ID;
            vTaskDelete(NULL);
        }

    }
}


// Set up I2S Processor configuration
void i2s_install() {
  // Set up I2S Processor configuration
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 4000, // or 44100 if you like
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Ground the L/R pin on the INMP441.
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S |I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = bufferCount,
    .dma_buf_len = bufferLen,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

// Set I2S pin configuration
void i2s_setpin() {
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };

    i2s_set_pin(I2S_PORT, &pin_config);
}

/**
 * @brief Read data from INMP441 and send to ring buffer
 * 
 * @param pvParameters 
 */
void readINMP441Task(void* parameter) {

    // Set up I2C
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT);

    size_t bytesRead = 0;
    char data_temp[8] = ""; 

    while (1)
    {
        vTaskDelay(1); // Feed for watchdog, if not watchdog timer will be triggered!
        i2s_read(I2S_PORT, &buffer32, sizeof(buffer32), &bytesRead, 100);
        int samplesRead = bytesRead / 4;

        for (uint8_t i = 0; i < samplesRead; i++) {
            uint8_t mid = buffer32[i * 4 + 2];
            uint8_t msb = buffer32[i * 4 + 3];
            uint16_t raw = (((uint32_t)msb) << 8) + ((uint32_t)mid);
            memcpy(&buffer16[i], &raw, sizeof(raw)); // Copy so sign bits aren't interfered with somehow.
            // printf("%d %d %d\n", 3000, -3000, buffer16[i]);
            
            memset(data_temp, 0, sizeof(data_temp));
            sprintf(data_temp, "\n%d", buffer16[i]);
            strcat(data_inm, data_temp);
            
        }
        
        bool res = pdFALSE;
        while (res != pdTRUE)
        {
            res = xRingbufferSend(buf_handle_inm, data_inm, sizeof(data_inm), pdMS_TO_TICKS(10));
        }
        memset(data_inm, 0, sizeof(data_inm));

        if (true == stopMeasuringFlag)
        {
            stopSavingDataToSDCardFlag |= 1 << INMP441_MEASURING_TASK_ID;
            vTaskDelete(NULL);
        }
    }
    
}

/**
 * @brief Receive data from 2 ring buffers and save them to SD card
 * 
 * @param parameter 
 */
void saveINMPAndMAXToSDTask(void *parameter) {
    while(1) {
        size_t item_size1;
        size_t item_size2;

        //Receive an item from no-split INMP441 ring buffer
        char *item1 = (char *)xRingbufferReceive(buf_handle_inm, &item_size1, 1);

        //Check received item
        if (NULL != item1) {
            //Return Item
            // Serial.println("r");
            vRingbufferReturnItem(buf_handle_inm, (void *)item1);
            sdcard_writeDataToFile_noArgument(nameFilePCGSDCard, item1);
        } 

        //Receive an item from no-split MAX30102 ring buffer
        char *item2 = (char *)xRingbufferReceive(buf_handle_max, &item_size2, 1);

        //Check received item
        if (NULL != item2) {
            //Return Item
            // Serial.println("rev");
            vRingbufferReturnItem(buf_handle_max, (void *)item2);
            sdcard_writeDataToFile_noArgument(nameFilePPGSDCard, item2);
        } 

        if (0x03 == stopSavingDataToSDCardFlag)
        {
            xSemaphoreGive( semaphoreAfterMearsuring_handle );
            vTaskDelete(NULL);
        }
        
    }
}

/**
 * @brief Timer callback that trigger stop measuring and saving data
 *          to SD card flag
 * 
 * @param xTimer One shot timer
 */
void timerCallback(TimerHandle_t xTimer) {
    ESP_LOGI("TIMER", "One-shot timer expired");
    stopMeasuringFlag = true;
    // Add your code to handle the timer event here
}

/**
 * @brief Read PCG data form SD card and send them through Bluetooth.
 *      Data are form PCGx.txt file. Before sending pcg data, I will
 *      send the line "PCG\n" first. After done sending, I will send 
 *      the line "end PCG\n". This make receiver easy to differentiate
 *      PPG and PCG data. 
 * 
 */
void readPCGDataFromSDCardAndSendToBluetoothFunction() {
    char pathFile[64];
    sprintf(pathFile, "%s/%s.txt", mount_point, nameFilePCGSDCard);

    ESP_LOGI(__func__, "Opening file %s...", pathFile);
    FILE *file = fopen(pathFile, "r");

    if (file == NULL) {
        ESP_LOGE(__func__, "Failed to open file for reading.");
        return;
    }

    char dataBufferForSendingBluetooth[1300];

    memset(dataBufferForSendingBluetooth, 0, sizeof(dataBufferForSendingBluetooth));

    int measured_flag = false;

    while (measured_flag == false)
    {
        if (spp_handle && notify_state) {
            send_spp_data("PCG\n", sizeof("PCG\n"));
            vTaskDelay(20);

            while (fread(dataBufferForSendingBluetooth, sizeof(char), sizeof(dataBufferForSendingBluetooth), file) != NULL)
            {
                send_spp_data(dataBufferForSendingBluetooth, sizeof(dataBufferForSendingBluetooth));
                vTaskDelay(20);
            }
            ESP_LOGI(__func__, "Read file done!");

            fclose(file);
            measured_flag = true;

            send_spp_data("end PCG\n", sizeof("end PCG\n"));
            vTaskDelay(20);
        }
        else 
        {
            ESP_LOGE(__func__, "Bluetooth is not available");
            vTaskDelay(1000);
        }
    }
}

/**
 * @brief Read PPG data form SD card and send them through Bluetooth.
 *      Data are form PPGx.txt file. Before sending pcg data, I will
 *      send the line "PPG\n" first. After done sending, I will send 
 *      the line "end PPG\n". This make receiver easy to differentiate
 *      PPG and PCG data. 
 * 
 */
void readPPGDataFromSDCardAndSendToBluetoothFunction() {
    char pathFile[64];
    sprintf(pathFile, "%s/%s.txt", mount_point, nameFilePPGSDCard);

    ESP_LOGI(__func__, "Opening file %s...", pathFile);
    FILE *file = fopen(pathFile, "r");

    if (file == NULL) {
        ESP_LOGE(__func__, "Failed to open file for reading.");
        return;
    }

    char dataBufferForSendingBluetooth[1300];

    memset(dataBufferForSendingBluetooth, 0, sizeof(dataBufferForSendingBluetooth));

    int measured_flag = false;

    while (measured_flag == false)
    {
        if (spp_handle && notify_state) {
            send_spp_data("PPG\n", sizeof("PPG\n"));
            vTaskDelay(20);

            while (fread(dataBufferForSendingBluetooth, sizeof(char), sizeof(dataBufferForSendingBluetooth), file) != NULL)
            {
                send_spp_data(dataBufferForSendingBluetooth, sizeof(dataBufferForSendingBluetooth));
                vTaskDelay(20);
            }
            ESP_LOGI(__func__, "Read file done!");

            fclose(file);
            measured_flag = true;

            send_spp_data("end PPG\n", sizeof("end PPG\n"));
            vTaskDelay(20);
        }
        else 
        {
            ESP_LOGE(__func__, "Bluetooth is not available");
            vTaskDelay(1000);
        }
    }
}


void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Define the timer period (20 s)
    const TickType_t xTimerPeriod = pdMS_TO_TICKS(20000);

    // Create the one-shot timer
    TimerHandle_t xOneShotTimer = xTimerCreate(
        "OneShotTimer",           // Name of the timer
        xTimerPeriod,             // Timer period in ticks
        pdFALSE,                  // Set to pdFALSE for a one-shot timer
        (void*)0,                 // Timer ID (not used here)
        timerCallback             // Callback function
    );

    // Initialize SPI Bus
    ESP_LOGI(__func__, "Initialize SD card with SPI interface.");
    esp_vfs_fat_mount_config_t mount_config_t = MOUNT_CONFIG_DEFAULT();
    spi_bus_config_t spi_bus_config_t = SPI_BUS_CONFIG_DEFAULT();
    sdmmc_host_t host_t = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_PIN_NUM_CS;
    slot_config.host_id = host_t.slot;

    sdmmc_card_t SDCARD;
    ESP_ERROR_CHECK(sdcard_initialize(&mount_config_t, &SDCARD, &host_t, &spi_bus_config_t, &slot_config));

    // Initialise ring buffers
    buf_handle_max = xRingbufferCreate(1028 * 6, RINGBUF_TYPE_NOSPLIT);
    buf_handle_inm = xRingbufferCreate(1028 * 15, RINGBUF_TYPE_NOSPLIT);

    if(NULL == buf_handle_inm || NULL == buf_handle_max) 
    {
        ESP_LOGE(__func__, "Ring buffers create fail");
    }
    else
    {
        ESP_LOGI(__func__, "Ring buffers create OK");
    }

    // Set up I2C
    ESP_ERROR_CHECK(i2cdev_init()); 

    /* We are using the semaphore for start sending data through Bluetooth */
    semaphoreAfterMearsuring_handle = xSemaphoreCreateBinary();

    indexSDCardFileName = getIndexSDCardFileName();
    if (indexSDCardFileName == -1)
    {
        ESP_LOGW(__func__, "Deleting file with index = 0!");
        // Delete file
        indexSDCardFileName = 0;
    }
    else
    {
        ESP_LOGI(__func__, "Index file name: %hhd", indexSDCardFileName);
    }
    

    sprintf(nameFilePCGSDCard, "%s%hhd", "pcg", indexSDCardFileName);
    sprintf(nameFilePPGSDCard, "%s%hhd", "ppg", indexSDCardFileName);    

    // Create tasks
    xTaskCreatePinnedToCore(max30102_test, "max30102_test", 1024 * 5, &readMAXTask_handle, 6, NULL, 0);
    xTaskCreatePinnedToCore(readINMP441Task, "readINM411", 1024 * 15, &readINMTask_handle, 6, NULL, 0);  // ?? Make max30102 task and inm task have equal priority can make polling cycle of max3012 shorter ??
    xTaskCreatePinnedToCore(saveINMPAndMAXToSDTask, "saveToSD", 1024 * 10, &saveToSDTask_handle, 10, NULL, 1);

    // Start timer
    xTimerStart(xOneShotTimer, 0);

    if ( xSemaphoreTake (semaphoreAfterMearsuring_handle,  portMAX_DELAY) == pdTRUE )
    {
        // Start SPP accepter
        startBluetoothTask();
        
        // Read data from SD card and send them through Bluetooth
        readPPGDataFromSDCardAndSendToBluetoothFunction();
        readPCGDataFromSDCardAndSendToBluetoothFunction();
    }
}
