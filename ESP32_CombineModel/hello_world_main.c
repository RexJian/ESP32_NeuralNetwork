
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include "driver/gpio.h"
#include <string.h>
#include "driver/spi_master.h"
#include <math.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "sdkconfig.h"
#include "driver/uart.h"
/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * DEFINITIONS
 */
#define ADXL345_SPI_MISO_PIN        GPIO_NUM_19
#define ADXL345_SPI_MOSI_PIN        GPIO_NUM_23
#define ADXL345_SPI_SCLK_PIN        GPIO_NUM_18
#define ADXL345_SPI_CS_PIN          GPIO_NUM_5
#define SWITCH_GPIO_PIN             GPIO_NUM_34

#define DMA_CHAN                    2

#define SPI_CS_LOW                  ADXL345_SPI_CS_Set(0)
#define SPI_CS_HIGH                 ADXL345_SPI_CS_Set(1)
#define DEVICE_ID       0X00    // 器件ID,0XE5
#define THRESH_TAP      0X1D    // 敲击阀值寄存器
#define OFSX            0X1E
#define OFSY            0X1F
#define OFSZ            0X20
#define DUR             0X21
#define Latent          0X22
#define Window          0X23
#define THRESH_ACT      0X24    // 运动阈值寄存器
#define THRESH_INACT    0X25    // 静止阈值寄存器
#define TIME_INACT      0X26    // 静止时间         比例1 sec /LSB
#define ACT_INACT_CTL   0X27    // 启用运动/静止检测
#define THRESH_FF       0X28    // 自由下落阈值   建议采用300 mg与600 mg(0x05至0x09)之间的值 比例62.5 mg/LSB
#define TIME_FF         0X29    // 自由下落时间   建议采用100 ms与350 ms(0x14至0x46)之间的值 比例5ms/LSB
#define TAP_AXES        0X2A
#define ACT_TAP_STATUS  0X2B
#define BW_RATE         0X2C
#define POWER_CTL       0X2D
#define INT_ENABLE      0X2E    // 设置中断配置
#define INT_MAP         0X2F
#define INT_SOURCE      0X30
#define DATA_FORMAT     0X31
#define DATA_X0         0X32
#define DATA_X1         0X33
#define DATA_Y0         0X34
#define DATA_Y1         0X35
#define DATA_Z0         0X36
#define DATA_Z1         0X37
#define FIFO_CTL        0X38
#define FIFO_STATUS     0X39

#define Z_AXIS          0
#define X_AXIS          1
#define Y_AXIS          2
#define ECHO_TEST_TXD 1 //(CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD 3 //(CONFIG_EXAMPLE_UART_RXD)
// #define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
// #define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_UART_PORT_NUM     0 //  (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE    115200 //(CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    1024 // (CONFIG_EXAMPLE_TASK_STACK_SIZE)
#define BUF_SIZE (1024)
#define MAX_DATA_POINTS 1000
static void delayMs(uint32_t time);
static spi_device_handle_t s_spiHandle;
static const char *TAG = "UART TEST";

typedef struct {
    float x;
    float y;
    float z;
} DataPoint;
// static DataPoint data_xyz;
DataPoint *dataPoints = NULL;

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, 0, 0));

                                           
    // Configure a temporary buffer for the incoming data
    // uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

     while (1) {
        // Read data from the UART
        // int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        // // Write data back to the UART
        // uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
        // if (len) {
        //     data[len] = '\0';
        //     ESP_LOGI(TAG, "Recv str: %s", (char *) data);
        // }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
/*********************************************************************
 * API FUNCTIONS
 */
void ADXL345_SPI_Init(void);
void ADXL345_SPI_Write(uint8_t *pData, uint32_t dataLen);
void ADXL345_SPI_Read(uint8_t *pData, uint32_t dataLen);
void ADXL345_SPI_CS_Set(uint8_t level);
void ADXL345_Init(void);
uint8_t ADXL345_GetDeviceId(void);
void ADXL345_ReadXYZ(short *x, short *y, short *z);
void ADXL345_ReadAverage(short *x, short *y, short *z, uint8_t times);
void ADXL345_WriteReg(uint8_t addr, uint8_t data);
uint8_t ADXL345_ReadReg(uint8_t addr);
float** upsample(float** datas, int length, int target_length);
float** upsample2(float** datas, int length, int target_length);
float get_median(float* nums,int len);
void insert_zeros(float** datas,int idx);

// float sigmoid3(float x) { return (float)1 / (1 + exp(-x)); }

int get_result(float* outputs , int length)
{
    int pred_ans = 0;
    float max = -999999;
    for (int i = 0; i < length; i++)
    {
        if (outputs[i] > max)
        {
            max = outputs[i];
            pred_ans = i; 
        }
    }
    return pred_ans;
}

void load_model_parameter_and_train(float** datas , int length)
{
    float hidden_weights[12][90] = {
      {-1.57 ,1.40 ,-0.92 ,-2.04 ,0.80 ,-0.43 ,-2.75 ,0.53 ,-0.06 ,-2.08 ,0.01 ,0.29 ,-1.06 ,-0.41 ,0.49 ,-0.10 ,0.14 ,-0.05 ,0.98 ,0.64 ,-0.70 ,1.72 ,-0.34 ,-0.77 ,1.68 ,-1.05 ,-0.40 ,2.08 ,-1.22 ,-0.23 ,1.38 ,-0.66 ,-0.21 ,1.33 ,-0.03 ,-0.49 ,2.09 ,0.48 ,-1.10 ,2.06 ,0.06 ,-0.87 ,0.92 ,-0.24 ,-0.24 ,0.02 ,-0.07 ,0.54 ,-0.15 ,-0.21 ,1.01 ,0.33 ,-0.58 ,1.16 ,0.56 ,-0.09 ,0.76 ,0.53 ,0.02 ,0.90 ,0.36 ,-0.27 ,1.67 ,0.42 ,-0.55 ,1.91 ,0.67 ,-0.54 ,1.60 ,0.52 ,0.19 ,1.24 ,0.14 ,0.71 ,0.91 ,-0.12 ,0.86 ,0.56 ,-0.20 ,0.75 ,0.14 ,-0.46 ,0.51 ,0.05 ,-1.00 ,0.10 ,0.06 ,-1.41 ,-0.04 ,-0.11},
      {0.23 ,-1.53 ,-0.60 ,1.32 ,-1.22 ,-0.74 ,2.42 ,-0.62 ,-0.95 ,2.21 ,-0.76 ,-1.43 ,1.15 ,-0.37 ,-1.59 ,-0.41 ,-0.34 ,-1.51 ,-1.41 ,-0.12 ,-1.11 ,-2.35 ,0.08 ,-0.54 ,-1.63 ,0.32 ,-0.86 ,-0.65 ,0.22 ,-0.56 ,0.13 ,-0.36 ,-0.19 ,0.44 ,-0.46 ,0.21 ,0.65 ,0.20 ,0.34 ,0.48 ,0.86 ,0.35 ,-0.02 ,0.94 ,0.59 ,-1.02 ,0.95 ,0.72 ,-1.24 ,0.88 ,0.62 ,-0.91 ,1.04 ,0.56 ,-0.72 ,1.15 ,0.81 ,-0.15 ,1.27 ,0.58 ,0.22 ,1.09 ,0.35 ,0.65 ,0.89 ,0.16 ,1.10 ,-0.23 ,0.20 ,1.82 ,-0.38 ,-0.39 ,2.08 ,-1.01 ,-0.54 ,1.12 ,-0.79 ,-0.20 ,-0.18 ,-0.96 ,0.56 ,-0.95 ,-0.45 ,0.75 ,-0.95 ,0.20 ,0.56 ,-0.91 ,0.66 ,0.41},
      {0.26 ,0.23 ,-0.06 ,-0.09 ,0.24 ,-0.33 ,-0.54 ,0.16 ,-0.52 ,-0.09 ,0.59 ,-0.63 ,0.81 ,0.32 ,-0.50 ,1.70 ,-0.18 ,-0.25 ,1.68 ,0.06 ,-0.04 ,1.23 ,-0.12 ,0.44 ,0.75 ,-0.41 ,0.98 ,0.90 ,-0.62 ,0.99 ,1.14 ,-0.38 ,1.02 ,0.84 ,-0.15 ,1.13 ,0.27 ,-0.54 ,1.34 ,0.02 ,-0.49 ,1.20 ,-0.76 ,0.43 ,0.65 ,-1.10 ,1.00 ,0.21 ,-0.77 ,1.02 ,-0.05 ,-0.31 ,0.91 ,-0.28 ,0.04 ,1.21 ,-0.56 ,-0.35 ,1.13 ,-0.60 ,-0.84 ,0.51 ,-0.42 ,-0.78 ,-0.02 ,-0.49 ,-0.76 ,-0.01 ,-0.75 ,-0.56 ,-0.28 ,-0.74 ,-0.38 ,-0.38 ,-0.91 ,-0.22 ,-0.34 ,-1.29 ,-0.10 ,-0.42 ,-1.45 ,0.06 ,-0.26 ,-1.56 ,0.51 ,-0.41 ,-1.76 ,1.04 ,-0.61 ,-1.93},
      {1.00 ,1.26 ,1.15 ,0.91 ,0.31 ,1.18 ,0.92 ,-0.71 ,1.25 ,0.68 ,-1.11 ,1.47 ,0.73 ,-0.97 ,1.48 ,0.82 ,-0.25 ,1.35 ,0.99 ,0.65 ,0.63 ,1.20 ,0.68 ,0.08 ,0.59 ,0.16 ,0.05 ,-0.21 ,0.60 ,-0.38 ,-0.71 ,1.04 ,-0.49 ,-0.97 ,1.26 ,-0.95 ,-0.96 ,1.00 ,-1.38 ,-0.93 ,0.69 ,-1.55 ,-0.81 ,0.11 ,-1.48 ,-0.54 ,-0.55 ,-1.20 ,-0.56 ,-0.93 ,-0.80 ,-0.84 ,-0.99 ,-0.69 ,-0.82 ,-0.57 ,-1.07 ,-0.96 ,-0.64 ,-0.95 ,-0.75 ,-1.08 ,-0.74 ,-0.73 ,-1.02 ,-0.56 ,-0.66 ,-0.19 ,-0.67 ,-1.03 ,0.15 ,-0.27 ,-1.22 ,0.60 ,0.13 ,-0.38 ,0.52 ,-0.23 ,0.40 ,0.71 ,-1.07 ,0.85 ,0.75 ,-1.61 ,0.86 ,0.19 ,-1.20 ,0.83 ,-0.35 ,-0.90},
      {0.04 ,0.06 ,1.01 ,-0.25 ,0.27 ,0.32 ,-0.51 ,-0.04 ,-0.24 ,-1.25 ,0.52 ,0.05 ,-1.77 ,-0.11 ,0.39 ,-1.38 ,-0.37 ,0.50 ,-1.28 ,-0.53 ,0.62 ,-0.50 ,-0.43 ,0.37 ,-0.13 ,-0.78 ,0.74 ,0.29 ,-1.22 ,0.70 ,0.67 ,-1.29 ,0.52 ,0.63 ,-1.02 ,-0.00 ,0.27 ,-0.92 ,-0.28 ,0.14 ,-0.40 ,-0.69 ,-0.38 ,0.38 ,-1.13 ,-0.59 ,0.57 ,-1.34 ,-1.16 ,0.69 ,-0.92 ,-1.31 ,0.49 ,-0.48 ,-0.30 ,0.21 ,-0.38 ,0.34 ,0.50 ,-0.53 ,0.67 ,0.43 ,-0.53 ,1.35 ,0.27 ,-0.46 ,1.66 ,0.34 ,-0.35 ,1.58 ,-0.74 ,0.27 ,1.49 ,-0.51 ,0.29 ,0.98 ,0.22 ,0.29 ,0.70 ,0.70 ,0.07 ,0.71 ,0.75 ,-0.20 ,0.42 ,0.70 ,-0.07 ,-0.14 ,0.92 ,-0.04},
      {-0.87 ,-1.78 ,0.15 ,-1.28 ,0.03 ,-0.13 ,-1.06 ,1.09 ,-0.31 ,-0.50 ,0.56 ,-0.49 ,-0.39 ,0.24 ,-0.79 ,-0.70 ,-0.18 ,-0.52 ,-0.16 ,-1.22 ,0.07 ,0.09 ,-0.88 ,0.07 ,0.49 ,0.33 ,-0.49 ,-0.23 ,0.54 ,-0.03 ,-1.17 ,0.00 ,0.46 ,-1.77 ,-0.56 ,0.82 ,-1.26 ,-1.01 ,1.19 ,-0.92 ,-0.61 ,0.93 ,0.30 ,-0.32 ,0.64 ,1.43 ,0.13 ,-0.11 ,2.11 ,0.92 ,-0.56 ,1.77 ,1.73 ,-1.10 ,1.09 ,0.85 ,-0.90 ,0.58 ,0.06 ,-0.77 ,0.49 ,0.60 ,-1.04 ,0.32 ,0.05 ,-0.59 ,0.24 ,0.23 ,0.04 ,0.45 ,0.54 ,0.03 ,0.38 ,0.86 ,-0.11 ,0.86 ,-0.37 ,0.52 ,0.45 ,-0.53 ,1.21 ,0.10 ,-0.94 ,1.59 ,0.23 ,0.16 ,0.53 ,0.43 ,0.46 ,0.55},
      {0.26 ,-0.51 ,-0.33 ,0.45 ,-0.37 ,-0.35 ,0.38 ,-0.15 ,-0.42 ,-0.05 ,0.48 ,-0.38 ,-0.46 ,0.70 ,-0.27 ,-0.54 ,0.36 ,-0.22 ,-0.93 ,-0.22 ,0.07 ,-1.15 ,-0.17 ,0.24 ,-1.32 ,0.24 ,0.34 ,-1.11 ,0.33 ,0.26 ,-0.35 ,0.31 ,0.19 ,0.44 ,0.00 ,0.56 ,0.54 ,-0.08 ,1.11 ,0.66 ,-0.48 ,1.57 ,0.78 ,-0.85 ,1.73 ,0.91 ,-0.95 ,1.80 ,0.64 ,-0.98 ,1.63 ,0.31 ,-0.86 ,1.50 ,0.24 ,-0.56 ,1.40 ,-0.16 ,-0.53 ,1.23 ,-0.35 ,-0.22 ,0.73 ,-0.20 ,0.50 ,0.11 ,-0.31 ,0.81 ,-0.03 ,-0.37 ,0.42 ,0.12 ,-0.15 ,0.05 ,0.05 ,0.10 ,0.01 ,-0.16 ,0.29 ,0.02 ,-0.31 ,0.11 ,-0.17 ,-0.17 ,-0.16 ,-0.19 ,-0.08 ,-0.32 ,0.03 ,-0.18},
      {0.22 ,-0.21 ,-0.59 ,-0.01 ,0.16 ,-0.57 ,0.02 ,0.17 ,-0.40 ,-0.03 ,-0.04 ,-0.03 ,0.00 ,-0.43 ,0.28 ,-0.05 ,-0.44 ,0.49 ,-0.07 ,-0.11 ,0.51 ,0.08 ,0.18 ,0.49 ,0.14 ,0.27 ,0.60 ,-0.08 ,0.24 ,0.65 ,-0.40 ,0.06 ,0.71 ,-0.83 ,0.07 ,0.55 ,-0.64 ,-0.18 ,0.45 ,-0.33 ,0.04 ,0.25 ,-0.31 ,0.16 ,0.13 ,-0.41 ,-0.10 ,0.00 ,-0.48 ,-0.31 ,0.19 ,-0.44 ,-0.38 ,0.21 ,-0.29 ,-0.06 ,-0.09 ,-0.20 ,0.48 ,-0.30 ,0.02 ,0.48 ,-0.43 ,0.24 ,0.54 ,-0.48 ,0.10 ,0.67 ,-0.45 ,0.03 ,0.23 ,-0.17 ,0.09 ,-0.24 ,0.28 ,-0.01 ,-0.52 ,0.58 ,-0.09 ,-0.52 ,0.63 ,0.04 ,-0.15 ,0.33 ,0.11 ,-0.04 ,0.27 ,-0.07 ,-0.10 ,0.24},
      {-0.63 ,1.11 ,0.12 ,-0.08 ,0.65 ,-0.05 ,0.24 ,0.09 ,-0.24 ,-0.25 ,0.36 ,-0.00 ,-0.69 ,-0.26 ,0.18 ,-0.62 ,0.03 ,-0.14 ,-0.92 ,-0.02 ,-0.30 ,-0.53 ,-0.04 ,-0.63 ,-0.42 ,-0.04 ,-0.51 ,-0.19 ,0.37 ,-0.93 ,-0.06 ,0.63 ,-0.96 ,0.13 ,0.44 ,-0.91 ,-0.20 ,0.07 ,-0.83 ,-0.12 ,-0.41 ,-0.66 ,-0.15 ,-0.62 ,-0.53 ,-0.04 ,-0.80 ,-0.09 ,-0.59 ,-0.49 ,0.36 ,-0.75 ,0.17 ,0.46 ,-0.09 ,0.09 ,0.44 ,0.44 ,-0.19 ,0.27 ,0.34 ,-0.56 ,0.34 ,0.28 ,-0.22 ,0.22 ,0.75 ,0.07 ,-0.21 ,1.10 ,-0.37 ,-0.41 ,1.21 ,-0.10 ,-0.97 ,1.02 ,0.59 ,-1.34 ,1.14 ,0.91 ,-1.38 ,0.93 ,0.72 ,-1.10 ,0.24 ,0.36 ,-0.73 ,-0.62 ,0.72 ,-0.89},
      {0.35 ,-0.96 ,-0.01 ,-0.20 ,-0.09 ,0.18 ,-0.50 ,0.66 ,0.41 ,0.00 ,0.54 ,0.34 ,0.53 ,0.72 ,0.06 ,0.80 ,0.81 ,-0.26 ,1.00 ,0.46 ,-0.42 ,1.06 ,0.52 ,-0.56 ,0.96 ,0.80 ,-0.65 ,0.30 ,0.91 ,-0.32 ,-0.70 ,0.76 ,0.04 ,-1.56 ,0.36 ,0.41 ,-1.66 ,-0.32 ,0.77 ,-1.43 ,-0.49 ,0.84 ,-0.81 ,-0.61 ,0.79 ,-0.41 ,-0.74 ,0.57 ,-0.02 ,-0.70 ,0.31 ,0.05 ,-0.48 ,-0.12 ,-0.42 ,-0.67 ,-0.29 ,-0.38 ,-0.63 ,-0.36 ,-0.04 ,-0.14 ,-0.53 ,-0.04 ,0.20 ,-0.60 ,-0.24 ,0.38 ,-0.59 ,-0.34 ,0.61 ,-0.65 ,-0.28 ,-0.09 ,-0.27 ,-0.20 ,-0.99 ,0.23 ,-0.21 ,-1.17 ,0.60 ,-0.13 ,-1.00 ,0.75 ,0.02 ,-0.54 ,0.50 ,0.04 ,-0.26 ,0.39},
      {0.90 ,-0.11 ,0.42 ,1.18 ,-0.14 ,0.37 ,1.42 ,0.05 ,0.24 ,0.95 ,-0.20 ,0.00 ,0.23 ,0.08 ,0.13 ,-0.47 ,-0.50 ,0.76 ,-0.59 ,-0.62 ,1.16 ,-1.05 ,-0.32 ,1.43 ,-0.90 ,0.19 ,0.97 ,-0.72 ,0.35 ,0.64 ,0.16 ,0.15 ,0.24 ,0.78 ,0.05 ,0.06 ,0.95 ,0.23 ,-0.11 ,0.72 ,0.42 ,-0.29 ,0.59 ,0.56 ,-0.46 ,0.33 ,0.82 ,-0.69 ,0.39 ,0.86 ,-1.04 ,0.36 ,0.65 ,-0.96 ,0.27 ,0.75 ,-0.72 ,-0.33 ,0.55 ,-0.40 ,-0.75 ,0.16 ,-0.18 ,-0.82 ,-0.30 ,0.05 ,-0.96 ,-0.53 ,0.30 ,-0.84 ,-0.34 ,0.35 ,-0.59 ,-0.25 ,0.57 ,-0.24 ,-0.06 ,0.41 ,-0.33 ,0.13 ,0.19 ,-0.57 ,0.21 ,0.11 ,-0.28 ,-0.00 ,0.06 ,0.33 ,-0.65 ,0.25},
      {0.67 ,-1.08 ,0.70 ,0.21 ,-0.43 ,0.34 ,-0.15 ,0.13 ,0.09 ,0.11 ,0.18 ,-0.15 ,0.77 ,0.66 ,-0.59 ,0.87 ,1.21 ,-1.00 ,0.83 ,1.42 ,-1.18 ,0.55 ,1.07 ,-0.91 ,0.18 ,0.74 ,-0.58 ,0.20 ,0.40 ,-0.24 ,0.49 ,0.24 ,-0.15 ,0.27 ,0.03 ,-0.06 ,-0.03 ,-0.45 ,0.04 ,-0.28 ,-0.51 ,0.16 ,-0.73 ,-0.18 ,0.19 ,-1.05 ,-0.01 ,0.05 ,-0.89 ,-0.22 ,0.06 ,-0.38 ,-0.43 ,0.21 ,0.01 ,-0.51 ,0.44 ,-0.22 ,-0.56 ,0.84 ,-0.45 ,-0.45 ,1.07 ,-0.18 ,-0.17 ,1.02 ,-0.14 ,-0.09 ,0.88 ,0.10 ,-0.37 ,0.88 ,0.14 ,-0.36 ,0.88 ,0.02 ,0.00 ,0.83 ,-0.20 ,-0.12 ,0.97 ,-0.26 ,-0.19 ,0.92 ,-0.09 ,-0.55 ,0.84 ,-0.03 ,-1.12 ,0.91}
    };

    float hidden_layer_bias[12] = {-1.91 ,0.21 ,-1.03 ,0.91 ,-1.58 ,-0.98 ,-3.09 ,1.74 ,1.90 ,0.98 ,-1.31 ,-0.19};

    float output_weights[14][12] = {
        {-0.13 ,3.93 ,-1.65 ,-4.21 ,-4.99 ,4.11 ,-4.00 ,-2.23 ,-2.64 ,-1.41 ,-0.13 ,0.67},
        {-6.94 ,2.13 ,2.50 ,0.99 ,3.50 ,3.13 ,-3.30 ,-2.38 ,-0.49 ,-2.03 ,0.17 ,-2.70},
        {4.20 ,-2.30 ,3.79 ,0.22 ,1.37 ,-4.11 ,-4.57 ,-2.61 ,2.02 ,-2.36 ,-1.99 ,-2.67},
        {1.41 ,-2.73 ,-1.92 ,2.67 ,-2.46 ,2.32 ,-3.85 ,-0.37 ,-0.70 ,2.88 ,-4.15 ,-3.60},
        {3.38 ,2.55 ,-2.58 ,-2.70 ,-2.09 ,-3.72 ,2.27 ,-4.18 ,3.24 ,-3.33 ,-1.50 ,-0.97},
        {-4.66 ,-4.00 ,-3.63 ,-2.28 ,-1.58 ,3.10 ,2.97 ,-1.21 ,-3.53 ,1.92 ,0.94 ,-0.99},
        {1.63 ,-5.42 ,-3.87 ,3.73 ,-3.03 ,-1.29 ,-2.36 ,-2.58 ,-2.99 ,-3.73 ,2.02 ,1.64},
        {0.10 ,-0.71 ,4.30 ,-3.86 ,-1.67 ,-2.14 ,2.91 ,-0.81 ,-3.72 ,-2.80 ,1.53 ,-3.56},
        {0.61 ,2.34 ,-3.71 ,-2.35 ,4.47 ,-4.88 ,-4.83 ,-0.74 ,-1.32 ,-1.77 ,-1.41 ,0.93},
        {-4.64 ,-0.70 ,-3.14 ,2.62 ,-2.10 ,-3.98 ,2.29 ,-0.61 ,1.00 ,-2.76 ,1.97 ,-4.53},
        {2.73 ,-2.88 ,2.87 ,-2.00 ,-2.80 ,-0.96 ,2.33 ,-2.01 ,-1.02 ,1.19 ,-6.92 ,1.49},
        {2.51 ,-4.91 ,-3.58 ,-3.88 ,3.83 ,3.00 ,0.50 ,-1.87 ,0.78 ,-3.09 ,-1.24 ,-1.51},
        {-4.94 ,3.38 ,-1.70 ,-0.94 ,-0.41 ,0.23 ,1.20 ,-0.54 ,1.14 ,1.31 ,-5.85 ,0.50},
        {-5.32 ,-3.22 ,3.18 ,0.85 ,-2.90 ,-3.41 ,-2.85 ,-1.56 ,-1.17 ,1.80 ,1.00 ,1.47}
    };

    float output_layer_bias[14] = {-1.66 ,-2.19 ,-2.41 ,-1.75 ,-2.91 ,-1.68 ,-1.79 ,-1.77 ,-1.64 ,-1.29 ,-2.59 ,-2.06 ,-1.57 ,-1.31};

    float hidden_layer[12];
    float output_layer[14];

    //float* data_buf = pvPortMalloc(sizeof(float) * length * 3);
    float data_buf[90];
    int idx = 0;
    for(int i = 0; i < length ; i++)
    {
        for(int j = 0 ; j < 3 ; j++)
        {
            data_buf[idx] = datas[j][i];
            idx++;
        }
    }
    
    for (int i = 0; i < length * 3; i++)
    {
     for (int h = 0; h < 12; h++)
     {
         hidden_layer[h] += data_buf[i] * hidden_weights[h][i];
     }
    }
    
    for (int h = 0; h < 12; h++)
    {
     hidden_layer[h] += hidden_layer_bias[h];
    //  hidden_layer[h] = sigmoid3(hidden_layer[h]);
     hidden_layer[h] = 1 / (1 + exp(-hidden_layer[h]));
    // hidden_layer[h]=pow(2,2);
    // printf("%.2f",hidden_layer[h]);
    }

    for (int o = 0; o < 14; o++)
    {
     for (int h = 0; h < 12; h++)
     {
         output_layer[o] += hidden_layer[h] * output_weights[o][h];
     }
     output_layer[o] += output_layer_bias[o];
    //  output_layer[o] = sigmoid3(output_layer[o]);
    output_layer[o] = 1 / (1 + exp(-output_layer[o]));
    //printf("%.2f\n",output_layer[o]);
    }
    int pred = 0;
    pred = get_result(output_layer, 14);
    printf("Pred : %d\n",pred);
    //vPortFree(data_buf);
}

float** upsample(float** datas, int length, int target_length)
{
    int all_iterate_nums = (target_length - length) / (length + 1);
    int iterate_idx = (target_length - length) % (length + 1);
    float** resample_datas = (float**)pvPortMalloc(sizeof(float*) * 3);
    resample_datas[0] = (float*)pvPortMalloc(sizeof(float) * target_length);
    resample_datas[1] = (float*)pvPortMalloc(sizeof(float) * target_length);
    resample_datas[2] = (float*)pvPortMalloc(sizeof(float) * target_length);

    int k = 0;
    short insert_or_not = 1;
    for (int i = 0; i < target_length;)
    {
        if (insert_or_not)
        {
            for (int j = 0; j < all_iterate_nums; j++)
            {
                insert_zeros(resample_datas, i);
                i++;
            }
            if (k < iterate_idx)
            {
                insert_zeros(resample_datas, i);
                i++;
            }
            insert_or_not = 0;
        }
        else
        {
            resample_datas[0][i] = datas[0][k];
            resample_datas[1][i] = datas[1][k];
            resample_datas[2][i] = datas[2][k];
            k++;
            i++;
            insert_or_not = 1;
        }
    }

    vPortFree(datas[0]);
    vPortFree(datas[1]);
    vPortFree(datas[2]);
    return resample_datas;

}
float** upsample2(float** datas, int length, int target_length)
{
    float factor = (float)(length - 1) / (target_length - 1);
    float** resample_datas = (float**)pvPortMalloc(sizeof(float*) * 3);
    resample_datas[0] = (float*)pvPortMalloc(sizeof(float) * target_length);
    resample_datas[1] = (float*)pvPortMalloc(sizeof(float) * target_length);
    resample_datas[2] = (float*)pvPortMalloc(sizeof(float) * target_length);

    for (int i = 0; i < target_length; i++)
    {
        float index = i * factor;
        int lower_index = (int)index;
        int upper_index = lower_index + 1;
        float weight = index - lower_index;
        if (upper_index >= length)
        {
            resample_datas[0][i] = datas[0][lower_index];
            resample_datas[1][i] = datas[1][lower_index];
            resample_datas[2][i] = datas[2][lower_index];
        }
        else
        {
            resample_datas[0][i] = (1-weight)*datas[0][lower_index]+weight*datas[0][upper_index];
            resample_datas[1][i] = (1 - weight) * datas[1][lower_index] + weight * datas[1][upper_index];
            resample_datas[2][i] = (1 - weight) * datas[2][lower_index] + weight * datas[2][upper_index];
        }
    }
    vPortFree(datas[0]);
    vPortFree(datas[1]);
    vPortFree(datas[2]);
    return resample_datas;
}
float** downsample(float** datas, int length,int target_length)
{
    float** resample_datas = (float**)pvPortMalloc(sizeof(float*) * 3);
    resample_datas[0] = (float*)pvPortMalloc(sizeof(float) * target_length);
    resample_datas[1] = (float*)pvPortMalloc(sizeof(float) * target_length);
    resample_datas[2] = (float*)pvPortMalloc(sizeof(float) * target_length);
    float x_median = get_median(datas[0], length);
    float y_median = get_median(datas[1], length);
    float z_median = get_median(datas[2], length);
    float* diff_median = (float*)pvPortMalloc(sizeof(float) * length);
    float* diff_median_buf = (float*)pvPortMalloc(sizeof(float) * length);
    for (int i = 0; i < length; i++)
        diff_median[i] = fabs(datas[0][i] - x_median) + fabs(datas[1][i] - y_median) + fabs(datas[2][i] - z_median);

    //bubble sort
    float tmp = 0;
    for (int i = 0; i < length; i++)
        diff_median_buf[i] = diff_median[i];

    for (int i = length - 1; i > 0; i--)
    {
        for (int j = 0; j <= i - 1; j++)
        {
            if (diff_median_buf[j] > diff_median_buf[j + 1])
            {
                tmp = diff_median_buf[j];
                diff_median_buf[j] = diff_median_buf[j + 1];
                diff_median_buf[j + 1] = tmp;
            }
        }
    }

    for (int i = length-1; i > length-(length-target_length)-1; i--)
    {
        for (int j = 0; j < length; j++)
        {
            if (diff_median[j] == diff_median_buf[i])
            {
                diff_median[j] = 9999999;
                break;
            }
        }
    }

    //add data to resample data
    int j = 0;
    for(int i=0;i<length;i++)
    {
        if (diff_median[i] != 9999999)
        {
            resample_datas[0][j] = datas[0][i];
            resample_datas[1][j] = datas[1][i];
            resample_datas[2][j] = datas[2][i];
            j++;
        }
        if (j >= target_length)
            break;
    }

    //for (int j = 0; j < 23; j++)
    //  printf("x:%.2f  y:%.2f  z:%.2f\n", resample_datas[0][j], resample_datas[1][j], resample_datas[2][j]);
    vPortFree(datas[0]);
    vPortFree(datas[1]);
    vPortFree(datas[2]);

    return resample_datas;
}
float get_median(float* nums,int len)
{
    float median;
    float* nums_buf = (float*)pvPortMalloc(sizeof(float) * len);
    for (int i = 0; i < len; i++)
        nums_buf[i] = nums[i];
    float tmp;
    for (int i = 0; i < len; i++)
    {
        for (int j = 0; j < len - i - j; j++)
        {
            if (nums_buf[j] > nums_buf[j + 1])
            {
                tmp = nums_buf[j];
                nums_buf[j] = nums_buf[j + 1];
                nums_buf[j + 1] = tmp;
            }
        }
    }
    if (len % 2 == 0)
        median = (nums_buf[(len / 2) - 1] + nums_buf[(len / 2)]) / 2;
    else
        median = nums_buf[len / 2];
    vPortFree(nums_buf);
    //for (int i = 0; i < len; i++)
    //  printf("%.2f ", nums_buf[i]);
    return median;
}

void normalize_data(float* datas, int length)
{
 float min = datas[0];
 float max = datas[0];
 for (int i = 0; i < length; i++)
 {
  if (datas[i] < min)
   min = datas[i];
  if (datas[i] > max)
   max = datas[i];
 }
 for (int i = 0; i < length; i++)
  datas[i] = (datas[i] - min) / (max - min);
}

void insert_zeros(float** datas,int idx)
{
    datas[0][idx] = 0;
    datas[1][idx] = 0;
    datas[2][idx] = 0;
}
void ADXL345_Init(void)
{
    ADXL345_SPI_Init();

    while(ADXL345_GetDeviceId() != 0xE5)
    {
        printf("ADXL345 Init Fail!\n");
        delayMs(1000);
    }

    ADXL345_WriteReg(DATA_FORMAT, 0x0B);    // 13位全分辨率，输出数据右对齐，16g量程
    ADXL345_WriteReg(BW_RATE, 0x0A);        // 数据输出速度为100Hz
    ADXL345_WriteReg(POWER_CTL, 0x08);      // 无链接，测量模式
    ADXL345_WriteReg(INT_ENABLE, 0x80);     // DATA_READY中断
    printf("ADXL345 Init Success!\n");
}

uint8_t ADXL345_GetDeviceId(void)
{
    uint8_t ret = ADXL345_ReadReg(DEVICE_ID);
    return ret;
}

void ADXL345_ReadXYZ(short *x, short *y, short *z)
{
    uint8_t x0,y0,z0;
    uint8_t x1,y1,z1;
    int16_t x_buf,y_buf,z_buf;
    float x_buf2,y_buf2,z_buf2;
    int switch_state = -1;
    int previous_switch_state = -1;
    int state = -1;

    DataPoint *dataPoints = malloc(sizeof(DataPoint) * MAX_DATA_POINTS);
    // float** data_points = pvPortMalloc(sizeof(float*) * 3);
    int data_length=0;
    while (1)
    {
        x0 = ADXL345_ReadReg(DATA_X0);
        y0 = ADXL345_ReadReg(DATA_Y0);
        z0 = ADXL345_ReadReg(DATA_Z0);
        x1 = ADXL345_ReadReg(DATA_X1);
        y1 = ADXL345_ReadReg(DATA_Y1);
        z1 = ADXL345_ReadReg(DATA_Z1);
        *x = (short)(((uint16_t)x1 << 8) + x0); // DATA_X1为高位有效字节
        *y = (short)(((uint16_t)y1 << 8) + y0); // DATA_Y1为高位有效字节
        *z = (short)(((uint16_t)z1 << 8) + z0); // DATA_Z1为高位有效字节
        
        x_buf=(x1<<8)|x0;
        y_buf=(y1<<8)|y0;
        z_buf=(z1<<8)|z0;
        x_buf2=((float)x_buf/256)-0.04;
        y_buf2=(float)y_buf/256-0.01;
        z_buf2=(float)z_buf/256-0.05;
        
        switch_state = gpio_get_level(SWITCH_GPIO_PIN);
        
        if (state == 0 )
        {
            
            if (data_length < MAX_DATA_POINTS)
            {
                dataPoints[data_length].x = x_buf2;
                dataPoints[data_length].y = y_buf2;
                dataPoints[data_length].z = z_buf2;
                data_length++;
            }
        }

        if (switch_state != previous_switch_state)
        {
            //printf("state: %d\n", switch_state);
            if (switch_state == 1 && data_length != 0)
            {
                state = 1;  
                float** data_points = (float**)pvPortMalloc(sizeof(float*)*3);
                data_points[0] = (float*)pvPortMalloc(sizeof(float)*data_length);
                data_points[1] = (float*)pvPortMalloc(sizeof(float)*data_length);               
                data_points[2] = (float*)pvPortMalloc(sizeof(float)*data_length);              
                for(int i=0 ; i < data_length ; i++)
                {
                    data_points[0][i] = dataPoints[i].x;
                    data_points[1][i] = dataPoints[i].y;
                    data_points[2][i] = dataPoints[i].z;
                }
                if(data_length<30)
                    data_points = upsample2(data_points, data_length, 30);

                else
                    data_points = downsample(data_points , data_length , 30);
                normalize_data(data_points[0], 30);
                normalize_data(data_points[1], 30);
                normalize_data(data_points[2], 30);

                for (int i = 0; i < 30; i++)
                    printf("x:%.2f,y:%.2f,z:%.2f\n",data_points[0][i], data_points[1][i], data_points[2][i]);
                
                load_model_parameter_and_train(data_points , 30);
                data_length = 0;

            }
            else
            {
                state = 0;
            }
        }

        previous_switch_state = switch_state;

        vTaskDelay(1000 / portTICK_PERIOD_MS/5);
    }

    free(dataPoints); // Free the allocated memory when the task is done
    
}

/*读取ADXL345的数据并做滤波处理，读times次再取平均值*/
void ADXL345_ReadAverage(short *x, short *y, short *z, uint8_t times)
{
    if(0 == times)
    {
        return;
    }

    uint8_t i;
    short x_temp,y_temp,z_temp;
    *x = 0;
    *y = 0;
    *z = 0;

    for(i = 0; i < times; i++)
    {
        ADXL345_ReadXYZ(&x_temp, &y_temp, &z_temp);
        *x += x_temp;
        *y += y_temp;
        *z += z_temp;
        delayMs(5);
    }
    *x /= times;
    *y /= times;
    *z /= times;
}

/*使用偏移寄存器，进行偏移校准*/

/**
 @brief 写寄存器
 @param addr -[in] 寄存器地址
 @param data -[in] 写入数据
 @return 无
*/
void ADXL345_WriteReg(uint8_t addr, uint8_t data)
{
    uint8_t send_data[2];
    uint32_t size = 2;

    addr &= 0x3F;
    send_data[0] = addr;
    send_data[1] = data;

    SPI_CS_LOW;

    ADXL345_SPI_Write(send_data, size);

    SPI_CS_HIGH;
}

/**
 @brief 读寄存器
 @param addr -[in] 寄存器地址
 @return 读出一字节数据
*/
uint8_t ADXL345_ReadReg(uint8_t addr)
{
    uint8_t receive_data;
    uint32_t size = 1;

    addr &= 0x3F;
    addr |= 0x80;

    SPI_CS_LOW;

    ADXL345_SPI_Write(&addr, size);
    ADXL345_SPI_Read(&receive_data, size);

    SPI_CS_HIGH;

    return receive_data;
}

/**
 @brief 毫秒级延时函数
 @param time -[in] 延时时间（毫秒）
 @return 无
*/
static void delayMs(uint32_t time)
{
    vTaskDelay(time / portTICK_PERIOD_MS);
}

void ADXL345_SPI_Init(void)
{
    esp_err_t ret;

    spi_bus_config_t spiBusConfig =
    {
        .miso_io_num = ADXL345_SPI_MISO_PIN,                // MISO信号线
        .mosi_io_num = ADXL345_SPI_MOSI_PIN,                // MOSI信号线
        .sclk_io_num = ADXL345_SPI_SCLK_PIN,                // SCLK信号线
        .quadwp_io_num = -1,                                // WP信号线，专用于QSPI的D2
        .quadhd_io_num = -1,                                // HD信号线，专用于QSPI的D3
        .max_transfer_sz = 64 * 8,                          // 最大传输数据大小
    };

    spi_device_interface_config_t spiDeviceConfig =
    {
        .clock_speed_hz = SPI_MASTER_FREQ_10M,              // Clock out at 10 MHz,
        .mode = 3,                                          // SPI mode 0
        /*
         * The timing requirements to read the busy signal from the EEPROM cannot be easily emulated
         * by SPI transactions. We need to control CS pin by SW to check the busy signal manually.
         */
        .spics_io_num = -1,
        .queue_size = 7,                                    // 传输队列大小，决定了等待传输数据的数量
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(SPI3_HOST, &spiBusConfig, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI3_HOST, &spiDeviceConfig, &s_spiHandle);
    ESP_ERROR_CHECK(ret);

    // 配置CS引脚
    gpio_config_t io_conf;  // 定义一个gpio_config类型的结构体，下面的都算对其进行的配置
    io_conf.intr_type = GPIO_INTR_DISABLE;      // 禁止中断
    io_conf.mode = GPIO_MODE_INPUT;             // 选择输入模式
    io_conf.pin_bit_mask = (1ULL<<ADXL345_SPI_CS_PIN);  // 配置GPIO_IN寄存器
    io_conf.pull_down_en = 0;                   // 禁止下拉
    io_conf.pull_up_en = 0;                     // 禁止上拉
    gpio_config(&io_conf);                      // 最后配置使能

    gpio_set_direction(ADXL345_SPI_CS_PIN, GPIO_MODE_OUTPUT);// 把这个GPIO作为输出
}

/**
 @brief ADXL345 SPI写入数据
 @param pData -[in] 写入数据
 @param dataLen -[in] 写入数据长度
 @return 无
*/
void ADXL345_SPI_Write(uint8_t *pData, uint32_t dataLen)
{
    esp_err_t ret;
    spi_transaction_t t;
    if(0 == dataLen)                                        // no need to send anything
    {
        return;
    }

    memset(&t, 0, sizeof(t));                               // Zero out the transaction
    t.length = dataLen * 8;                                 // Len is in bytes, transaction length is in bits.
    t.tx_buffer = pData;                                    // Data
    ret = spi_device_polling_transmit(s_spiHandle, &t);     // Transmit!
    assert(ret == ESP_OK);                                  // Should have had no issues.
}

/**
 @brief ADXL345 SPI读取数据
 @param pData -[out] 读取数据
 @param dataLen -[in] 读取数据长度
 @return 无
*/
void ADXL345_SPI_Read(uint8_t *pData, uint32_t dataLen)
{
    spi_transaction_t t;
    if(0 == dataLen)                                        // no need to receivce anything
    {
        return;
    }

    memset(&t, 0, sizeof(t));                               // Zero out the transaction
    t.length = dataLen * 8;                                 // Len is in bytes, transaction length is in bits.
    t.rx_buffer = pData;
    esp_err_t ret = spi_device_polling_transmit(s_spiHandle, &t);
    assert(ret == ESP_OK);
}

/**
 @brief ADXL345 SPI CS引脚设置
 @param level -[in] 电平
 @return 无
*/
void ADXL345_SPI_CS_Set(uint8_t level)
{
    gpio_set_level(ADXL345_SPI_CS_PIN, level);
}
static void monitor_task(void *arg)
{

    while (1)
    {
        short x_value, y_value, z_value;
        short x_angle, y_angle, z_angle;
        ADXL345_ReadXYZ(&x_value,&y_value,&z_value);
        delayMs(5);
    }

   
}

void app_main(void)
{   
    ADXL345_Init();
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << SWITCH_GPIO_PIN);
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(monitor_task, "monitor_task", 8192, NULL, 4, NULL);
    

}






