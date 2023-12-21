
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
// float *x_buf3 = NULL;
// float *y_buf3 = NULL;
// float *z_buf3 = NULL;
// static void switch_task(void *arg)
// {
//     while (1)
//     {
//         int switch_state = gpio_get_level(SWITCH_GPIO_PIN);

//         // printf("state: %d\n", switch_state);
//         if (switch_state != previous_switch_state)
//         {
//             printf("state: %d\n", switch_state);
//             previous_switch_state = switch_state;
//         }

//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }
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
short ADXL345_GetAngle(float x, float y, float z, uint8_t direction);
void ADXL345_WriteReg(uint8_t addr, uint8_t data);
uint8_t ADXL345_ReadReg(uint8_t addr);
// void switch(void *arg)
// {
//     while (1)
//     {
//         int switch_state = gpio_get_level(SWITCH_GPIO_PIN);

//         printf("Switch state: %d\n", switch_state);

//         vTaskDelay(1000 / portTICK_PERIOD_MS); 
//     }
// }
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
    uint8_t x0, y0, z0;
    uint8_t x1, y1, z1;
    int16_t x_buf, y_buf, z_buf;
    float x_buf2, y_buf2, z_buf2;
    int switch_state = -1;
    int previous_switch_state = -1;
    int state = -1;
    int count = 0;
    int buf = 0;
    DataPoint *dataPoints = malloc(sizeof(DataPoint) * MAX_DATA_POINTS);
    int i = 0;

    while (1)
    {
        x0 = ADXL345_ReadReg(DATA_X0);
        y0 = ADXL345_ReadReg(DATA_Y0);
        z0 = ADXL345_ReadReg(DATA_Z0);
        x1 = ADXL345_ReadReg(DATA_X1);
        y1 = ADXL345_ReadReg(DATA_Y1);
        z1 = ADXL345_ReadReg(DATA_Z1);

        *x = (short)(((uint16_t)x1 << 8) + x0);
        *y = (short)(((uint16_t)y1 << 8) + y0);
        *z = (short)(((uint16_t)z1 << 8) + z0);

        x_buf = (x1 << 8) | x0;
        y_buf = (y1 << 8) | y0;
        z_buf = (z1 << 8) | z0;
        x_buf2 = ((float)x_buf / 256) - 0.04;
        y_buf2 = (float)y_buf / 256 - 0.01;
        z_buf2 = (float)z_buf / 256 - 0.05;

        switch_state = gpio_get_level(SWITCH_GPIO_PIN);

        if (state == 0)
        {
            if (i < MAX_DATA_POINTS)
            {
                dataPoints[i].x = x_buf2;
                dataPoints[i].y = y_buf2;
                dataPoints[i].z = z_buf2;
                i++;
                buf = 0;

            }
        }
        else if (switch_state == 1)
        {
           
            if (i < MAX_DATA_POINTS && buf == 0)
            {
                dataPoints[i].x = -100.0;
                dataPoints[i].y = -100.0;
                dataPoints[i].z = -100.0;
                buf = 1;
                i++;
            }
            else
            {
                dataPoints[i].x = x_buf2;
                dataPoints[i].y = y_buf2;
                dataPoints[i].z = z_buf2;
                i++;
            }
            // printf("-100, -100, -100\n");
        }

        if (switch_state != previous_switch_state)
        {
            printf("state: %d\n", switch_state);

            if (switch_state == 1)
            {
                state = 1;
                count++;

                if (count == 3)
                {
                    for (int j = 0; j < i; j++)
                    {
                        printf("%.2f, %.2f, %.2f\n", dataPoints[j].x, dataPoints[j].y, dataPoints[j].z);
                    }
                    i = 0;
                    count = 0;

                }
            }
            else
            {
                state = 0;
            }
        }

        previous_switch_state = switch_state;

        vTaskDelay(1000 / portTICK_PERIOD_MS / 5);
    }

    free(dataPoints);
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
void ADXL345_AutoAdjust(void)
{
    uint8_t i;
    short x_temp,y_temp,z_temp;
    short x_offset = 0;
    short y_offset = 0;
    short z_offset = 0;
    char x_calib = 0;
    char y_calib = 0;
    char z_calib = 0;

    ADXL345_WriteReg(DATA_FORMAT, 0x0B);    // 13位全分辨率，输出数据右对齐，16g量程
    ADXL345_WriteReg(BW_RATE, 0x0A);        // 数据输出速度为100Hz
    ADXL345_WriteReg(POWER_CTL, 0x08);      // 无链接，测量模式
    ADXL345_WriteReg(INT_ENABLE, 0x80);     // DATA_READY中断

    delayMs(12);

    for(i = 0; i < 10; i++)
    {
        ADXL345_ReadAverage(&x_temp, &y_temp, &z_temp, 10);
        x_offset += x_temp;
        y_offset += y_temp;
        z_offset += z_temp;
    }
    x_offset /= 10;
    y_offset /= 10;
    z_offset /= 10;

    x_calib =- x_offset / 4;
    y_calib =- y_offset / 4;
    z_calib =- (z_offset - 256) / 4;
    ADXL345_WriteReg(OFSX, x_calib);
    ADXL345_WriteReg(OFSY, y_calib);
    ADXL345_WriteReg(OFSZ, z_calib);
}

/*计算ADXL345角度，x/y/表示各方向上的加速度分量，direction表示要获得的角度*/
short ADXL345_GetAngle(float x, float y, float z, uint8_t direction)
{
    float temp;
    float res = 0;                          // 弧度值
    switch(direction)
    {
        case 0:                             // 0表示与Z轴的角度
            temp = sqrt((x*x + y*y)) / z;
            res = atan(temp);
            break;
        case 1:                             // 1表示与X轴的角度
            temp = x / sqrt((y*y + z*z));
            res = atan(temp);
            break;
        case 2:                             // 2表示与Y轴的角度
            temp = y / sqrt((x*x + z*z));
            res = atan(temp);
            break;
    }
    return res * 180 / 3.14;                // 返回角度值
    // return res*180/3.14*10;      //乘以10是为了取一位小数，角度精确到0.1°所以要乘以10
}

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
    // xTaskCreate(switch_task, "switch_task", 2048, NULL, 4, NULL);
    xTaskCreate(monitor_task, "monitor_task", 2048, NULL, 4, NULL);
}



