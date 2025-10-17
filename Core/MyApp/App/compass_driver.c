#include "main.h"
#include "cmsis_os.h"
#include "admin.h"

#include <stdint.h>
#include "math.h"

#define LSM303M_ADDR_7BIT  0x1E
#define LSM303M_ADDR       (LSM303M_ADDR_7BIT << 1)
#define LSM303A_ADDR_7BIT  0x19
#define LSM303A_ADDR      0x33 //(LSM303A_ADDR_7BIT << 1)
#define M_PI 3.14159265358979323846f
HAL_StatusTypeDef status;



HAL_StatusTypeDef LSM303M_ReadMag(I2C_HandleTypeDef *hi2c, int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t reg = 0x03; // OUT_X_H_M
    uint8_t buf[6];
    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(hi2c, LSM303M_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 6, 100);
    if (r != HAL_OK) return r;
    // datasheet: XH XL ZH ZL YH YL
    *mx = (int16_t)((buf[0] << 8) | buf[1]);
    *mz = (int16_t)((buf[2] << 8) | buf[3]);
    *my = (int16_t)((buf[4] << 8) | buf[5]);
    return HAL_OK;
}

HAL_StatusTypeDef LSM303A_ReadAccel(I2C_HandleTypeDef *hi2c, int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = 0x28 | 0x80; // OUT_X_L_A (auto-increment), LSB first; note: accelerometer uses low byte first in this device
    uint8_t buf[6];
    // some boards require setting MSB (0x80) for auto-increment when reading multiple bytes from accel
    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(hi2c, LSM303A_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 6, 100);
    if (r != HAL_OK) return r;
    // accel datasheet: OUT_X_L_A (28h), OUT_X_H_A (29h) etc.
    *ax = (int16_t)((buf[1] << 8) | buf[0]);
    *ay = (int16_t)((buf[3] << 8) | buf[2]);
    *az = (int16_t)((buf[5] << 8) | buf[4]);
    return HAL_OK;
}

float LSM303_HeadingTiltComp(int16_t mx, int16_t my, int16_t mz,
                             int16_t ax, int16_t ay, int16_t az)
{
    // naar float
    float fx = (float)mx;
    float fy = (float)my;
    float fz = (float)mz;

    float fax = (float)ax;
    float fay = (float)ay;
    float faz = (float)az;

    // normaliseer accelerometer
    float normA = sqrtf(fax*fax + fay*fay + faz*faz);
    if (normA == 0.0f) return NAN;
    fax /= normA;
    fay /= normA;
    faz /= normA;

    // roll en pitch
    float roll  = atan2f(fay, faz);
    float pitch = atan2f(-fax, sqrtf(fay*fay + faz*faz));

    // tilt-compensatie
    float cosR = cosf(roll),  sinR = sinf(roll);
    float cosP = cosf(pitch), sinP = sinf(pitch);

    float Xh = fx * cosP + fz * sinP;
    float Yh = fx * sinR * sinP + fy * cosR - fz * sinR * cosP;

    // bereken heading in graden
    float heading = atan2f(-Yh, Xh) * 180.0f / M_PI;
    if (heading < 0.0f)
        heading += 360.0f;

    return heading;
}


void LSM303M_Test()
{
    int16_t mx, my, mz;
    int16_t ax, ay, az;
    if (LSM303M_ReadMag(&hi2c3, &mx, &my, &mz) != HAL_OK) {
        UART_puts("LSM303M mag read error\r\n");
        return;
    }
    if (LSM303A_ReadAccel(&hi2c3, &ax, &ay, &az) != HAL_OK) {
        UART_puts("LSM303A accel read error\r\n");
        UART_putint(LSM303A_ReadAccel(&hi2c3, &ax, &ay, &az));
        return;
    }
    float heading = LSM303_HeadingTiltComp(mx, my, mz, ax, ay, az);
    UART_puts("Heading: ");
    UART_putint(heading);
    UART_puts(" deg\r\n");
}

int LSM303M_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t cfg[2];

    // 1. CRA_REG_M (0x00): data rate = 15 Hz, temperatuurmeting uit
    cfg[0] = 0x00;
    cfg[1] = 0b00010000; // DO2..0 = 100 (15 Hz), TEMP_EN=0
    ret = HAL_I2C_Master_Transmit(hi2c, LSM303M_ADDR, cfg, 2, 100);
    if (ret != HAL_OK) return ret;

    // 2. CRB_REG_M (0x01): gain = ±1.3 gauss (default)
    cfg[0] = 0x01;
    cfg[1] = 0b00100000; // GN2..0 = 001
    ret = HAL_I2C_Master_Transmit(hi2c, LSM303M_ADDR, cfg, 2, 100);
    if (ret != HAL_OK) return ret;

    // 3. MR_REG_M (0x02): continuous-conversion mode
    cfg[0] = 0x02;
    cfg[1] = 0x00;
    ret = HAL_I2C_Master_Transmit(hi2c, LSM303M_ADDR, cfg, 2, 100);
    if (ret != HAL_OK) return ret;

    HAL_Delay(10); // kleine opstartpauze
    return HAL_OK;
}

int LSM303A_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t cfg[2];

    // CTRL_REG1_A (0x20): 10 Hz, alle assen aan
    cfg[0] = 0x20;
    cfg[1] = 0x27; // 0b00100111 -> normal mode, X/Y/Z enabled
    ret = HAL_I2C_Master_Transmit(hi2c, LSM303A_ADDR, cfg, 2, 100);
    if (ret != HAL_OK) return ret;

    // CTRL_REG4_A (0x23): ±2g, 16-bit
    cfg[0] = 0x23;
    cfg[1] = 0x00;
    ret = HAL_I2C_Master_Transmit(hi2c, LSM303A_ADDR, cfg, 2, 100);
    if (ret != HAL_OK) return ret;

    HAL_Delay(10);
    return HAL_OK;
}




void Compass_Heading(void *argument)
{
    if (status = HAL_I2C_IsDeviceReady(&hi2c3, LSM303A_ADDR, 2, 100) == HAL_OK)
    UART_puts("Accelerometer aanwezig\r\n");
    else
    UART_puts("Accelerometer reageert niet!\r\n");
    UART_putint(status);

    LSM303M_Init(&hi2c3);
    LSM303A_Init(&hi2c3);
    while (1)
    {
        LSM303M_Test();
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 seconde delay
    }
}