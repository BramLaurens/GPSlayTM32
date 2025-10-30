#include "main.h"
#include "cmsis_os.h"
#include "admin.h"

#include <stdint.h>
#include "math.h"

#define LSM303M_ADDR_7BIT  0x1E
#define LSM303M_ADDR       (LSM303M_ADDR_7BIT << 1)
#define LSM303A_ADDR_7BIT  0x19
#define LSM303A_ADDR      (LSM303A_ADDR_7BIT << 1)
#define M_PI 3.14159265358979323846f

#define CAL_SAMPLES       1500
#define CAL_DELAY_MS      10
#define DEG_RAD           (180.0f / M_PI)

#define compass_offset 2

// #define DEBUG_COMPASS      

typedef struct {
    float offx, offy, offz;
    float scalex, scaley, scalez;
} MagCalibration;

MagCalibration magCal;

HAL_StatusTypeDef status;

double externAngle = 0.0;
double angleAvgBuffer = 0.0;

void getlatestHeading(double *dest) {
    if(xSemaphoreTake(hCompass_Mutex, portMAX_DELAY) == pdTRUE) {
        // critical section
        *dest = externAngle;
        xSemaphoreGive(hCompass_Mutex);
    } else {
        // failed to take mutex, return some default value
        *dest = -1.0;
    }
}

/**
 * @brief Read magnetometer data from LSM303M
 *
 * @param hi2c I2C handle
 * @param mx Pointer to store X magnetometer data
 * @param my Pointer to store Y magnetometer data
 * @param mz Pointer to store Z magnetometer data
 * @return HAL_StatusTypeDef HAL status
 */
HAL_StatusTypeDef LSM303AGR_ReadMag(I2C_HandleTypeDef *hi2c, int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t reg = 0x68; // OUTX_L_M
    uint8_t buf[6];

    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(hi2c, (0x1E << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, 6, 100);
    if (r != HAL_OK) return r;

    *mx = (int16_t)((buf[1] << 8) | buf[0]);
    *my = (int16_t)((buf[3] << 8) | buf[2]);
    *mz = (int16_t)((buf[5] << 8) | buf[4]);

    return HAL_OK;
}

/**
 * @brief Read accelerometer data from LSM303A
 *
 * @param hi2c I2C handle
 * @param ax Pointer to store X acceleration
 * @param ay Pointer to store Y acceleration
 * @param az Pointer to store Z acceleration
 * @return HAL_StatusTypeDef HAL status
 */
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

double debug_mag_angle(int16_t mx, int16_t my)
{
    float ang = atan2f((float)my, (float)mx) * 180.0f / M_PI;
    ang = -ang;

    // Sensor is mounted 180 offset
    ang += 180.0f;
    if (ang < 0) ang += 360.0f;
    // char b[64];
    // sprintf(b, "MAG angle = %.1f deg  raw=%d,%d\r\n", ang, mx, my);
    // UART_puts(b);
    return ang;
}

/**
 * @brief Calculate tilt-compensated heading from magnetometer and accelerometer data
 *
 * @param mx Magnetometer X
 * @param my Magnetometer Y
 * @param mz Magnetometer Z
 * @param ax Accelerometer X
 * @param ay Accelerometer Y
 * @param az Accelerometer Z
 * @return float Heading in degrees (0-360)
 */
float LSM303_HeadingTiltComp(int16_t mx, int16_t my, int16_t mz,
                             int16_t ax, int16_t ay, int16_t az)
{
    // remap magnetometer axes to match accelerometer frame
    float fx =  (float)mx;
    float fy = -(float)my;
    float fz = -(float)mz;

    float fax = (float)ax;
    float fay = (float)ay;
    float faz = (float)az;

    // normalize accelerometer
    float normA = sqrtf(fax*fax + fay*fay + faz*faz);
    if (normA == 0) return NAN;
    fax /= normA;
    fay /= normA;
    faz /= normA;

    // roll and pitch
    float roll  = atan2f(fay, faz);
    float pitch = atan2f(-fax, sqrtf(fay*fay + faz*faz));

    float cosR = cosf(roll), sinR = sinf(roll);
    float cosP = cosf(pitch), sinP = sinf(pitch);

    // tilt compensation
    float Xh = fx * cosP + fz * sinP;
    float Yh = fx * sinR * sinP + fy * cosR - fz * sinR * cosP;

    float heading = atan2f(Yh, Xh) * 180.0f / M_PI;
    if (heading < 0.0f)
        heading += 360.0f;

    return heading;
}

void LSM303AGR_Calibrate(I2C_HandleTypeDef *hi2c, MagCalibration *cal)
{
    int16_t mx, my, mz;
    int16_t minx = 32767, miny = 32767, minz = 32767;
    int16_t maxx = -32768, maxy = -32768, maxz = -32768;

    UART_puts("=== Magnetometer Calibration ===\r\n");
    UART_puts("Rotate the board slowly in all directions...\r\n");

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < 15000) // ~15 seconds
    {
        if (LSM303AGR_ReadMag(hi2c, &mx, &my, &mz) == HAL_OK)
        {
            if (mx < minx) minx = mx;
            if (mx > maxx) maxx = mx;
            if (my < miny) miny = my;
            if (my > maxy) maxy = my;
            if (mz < minz) minz = mz;
            if (mz > maxz) maxz = mz;
        }
        osDelay(50);
    }

    cal->offx = (maxx + minx) / 2.0f;
    cal->offy = (maxy + miny) / 2.0f;
    cal->offz = (maxz + minz) / 2.0f;

    cal->scalex = (maxx - minx) / 2.0f;
    cal->scaley = (maxy - miny) / 2.0f;
    cal->scalez = (maxz - minz) / 2.0f;

    char msg[128];
    sprintf(msg,
            "Calibration done!\r\n"
            "Offsets: X=%.1f Y=%.1f Z=%.1f\r\n"
            "Scales : X=%.1f Y=%.1f Z=%.1f\r\n",
            cal->offx, cal->offy, cal->offz,
            cal->scalex, cal->scaley, cal->scalez);
    UART_puts(msg);
}

void LSM303AGR_ApplyCalibration(const MagCalibration *cal, int16_t mx, int16_t my, int16_t mz,
                                float *mx_corr, float *my_corr, float *mz_corr)
{
    float fx = mx - cal->offx;
    float fy = my - cal->offy;
    float fz = mz - cal->offz;

    float avg_scale = (cal->scalex + cal->scaley + cal->scalez) / 3.0f;

    *mx_corr = fx * (avg_scale / cal->scalex);
    *my_corr = fy * (avg_scale / cal->scaley);
    *mz_corr = fz * (avg_scale / cal->scalez);
}


/**
 * @brief Calculate and return the current heading angle from LSM303M
 *
 * @return double Heading angle in degrees
 */
double LSM303M_RawAngle()
{
    double angle;
    char msg[100];

    int16_t mx, my, mz;
    int16_t ax, ay, az;

    HAL_StatusTypeDef HALreturn;
    if ((HALreturn = LSM303AGR_ReadMag(&hi2c3, &mx, &my, &mz)) != HAL_OK) {
        UART_puts("LSM303M mag read error:  ");
        UART_putint(HALreturn);
        UART_puts("\r\n");
        return;
    }

    if (LSM303A_ReadAccel(&hi2c3, &ax, &ay, &az) != HAL_OK) {
        UART_puts("LSM303A accel read error\r\n");
        return;
    }

    // float mx_corr, my_corr, mz_corr;
    // LSM303AGR_ApplyCalibration(&magCal, mx, my, mz, &mx_corr, &my_corr, &mz_corr);

    // float heading = LSM303_HeadingTiltComp(mx_corr, my_corr, mz_corr, ax, ay, az);
    // sprintf(msg, "Heading: %.2f deg\r\n", heading);
    // UART_puts(msg);

    // snprintf(msg, sizeof(msg), "Mag: X=%d Y=%d Z=%d | Accel: X=%d Y=%d Z=%d\r\n", mx, my, mz, ax, ay, az);
    // UART_puts(msg);


    angle = debug_mag_angle(mx, my);
    return angle;
}

/**
 * @brief Initialize the LSM303M magnetometer
 *
 * @param hi2c I2C handle
 * @return int HAL status
 */
int LSM303AGR_Mag_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t cfg[2];

    // CFG_REG_A_M (0x60): Continuous mode, ODR = 10 Hz
    cfg[0] = 0x60;
    cfg[1] = 0b10000000;  // COMP_TEMP_EN=1, MD=00 (continuous), ODR=10Hz
    ret = HAL_I2C_Master_Transmit(hi2c, (0x1E << 1), cfg, 2, 100);
    if (ret != HAL_OK) return ret;

    // CFG_REG_B_M (0x61): Default gain config
    cfg[0] = 0x61;
    cfg[1] = 0x02;
    ret = HAL_I2C_Master_Transmit(hi2c, (0x1E << 1), cfg, 2, 100);
    if (ret != HAL_OK) return ret;

    // CFG_REG_C_M (0x62): BDU=1
    cfg[0] = 0x62;
    cfg[1] = 0x10;
    ret = HAL_I2C_Master_Transmit(hi2c, (0x1E << 1), cfg, 2, 100);
    if (ret != HAL_OK) return ret;

    HAL_Delay(10);
    return HAL_OK;
}

/**
 * @brief Initialize the LSM303A accelerometer
 *
 * @param hi2c I2C handle
 * @return int HAL status
 */
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

void LSM303AGR_CFG_Read(void)
{
    uint8_t cfg;
    HAL_I2C_Mem_Read(&hi2c3, (0x1E << 1), 0x60, I2C_MEMADD_SIZE_8BIT, &cfg, 1, 100);
    char msg[32];
    sprintf(msg, "CFG_REG_A_M=0x%02X\r\n", cfg);
    UART_puts(msg);
}


void Compass_Heading(void *argument)
{
    osDelay(2000); // wait for system to stabilize

    if ((status = HAL_I2C_IsDeviceReady(&hi2c3, LSM303M_ADDR, 2, 100)) == HAL_OK)
        UART_puts("Magnetometer aanwezig\r\n");
    else
        UART_puts("Magnetometer reageert niet!\r\n");
    UART_putint((unsigned int)status);

    if ((status = HAL_I2C_IsDeviceReady(&hi2c3, LSM303A_ADDR, 2, 100)) == HAL_OK)
        UART_puts("Accelerometer aanwezig\r\n");
    else
        UART_puts("Accelerometer reageert niet!\r\n");
    UART_putint((unsigned int)status);

    LSM303AGR_Mag_Init(&hi2c3);
    LSM303A_Init(&hi2c3);

    double filteredSin = 0.0;
    double filteredCos = 0.0;
    const double alpha = 0.1; // smoothing factor

    osDelay(1000);
    // LSM303AGR_CFG_Read();
    // LSM303AGR_Calibrate(&hi2c3, &magCal);

    // LSM303M_Calibrate(&hi2c3, &offx, &offy, &offz, &scalex, &scaley, &scalez);
    while (1)
    {
        char b[64];
        double a = LSM303M_RawAngle(); // in degrees
        double rad = a * M_PI / 180.0;

        // Low-pass filter on sin and cos components
        filteredSin = alpha * sin(rad) + (1.0 - alpha) * filteredSin;
        filteredCos = alpha * cos(rad) + (1.0 - alpha) * filteredCos;

        // Reconstruct filtered angle
        double filtAngle = atan2(filteredSin, filteredCos) * 180.0 / M_PI;
        if (filtAngle < 0) filtAngle += 360.0;

        #ifdef DEBUG_COMPASS
            sprintf(b, "        Raw angle: %.2f deg, Filtered angle: %.2f deg\r\n", a, filtAngle);
            UART_puts(b);
        #endif

        if (xSemaphoreTake(hCompass_Mutex, portMAX_DELAY) == pdTRUE) {
            externAngle = filtAngle;
            // sprintf(b, "Compass angle updated: %.2f deg\r\n", externAngle);
            // UART_puts(b);
            xSemaphoreGive(hCompass_Mutex);
        }

        osDelay(10); // 10 ms delay between samples
    }
}
