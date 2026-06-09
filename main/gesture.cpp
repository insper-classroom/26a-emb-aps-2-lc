/**
 * gesture.cpp — Detecção do gesto "updown" do IMU (MPU6050) por heurística leve.
 *
 * SEM Edge Impulse. Mede a oscilação vertical do acelerômetro e dispara "<<U>".
 * "Vertical" = eixo dominado pela gravidade (maior |média| na janela). Se ESSE
 * eixo oscila forte (pico-a-pico alto) E mais que os horizontais, é updown — o
 * que separa de wave (oscilação horizontal) e idle (parado).
 *
 * Só emite quando ocioso (sem gravar nem tocar) pra não corromper o stream de
 * áudio, que usa a mesma serial. IMU no i2c0: SDA=GP4, SCL=GP5, addr 0x68.
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "task.h"

// Estado do áudio (definido em main.c). Só emite o gesto quando ambos false.
extern "C" {
    extern volatile bool recording;
    extern volatile bool playing;
}

// --- IMU / I2C ---
#define IMU_I2C            i2c0
#define IMU_ADDR           0x68
#define IMU_SDA_GPIO       4
#define IMU_SCL_GPIO       5
#define I2C_TIMEOUT_US     10000     // nunca trava: se o sensor não responder, retorna erro

// --- Heurística ---
#define IMU_SAMPLE_MS         20     // ~50 Hz
#define IMU_WINDOW_SAMPLES    50     // janela de 1 s
#define UPDOWN_P2P_THRESHOLD  10000  // pico-a-pico (LSB) no eixo vertical p/ contar como updown (~0,6 g)
#define GESTURE_COOLDOWN_MS   3000   // evita disparo repetido

static void mpu6050_init(void) {
    i2c_init(IMU_I2C, 400 * 1000);
    gpio_set_function(IMU_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_GPIO);
    gpio_pull_up(IMU_SCL_GPIO);

    // Acorda o MPU6050 (PWR_MGMT_1 = 0)
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_timeout_us(IMU_I2C, IMU_ADDR, buf, 2, false, I2C_TIMEOUT_US);
}

// Lê os 3 eixos do acelerômetro. false se o I2C falhar (fiação/sensor).
static bool mpu6050_read_accel(int16_t accel[3]) {
    uint8_t buffer[6];
    uint8_t reg = 0x3B;  // ACCEL_XOUT_H
    if (i2c_write_timeout_us(IMU_I2C, IMU_ADDR, &reg, 1, true, I2C_TIMEOUT_US) < 0) return false;
    if (i2c_read_timeout_us(IMU_I2C, IMU_ADDR, buffer, 6, false, I2C_TIMEOUT_US) < 0) return false;
    for (int i = 0; i < 3; i++)
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
    return true;
}

static inline void window_reset(int32_t amin[3], int32_t amax[3], int32_t asum[3], int *count) {
    for (int i = 0; i < 3; i++) { amin[i] = 32767; amax[i] = -32768; asum[i] = 0; }
    *count = 0;
}

extern "C" void imu_task(void *params) {
    (void) params;
    mpu6050_init();
    vTaskDelay(pdMS_TO_TICKS(200));

    int32_t amin[3], amax[3], asum[3];
    int count;
    TickType_t cooldown_until = 0;
    window_reset(amin, amax, asum, &count);

    while (true) {
        // Áudio tem prioridade: enquanto grava/toca, não mede (descarta a janela).
        if (recording || playing) {
            window_reset(amin, amax, asum, &count);
            vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_MS));
            continue;
        }

        int16_t a[3];
        if (!mpu6050_read_accel(a)) {
            vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_MS));
            continue;
        }

        for (int i = 0; i < 3; i++) {
            if (a[i] < amin[i]) amin[i] = a[i];
            if (a[i] > amax[i]) amax[i] = a[i];
            asum[i] += a[i];
        }
        count++;

        if (count >= IMU_WINDOW_SAMPLES) {
            int p2p[3];
            int grav = 0;
            int32_t best_abs_mean = -1;
            for (int i = 0; i < 3; i++) {
                p2p[i] = (int)(amax[i] - amin[i]);
                int32_t mean = asum[i] / count;
                int32_t am = mean < 0 ? -mean : mean;
                if (am > best_abs_mean) { best_abs_mean = am; grav = i; }  // eixo da gravidade = vertical
            }

            // updown: o eixo vertical oscila forte E mais que os dois horizontais
            bool updown = (p2p[grav] > UPDOWN_P2P_THRESHOLD) &&
                          (p2p[grav] >= p2p[(grav + 1) % 3]) &&
                          (p2p[grav] >= p2p[(grav + 2) % 3]);

            if (updown && !recording && !playing && xTaskGetTickCount() >= cooldown_until) {
                fwrite("<<U>", 1, 4, stdout);
                fflush(stdout);
                cooldown_until = xTaskGetTickCount() + pdMS_TO_TICKS(GESTURE_COOLDOWN_MS);
            }

            window_reset(amin, amax, asum, &count);
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_MS));
    }
}
