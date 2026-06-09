/**
 * main.c — Controle de voz com gesto do IMU.
 *
 * Estruturado com FreeRTOS: cria as tasks e inicia o scheduler. A lógica de
 * cada subsistema fica encapsulada nos drivers (audio.c, gesture.cpp), sem
 * variáveis globais espalhadas no main.
 *
 * Tasks:
 *   - usb_service : mantém o TinyUSB vivo sob o FreeRTOS
 *   - button      : push-to-talk (GP3) + LED de status (GP15)
 *   - mic         : captura do microfone (ADC+DMA) e envio ao PC
 *   - usb_rx      : recebe áudio do PC e toca no speaker (PWM)
 *   - imu         : reconhece o gesto "updown" no MPU6050 e avisa o PC
 */

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "audio.h"

// Task de reconhecimento de gestos do IMU (definida em gesture.cpp).
extern void imu_task(void *params);

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    // Reprodução é dirigida por ISR do PWM (não por task) — inicializa aqui.
    spk_audio_init();

    xTaskCreate(usb_service_task, "usb", 1024, NULL, 4, NULL);
    xTaskCreate(button_task,      "btn", 1024, NULL, 3, NULL);
    xTaskCreate(mic_task,         "mic", 1024, NULL, 2, NULL);
    xTaskCreate(usb_rx_task,      "rx",  2048, NULL, 2, NULL);
    xTaskCreate(imu_task,         "imu", 2048, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true) { }
}
