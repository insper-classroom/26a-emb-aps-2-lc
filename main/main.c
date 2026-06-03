/**
 * voice-bridge v2 — Echo test com 16 kHz @ 12-bit
 *
 * Push-to-talk: aperta GP14 → grava do mic → envia ao PC via USB
 *               solta GP14 → PC manda o áudio de volta → toca no speaker
 *
 * Hardware:
 *   - GP14: switch do botão (pull-up interno)
 *   - GP4:  LED do botão (ânodo, com 220Ω em série)
 *   - GP27: AUD do MAX9814 (ADC1)
 *   - GP26: PWM → filtro RC → TPA2012 → speaker
 *
 * Áudio: 16 kHz, 12-bit, mono — 2 bytes por sample (little-endian)
 *
 * Protocolo USB (serial CDC):
 *   Pico → PC: "<<S>" inicia gravação, "<<E>" finaliza, entre eles samples de 2 bytes
 *   PC → Pico: "<<P>" inicia playback, "<<X>" finaliza, entre eles samples de 2 bytes
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define PIN_BTN   14
#define PIN_LED    4
#define PIN_MIC   27   // GP27 = ADC1
#define PIN_SPK   26   // PWM
#define ADC_CHAN   1

#define SAMPLE_RATE_HZ   16000
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)   // 62.5us
#define PWM_WRAP         4095   // 12-bit
#define PWM_MID          2048   // silêncio (ponto médio)

static QueueHandle_t xQueueSpkSamples;
static volatile bool recording = false;
static volatile bool playing   = false;

static void button_task(void *params) {
    (void) params;

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    gpio_init(PIN_BTN);
    gpio_set_dir(PIN_BTN, GPIO_IN);
    gpio_pull_up(PIN_BTN);

    const int STABLE_NEEDED = 10;  // 50ms de estabilidade
    int stable_count = 0;
    bool last_raw = false;
    bool stable_state = false;

    while (true) {
        bool raw = !gpio_get(PIN_BTN);

        if (raw == last_raw) {
            if (stable_count < STABLE_NEEDED) {
                stable_count++;
                if (stable_count == STABLE_NEEDED && raw != stable_state) {
                    stable_state = raw;
                    if (stable_state) {
                        recording = true;
                        gpio_put(PIN_LED, 1);
                        fwrite("<<S>", 1, 4, stdout);
                    } else {
                        recording = false;
                        gpio_put(PIN_LED, 0);
                        fwrite("<<E>", 1, 4, stdout);
                    }
                    fflush(stdout);
                }
            }
        } else {
            stable_count = 0;
            last_raw = raw;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void mic_task(void *params) {
    (void) params;

    adc_init();
    adc_gpio_init(PIN_MIC);
    adc_select_input(ADC_CHAN);

    while (true) {
        if (recording) {
            uint16_t raw = adc_read();  // 12-bit, 0..4095
            // Envia 2 bytes em little-endian
            putchar(raw & 0xFF);
            putchar((raw >> 8) & 0xFF);
            busy_wait_us(SAMPLE_PERIOD_US);
        } else {
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

static void spk_task(void *params) {
    (void) params;

    gpio_set_function(PIN_SPK, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_SPK);
    uint chan  = pwm_gpio_to_channel(PIN_SPK);

    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_clkdiv(slice, 1.0f);   // ~30.5 kHz com wrap 4095 (sysclk 125MHz)
    pwm_set_chan_level(slice, chan, PWM_MID);
    pwm_set_enabled(slice, true);

    uint16_t sample;
    while (true) {
        if (xQueueReceive(xQueueSpkSamples, &sample, portMAX_DELAY) == pdTRUE) {
            pwm_set_chan_level(slice, chan, sample);
            busy_wait_us(SAMPLE_PERIOD_US);
        }
    }
}

static void usb_rx_task(void *params) {
    (void) params;

    // Janela deslizante de 4 bytes pra detectar marcadores
    char window[4] = {0, 0, 0, 0};

    // Estado pra remontar samples de 2 bytes
    bool have_lsb = false;
    uint8_t lsb = 0;

    while (true) {
        int c = getchar_timeout_us(1000);
        if (c == PICO_ERROR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Detecta marcadores antes de tratar como sample
        window[0] = window[1];
        window[1] = window[2];
        window[2] = window[3];
        window[3] = (char) c;

        bool is_start = (window[0]=='<' && window[1]=='<' && window[2]=='P' && window[3]=='>');
        bool is_end   = (window[0]=='<' && window[1]=='<' && window[2]=='X' && window[3]=='>');

        if (is_start) {
            playing = true;
            have_lsb = false;
        } else if (is_end) {
            playing = false;
            have_lsb = false;
            uint16_t silence = PWM_MID;
            xQueueSend(xQueueSpkSamples, &silence, 0);
        } else if (playing) {
            // Remonta sample de 2 bytes em little-endian
            if (!have_lsb) {
                lsb = (uint8_t) c;
                have_lsb = true;
            } else {
                uint16_t sample = ((uint16_t)(uint8_t)c << 8) | lsb;
                if (sample > PWM_WRAP) sample = PWM_WRAP;
                xQueueSend(xQueueSpkSamples, &sample, 0);
                have_lsb = false;
            }
        }
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    xQueueSpkSamples = xQueueCreate(4096, sizeof(uint16_t));

    xTaskCreate(button_task, "btn", 1024, NULL, 3, NULL);
    xTaskCreate(mic_task,    "mic", 1024, NULL, 2, NULL);
    xTaskCreate(spk_task,    "spk", 1024, NULL, 2, NULL);
    xTaskCreate(usb_rx_task, "rx",  2048, NULL, 2, NULL);

    vTaskStartScheduler();
    while (true) { }
}