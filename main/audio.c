/**
 * audio.c — Driver de áudio do controle de voz (push-to-talk, 16 kHz @ 12-bit).
 *
 * Encapsula a captura do microfone (ADC + DMA ping-pong), a reprodução no
 * speaker (PWM + ISR + ring buffer) e as tasks do FreeRTOS. Todo o estado de
 * baixo nível compartilhado entre ISRs e tasks vive aqui, fora do main.c.
 *
 * Hardware:
 *   - GP3:  switch do botão (pull-up interno)
 *   - GP15: LED de status
 *   - GP28: AUD do MAX9814 (ADC2)
 *   - GP26: PWM → filtro RC → TPA2012 → speaker
 *
 * Protocolo USB (serial CDC):
 *   Pico → PC: "<<S>" inicia gravação, "<<E>" finaliza, entre eles samples de 2 bytes
 *   PC → Pico: "<<P>" inicia playback, "<<X>" finaliza, entre eles samples de 2 bytes
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/structs/adc.h"
#include "FreeRTOS.h"
#include "task.h"

#include "audio.h"

#define PIN_BTN    3
#define PIN_LED   15   // LED de status
#define PIN_MIC   28   // GP28 = ADC2
#define PIN_SPK   26   // PWM
#define ADC_CHAN   2

#define SAMPLE_RATE_HZ   16000
#define PWM_WRAP         4095   // espaço de 12-bit das amostras (0..4095)
#define PWM_MID          2048   // silêncio (ponto médio do 12-bit)

// ADC roda a 48 MHz. 48e6 / 16000 = 3000 ciclos por amostra -> clkdiv = 2999.
#define ADC_CLKDIV       2999.0f

// Captura por DMA em ping-pong. Cada buffer = MIC_CHUNK amostras (~16 ms @ 16 kHz).
#define MIC_CHUNK        256

// Reprodução: portadora a 48 kHz (inaudível, filtrável no RC); a ISR alimenta uma
// amostra a cada SPK_DECIM wraps -> 16 kHz exatos.
#define SPK_CARRIER_HZ   48000
#define SPK_DECIM        3
#define SPK_WRAP         3124
#define SPK_SILENCE      (SPK_WRAP / 2)

// Ring buffer SPSC: usb_rx_task (produtor) enche, a ISR do PWM (consumidor) drena.
#define SPK_RING_SIZE    8192
#define SPK_RING_MASK    (SPK_RING_SIZE - 1)

static volatile bool recording = false;
static volatile bool playing   = false;

// --- Captura do microfone via DMA ---
static uint16_t mic_buf[2][MIC_CHUNK];   // ping-pong
static int      mic_dma_chan;
static volatile int  mic_cur        = 0; // buffer que o DMA está enchendo agora
static volatile int  mic_filled_buf = -1;// buffer pronto pra enviar (-1 = nenhum)
static volatile bool mic_overflow   = false;

// --- Reprodução no speaker ---
static uint16_t spk_ring[SPK_RING_SIZE];
static volatile uint32_t spk_head = 0;   // total de amostras escritas (produtor)
static volatile uint32_t spk_tail = 0;   // total de amostras lidas (consumidor/ISR)
static uint spk_slice, spk_chan;

// Consultado pela gesture.cpp pra só emitir o gesto quando o áudio está ocioso.
bool audio_is_busy(void) {
    return recording || playing;
}

// Produtor: empilha uma amostra (12-bit, 0..4095). Descarta se o ring estiver cheio.
static inline void spk_ring_push(uint16_t s) {
    if (spk_head - spk_tail < SPK_RING_SIZE) {
        spk_ring[spk_head & SPK_RING_MASK] = s;
        __asm volatile ("" ::: "memory");   // garante o dado antes de avançar head
        spk_head++;
    }
}

void button_task(void *params) {
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
                        gpio_put(PIN_LED, 1);
                        fwrite("<<S>", 1, 4, stdout);
                        fflush(stdout);
                        recording = true;   // só depois do marcador sair
                    } else {
                        recording = false;  // mic para de empurrar amostras
                        gpio_put(PIN_LED, 0);
                        fwrite("<<E>", 1, 4, stdout);
                        fflush(stdout);
                    }
                }
            }
        } else {
            stable_count = 0;
            last_raw = raw;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ISR de fim de transferência do DMA. Reinicia o ping-pong no outro buffer
// imediatamente e marca qual buffer ficou pronto. Não chama API do FreeRTOS.
static void mic_dma_isr(void) {
    if (dma_hw->ints0 & (1u << mic_dma_chan)) {
        dma_hw->ints0 = (1u << mic_dma_chan);   // limpa o IRQ

        int filled = mic_cur;
        mic_cur ^= 1;

        dma_channel_set_write_addr(mic_dma_chan, mic_buf[mic_cur], false);
        dma_channel_set_trans_count(mic_dma_chan, MIC_CHUNK, true);

        if (mic_filled_buf != -1) mic_overflow = true;
        mic_filled_buf = filled;
    }
}

static void mic_dma_start(void) {
    mic_cur = 0;
    mic_filled_buf = -1;
    adc_fifo_drain();
    dma_channel_set_write_addr(mic_dma_chan, mic_buf[mic_cur], false);
    dma_channel_set_irq0_enabled(mic_dma_chan, true);
    dma_channel_set_trans_count(mic_dma_chan, MIC_CHUNK, true);  // dispara
    adc_run(true);
}

static void mic_dma_stop(void) {
    adc_run(false);
    dma_channel_set_irq0_enabled(mic_dma_chan, false);
    dma_channel_abort(mic_dma_chan);
    dma_hw->ints0 = (1u << mic_dma_chan);   // limpa pendência residual
    adc_fifo_drain();
    mic_filled_buf = -1;
}

void mic_task(void *params) {
    (void) params;

    // ADC em modo free-running, clock de hardware a 16 kHz, FIFO + DMA.
    adc_init();
    adc_gpio_init(PIN_MIC);
    adc_select_input(ADC_CHAN);
    adc_fifo_setup(true, true, 1, false, false);
    adc_set_clkdiv(ADC_CLKDIV);   // 16 kHz exatos

    mic_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(mic_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);
    dma_channel_configure(mic_dma_chan, &c, mic_buf[0], &adc_hw->fifo, MIC_CHUNK, false);

    dma_channel_set_irq0_enabled(mic_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, mic_dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);

    bool was_recording = false;

    while (true) {
        if (recording) {
            if (!was_recording) {
                mic_overflow = false;
                mic_dma_start();
                was_recording = true;
            }

            int b = mic_filled_buf;
            if (b >= 0) {
                fwrite(mic_buf[b], sizeof(uint16_t), MIC_CHUNK, stdout);
                fflush(stdout);
                mic_filled_buf = -1;
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        } else {
            if (was_recording) {
                mic_dma_stop();
                was_recording = false;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// ISR do wrap do PWM (48 kHz). A cada SPK_DECIM ciclos puxa a próxima amostra do
// ring (16 kHz) e ajusta o duty. Fica na RAM pra não tomar latência de XIP.
static void __not_in_flash_func(spk_pwm_isr)(void) {
    pwm_clear_irq(spk_slice);

    static int decim = 0;
    if (++decim < SPK_DECIM) return;
    decim = 0;

    uint16_t level = SPK_SILENCE;
    if (spk_head != spk_tail) {                       // ring não vazio
        uint16_t s = spk_ring[spk_tail & SPK_RING_MASK];
        spk_tail++;
        level = (uint16_t)(((uint32_t) s * (SPK_WRAP + 1)) >> 12);
        if (level > SPK_WRAP) level = SPK_WRAP;
    }
    pwm_set_chan_level(spk_slice, spk_chan, level);
}

void spk_audio_init(void) {
    gpio_set_function(PIN_SPK, GPIO_FUNC_PWM);
    spk_slice = pwm_gpio_to_slice_num(PIN_SPK);
    spk_chan  = pwm_gpio_to_channel(PIN_SPK);

    pwm_clear_irq(spk_slice);
    pwm_set_irq_enabled(spk_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, spk_pwm_isr);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    float clkdiv = (float) clock_get_hz(clk_sys) / ((float) SPK_CARRIER_HZ * (SPK_WRAP + 1));

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clkdiv);
    pwm_config_set_wrap(&config, SPK_WRAP);
    pwm_init(spk_slice, &config, true);

    pwm_set_chan_level(spk_slice, spk_chan, SPK_SILENCE);
}

void usb_rx_task(void *params) {
    (void) params;

    char window[4] = {0, 0, 0, 0};
    bool have_lsb = false;
    uint8_t lsb = 0;

    while (true) {
        int c = getchar_timeout_us(1000);
        if (c == PICO_ERROR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

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
        } else if (playing) {
            if (!have_lsb) {
                lsb = (uint8_t) c;
                have_lsb = true;
            } else {
                uint16_t sample = ((uint16_t)(uint8_t)c << 8) | lsb;
                if (sample > PWM_WRAP) sample = PWM_WRAP;
                spk_ring_push(sample);
                have_lsb = false;
            }
        }
    }
}

// Mantém o TinyUSB vivo sob o FreeRTOS: chama stdio_flush() (que serviça o
// tud_task) a cada 1 ms. Prioridade alta pra responder o host rápido.
void usb_service_task(void *params) {
    (void) params;
    while (true) {
        stdio_flush();
        vTaskDelay(1);
    }
}
