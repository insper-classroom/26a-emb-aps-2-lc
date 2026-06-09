/**
 * voice-bridge — Controle de voz com push-to-talk (16 kHz @ 12-bit)
 *
 * Push-to-talk: aperta GP3 → grava do mic → envia ao PC via USB
 *               solta GP3 → PC manda o áudio de volta → toca no speaker
 *
 * Hardware:
 *   - GP3:  switch do botão (pull-up interno)
 *   - GP15: LED de status
 *   - GP28: AUD do MAX9814 (ADC2)
 *   - GP26: PWM → filtro RC → TPA2012 → speaker
 *   - GP4/GP5: IMU MPU6050 (i2c0) — gesto updown (gesture.cpp)
 *
 * Áudio: 16 kHz, 12-bit, mono — 2 bytes por sample (little-endian)
 *
 * Protocolo USB (serial CDC):
 *   Pico → PC: "<<S>" inicia gravação, "<<E>" finaliza, entre eles samples de 2 bytes
 *   PC → Pico: "<<P>" inicia playback, "<<X>" finaliza, entre eles samples de 2 bytes
 *
 * RTOS: o estado compartilhado entre tasks e ISRs fica todo agrupado num único
 * contexto (ctrl_t g) em vez de variáveis globais soltas.
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

#define PIN_BTN    3
#define PIN_LED   15   // LED de status
#define PIN_MIC   28   // GP28 = ADC2
#define PIN_SPK   26   // PWM
#define ADC_CHAN   2

#define SAMPLE_RATE_HZ   16000
#define PWM_WRAP         4095   // espaço de 12-bit das amostras (0..4095)
#define PWM_MID          2048   // silêncio (ponto médio do 12-bit)

// ADC roda a 48 MHz. Período de amostra = (1 + clkdiv) ciclos.
// 48e6 / 16000 = 3000 ciclos por amostra  ->  clkdiv = 2999  ->  16 kHz exatos.
#define ADC_CLKDIV       2999.0f

// Captura por DMA em ping-pong: o hardware do ADC gera amostras a 16 kHz e o DMA
// drena a FIFO pra RAM sem CPU. Cada buffer = MIC_CHUNK amostras (~16 ms @ 16 kHz).
#define MIC_CHUNK        256

// --- Reprodução no speaker (estilo pico-pwm-audio: ISR do PWM, sem busy_wait) ---
// Portadora a 48 kHz (inaudível, filtrável no RC). A ISR do wrap alimenta uma
// amostra a cada SPK_DECIM wraps -> 16 kHz exatos. Igual ao truque do exemplo
// de repetir a amostra pra separar a portadora da taxa de amostragem.
#define SPK_CARRIER_HZ   48000
#define SPK_DECIM        3              // 48 kHz / 3 = 16 kHz (taxa de amostra)
#define SPK_WRAP         3124          // 3125 níveis (~11.6 bit); 150MHz/3125 = 48kHz
#define SPK_SILENCE      (SPK_WRAP / 2)

// Ring buffer SPSC: usb_rx_task (produtor) enche, a ISR do PWM (consumidor) drena.
#define SPK_RING_SIZE    8192          // potência de 2; ~0.5 s @ 16 kHz
#define SPK_RING_MASK    (SPK_RING_SIZE - 1)

// ============================================================================
// Contexto do firmware. Em vez de variáveis globais soltas (proibidas no RTOS),
// todo o estado compartilhado entre tasks/ISRs vive agrupado aqui, numa única
// instância. Os campos voláteis são tocados por ISR.
// ============================================================================
typedef struct {
    volatile bool recording;   // botão pressionado: mic empurrando amostras
    volatile bool playing;     // PC enviando áudio pro speaker

    // Captura do microfone via DMA (ping-pong)
    uint16_t      mic_buf[2][MIC_CHUNK];
    int           mic_dma_chan;
    volatile int  mic_cur;         // buffer que o DMA está enchendo agora
    volatile int  mic_filled_buf;  // buffer pronto pra enviar (-1 = nenhum)
    volatile bool mic_overflow;

    // Reprodução no speaker
    uint16_t          spk_ring[SPK_RING_SIZE];
    volatile uint32_t spk_head;    // total de amostras escritas (produtor)
    volatile uint32_t spk_tail;    // total de amostras lidas (consumidor/ISR)
    unsigned int      spk_slice;
    unsigned int      spk_chan;
} ctrl_t;

static ctrl_t g;

// Task de reconhecimento de gestos do IMU (definida em gesture.cpp).
extern void imu_task(void *params);

// Consultado pela gesture.cpp (outro arquivo) pra só emitir o gesto quando o
// controle está ocioso. Expor por função evita deixar o estado como global.
bool audio_is_busy(void) {
    return g.recording || g.playing;
}

// Produtor: empilha uma amostra (12-bit, 0..4095). Descarta se o ring estiver cheio.
static inline void spk_ring_push(uint16_t s) {
    if (g.spk_head - g.spk_tail < SPK_RING_SIZE) {
        g.spk_ring[g.spk_head & SPK_RING_MASK] = s;
        __asm volatile ("" ::: "memory");   // garante o dado antes de avançar head
        g.spk_head++;
    }
}

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
                        gpio_put(PIN_LED, 1);
                        fwrite("<<S>", 1, 4, stdout);
                        fflush(stdout);
                        g.recording = true;   // só depois do marcador sair
                    } else {
                        g.recording = false;  // mic para de empurrar amostras
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

// ISR de fim de transferência do DMA. Reinicia o DMA no outro buffer
// imediatamente (ping-pong, sem buraco de amostragem) e marca qual buffer ficou
// pronto. Não chama nenhuma API do FreeRTOS — só toca em registradores e flags
// volatile — então é seguro independente da prioridade do IRQ.
static void mic_dma_isr(void) {
    if (dma_hw->ints0 & (1u << g.mic_dma_chan)) {
        dma_hw->ints0 = (1u << g.mic_dma_chan);   // limpa o IRQ

        int filled = g.mic_cur;
        g.mic_cur ^= 1;

        // Reaponta e redispara o DMA no outro buffer. A FIFO (4 níveis) cobre os
        // poucos ciclos entre o fim e o redisparo, então nenhuma amostra é perdida.
        dma_channel_set_write_addr(g.mic_dma_chan, g.mic_buf[g.mic_cur], false);
        dma_channel_set_trans_count(g.mic_dma_chan, MIC_CHUNK, true);

        // Se a task ainda não consumiu o buffer anterior, registra overflow.
        if (g.mic_filled_buf != -1) g.mic_overflow = true;
        g.mic_filled_buf = filled;
    }
}

static void mic_dma_start(void) {
    g.mic_cur = 0;
    g.mic_filled_buf = -1;
    adc_fifo_drain();
    dma_channel_set_write_addr(g.mic_dma_chan, g.mic_buf[g.mic_cur], false);
    dma_channel_set_irq0_enabled(g.mic_dma_chan, true);
    dma_channel_set_trans_count(g.mic_dma_chan, MIC_CHUNK, true);  // dispara
    adc_run(true);
}

static void mic_dma_stop(void) {
    adc_run(false);
    // Desabilita o IRQ deste canal antes do abort: o abort pode deixar um IRQ
    // pendente que reativaria o ping-pong indevidamente.
    dma_channel_set_irq0_enabled(g.mic_dma_chan, false);
    dma_channel_abort(g.mic_dma_chan);
    dma_hw->ints0 = (1u << g.mic_dma_chan);   // limpa pendência residual
    adc_fifo_drain();
    g.mic_filled_buf = -1;
}

static void mic_task(void *params) {
    (void) params;

    // ADC em modo free-running, clock de hardware a 16 kHz, FIFO + DMA.
    adc_init();
    adc_gpio_init(PIN_MIC);
    adc_select_input(ADC_CHAN);
    adc_fifo_setup(
        true,    // habilita FIFO
        true,    // gera DREQ (pedido de DMA)
        1,       // DREQ quando >= 1 amostra na FIFO
        false,   // sem bit de erro na FIFO
        false    // mantém 12-bit (não reduz pra 8-bit)
    );
    adc_set_clkdiv(ADC_CLKDIV);   // 16 kHz exatos

    // Canal de DMA: lê da FIFO do ADC (endereço fixo) e escreve no buffer (incrementa).
    g.mic_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(g.mic_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);
    dma_channel_configure(g.mic_dma_chan, &c, g.mic_buf[0], &adc_hw->fifo, MIC_CHUNK, false);

    // IRQ de fim de transferência -> reinicia ping-pong na ISR.
    dma_channel_set_irq0_enabled(g.mic_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, mic_dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);

    bool was_recording = false;

    while (true) {
        if (g.recording) {
            if (!was_recording) {
                g.mic_overflow = false;
                mic_dma_start();
                was_recording = true;
            }

            int b = g.mic_filled_buf;
            if (b >= 0) {
                // Envio em bloco (não byte-a-byte): little-endian nativo do ARM,
                // exatamente o formato <u2 que o bridge.py espera.
                fwrite(g.mic_buf[b], sizeof(uint16_t), MIC_CHUNK, stdout);
                fflush(stdout);
                g.mic_filled_buf = -1;
            } else {
                // DMA ainda enchendo o buffer (~16 ms); cede CPU pras outras tasks.
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

// ISR do wrap do PWM — paga pelo hardware a cada ciclo da portadora (48 kHz).
// A cada SPK_DECIM ciclos puxa a próxima amostra do ring (16 kHz) e ajusta o
// duty. Sem busy_wait, sem fila no caminho crítico -> reprodução sem jitter.
// Fica na RAM (__not_in_flash_func) pra não tomar latência de XIP.
static void __not_in_flash_func(spk_pwm_isr)(void) {
    pwm_clear_irq(g.spk_slice);

    static int decim = 0;
    if (++decim < SPK_DECIM) return;
    decim = 0;

    uint16_t level = SPK_SILENCE;
    if (g.spk_head != g.spk_tail) {                   // ring não vazio
        uint16_t s = g.spk_ring[g.spk_tail & SPK_RING_MASK];
        g.spk_tail++;
        // mapeia 12-bit (0..4095) -> 0..SPK_WRAP
        level = (uint16_t)(((uint32_t) s * (SPK_WRAP + 1)) >> 12);
        if (level > SPK_WRAP) level = SPK_WRAP;
    }
    pwm_set_chan_level(g.spk_slice, g.spk_chan, level);
}

static void spk_audio_init(void) {
    gpio_set_function(PIN_SPK, GPIO_FUNC_PWM);
    g.spk_slice = pwm_gpio_to_slice_num(PIN_SPK);
    g.spk_chan  = pwm_gpio_to_channel(PIN_SPK);

    pwm_clear_irq(g.spk_slice);
    pwm_set_irq_enabled(g.spk_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, spk_pwm_isr);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // clkdiv tal que (wrap+1)*clkdiv = sysclk/48kHz. Com sysclk=150MHz -> clkdiv=1.0.
    float clkdiv = (float) clock_get_hz(clk_sys) / ((float) SPK_CARRIER_HZ * (SPK_WRAP + 1));

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clkdiv);
    pwm_config_set_wrap(&config, SPK_WRAP);
    pwm_init(g.spk_slice, &config, true);

    pwm_set_chan_level(g.spk_slice, g.spk_chan, SPK_SILENCE);
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
            g.playing = true;
            have_lsb = false;
        } else if (is_end) {
            g.playing = false;
            have_lsb = false;
            // Sem empurrar silêncio: quando o ring esvazia, a ISR já volta sozinha
            // pro nível de silêncio (SPK_SILENCE).
        } else if (g.playing) {
            // Remonta sample de 2 bytes em little-endian
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

// Mantém o TinyUSB vivo sob o FreeRTOS. O servicing por timer/IRQ do stdio_usb
// não dispara de forma confiável depois que o scheduler assume, então o tud_task()
// para e os control-transfers (SET_LINE_CODING ao abrir a porta) ficam sem resposta
// -> WinError 31 no PC. Aqui o próprio scheduler (que comprovadamente roda) chama
// stdio_flush() a cada 1 ms, e ele invoca tud_task() pelo caminho serializado
// (sob o mutex do stdio_usb), sem reentrância. Prioridade alta pra responder o
// host rápido; cede a CPU a cada tick.
static void usb_service_task(void *params) {
    (void) params;
    while (true) {
        stdio_flush();
        vTaskDelay(1);   // 1 tick = 1 ms (configTICK_RATE_HZ = 1000)
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    // Reprodução é dirigida por ISR do PWM (não por task) — inicializa aqui.
    spk_audio_init();

    xTaskCreate(usb_service_task, "usb", 1024, NULL, 4, NULL);
    xTaskCreate(button_task,      "btn", 1024, NULL, 3, NULL);
    xTaskCreate(mic_task,         "mic", 1024, NULL, 2, NULL);
    xTaskCreate(usb_rx_task,      "rx",  2048, NULL, 2, NULL);
    // Heurística de gesto do IMU: prioridade 1 (mais baixa), stack pequena (sem EI).
    xTaskCreate(imu_task,         "imu", 2048, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true) { }
}
