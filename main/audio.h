#ifndef AUDIO_H
#define AUDIO_H

#include <stdbool.h>

// Driver de áudio (captura por mic + reprodução no speaker) e push-to-talk.
// O estado de baixo nível (buffers do DMA, ring do PWM, flags de ISR) fica
// encapsulado dentro de audio.c — fora do main.c.

// Inicializa a reprodução do speaker (PWM + ISR). Chamar antes do scheduler.
void spk_audio_init(void);

// Tasks do FreeRTOS (criadas pelo main()).
void button_task(void *params);
void mic_task(void *params);
void usb_rx_task(void *params);
void usb_service_task(void *params);

// true enquanto está gravando ou tocando áudio (usado pela gesture.cpp).
bool audio_is_busy(void);

#endif // AUDIO_H
