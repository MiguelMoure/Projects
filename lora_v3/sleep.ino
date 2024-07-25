//--------------------------Codigo de Funciones Sleep-----------------------------//
//Incluimos librerias
#include <esp_sleep.h>
//Se incluyen las funciones de sleep
void sleep_interrupt(uint8_t gpio, uint8_t mode) {
    esp_sleep_enable_ext0_wakeup((gpio_num_t) gpio, mode);
}

void sleep_interrupt_mask(uint64_t mask, uint8_t mode) {
    esp_sleep_enable_ext1_wakeup(mask, (esp_sleep_ext1_wakeup_mode_t) mode);
}
//Utilizamos la funcion Sleep millis para desactivar el microcontrolador durante 15 minutos
void sleep_millis(uint64_t ms) {
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_deep_sleep_start();
}

void sleep_seconds(uint32_t seconds) {
    esp_sleep_enable_timer_wakeup(seconds * 1000000);
    esp_deep_sleep_start();
}

void sleep_forever() {
    esp_deep_sleep_start();
}
