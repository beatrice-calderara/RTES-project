#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "pico/stdio_usb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

#define SAMPLE_RATE     16000
#define BUFFER_SIZE     256
#define ADC_PIN         26
#define LED_PIN         25          // LED onboard
#define BUZZER_PIN      15          // GPIO for buzzer (PWM)

// Upper 4 bits of the 16bit usb package
#define MARKER_START    0xF000
#define MARKER_END      0xE000
#define MARKER_NORMAL   0x0000

// Task priority (higher = higher priority)
#define PRIORITY_ALARM  4
#define PRIORITY_BLINK  3
#define PRIORITY_OFF    2
#define PRIORITY_ON     1
#define PRIORITY_RECEIVER 1
#define PRIORITY_AUDIO  5       // Audio sampling is given the max priority


// Ping-pong buffers for producer-consumer
#define NUM_BUFFERS 2
uint16_t audio_buffers[NUM_BUFFERS][BUFFER_SIZE];
volatile int active_buffer_index = 0;
volatile uint16_t buffer_index = 0;

QueueHandle_t xAudioBufferQueue = NULL;

// Mutex used for shared resources
SemaphoreHandle_t xLedMutex = NULL;
SemaphoreHandle_t xBuzzerMutex = NULL;

// Task handles
TaskHandle_t xOnTaskHandle = NULL;
TaskHandle_t xOffTaskHandle = NULL;
TaskHandle_t xBlinkTaskHandle = NULL;
TaskHandle_t xAlarmTaskHandle = NULL;

// Current states
volatile bool led_state = false;
volatile bool buzzer_state = false;

// Buffer for serial reception
#define CMD_BUFFER_SIZE 32
char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_index = 0;



bool adc_sample_callback(struct repeating_timer *t) {
    // Read an ADC sample and store it in the active buffer (only if we haven't reached the buffer size yet)
    if (buffer_index < BUFFER_SIZE) {
        audio_buffers[active_buffer_index][buffer_index++] = adc_read();
    }

    // If the active buffer is full, perform the buffer swap
    if (buffer_index >= BUFFER_SIZE) {
        // Pointer to the buffer we have just finished filling
        uint16_t* p_full_buffer = audio_buffers[active_buffer_index];

        // Send the POINTER to the queue, notifying the task which buffer is ready to be processed
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendToBackFromISR(xAudioBufferQueue, &p_full_buffer, &xHigherPriorityTaskWoken);

        // Switch the active buffer for the next sampling cycle
        active_buffer_index = 1 - active_buffer_index;      // Toggles between 0 and 1

        // Reset the sample index to start filling the new buffer
        buffer_index = 0;

        // Request a context switch if a higher-priority task was woken
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    return true;
}


// Send sample with marker
static inline void send_packed_sample(uint16_t adc_value, uint16_t marker) {
    // Keep only the lower 12 bits of the ADC value
    adc_value &= 0x0FFF;

    // Combine marker and ADC value into a single 16-bit packet
    uint16_t packed = marker | adc_value;

    // Send the packed value byte by byte (little-endian)
    putchar_raw(packed & 0xFF);         // Send low byte
    putchar_raw((packed >> 8) & 0xFF);  // Send high byte
}

// Configure PWM for buzzer
void setup_buzzer(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);         // Set the selected GPIO pin to PWM mode
    uint slice_num = pwm_gpio_to_slice_num(gpio);   // Get the PWM slice associated with this GPIO
    pwm_config config = pwm_get_default_config();   // Load default PWM configuration
    pwm_config_set_clkdiv(&config, 4.0f);           // Set PWM clock divider to reduce frequency
    pwm_init(slice_num, &config, true);             // Initialize and start the PWM slice
    pwm_set_gpio_level(gpio, 0);                    // Set initial duty cycle to 0 (buzzer off)
}

void buzzer_on(uint gpio, uint16_t freq) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);   // Get PWM slice for this GPIO
    uint32_t clock = 125000000 / 4;                 // Effective PWM clock after divider
    uint32_t wrap = clock / freq;                   // Compute wrap value to generate desired frequency
    pwm_set_wrap(slice_num, wrap);                  // Set the PWM period
    pwm_set_gpio_level(gpio, wrap / 2);             // Set duty cycle to 50% (sound on)
}

void buzzer_off(uint gpio) {
    pwm_set_gpio_level(gpio, 0);                    // Set duty cycle to 0 (disable buzzer output)
}

// ========== TASK ON ==========
void vTaskOn(void *pvParameters) {
    (void) pvParameters;

    while (1) {
        // Waits to be notified to start
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Attempts to take the semaphore
        if (xSemaphoreTake(xLedMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // If the LED is already ON, do nothing and release the semaphore
            if (led_state == false) {
                gpio_put(LED_PIN, 1);
                led_state = true;
            }
            // Release the semaphore
            xSemaphoreGive(xLedMutex);
        }
    }
}

// ========== TASK OFF ==========
void vTaskOff(void *pvParameters) {
    (void) pvParameters;

    while (1) {
        // Waits to be notified to start
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Attempts to take the semaphore
        if (xSemaphoreTake(xLedMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // If the LED is already OFF, do nothing and release the semaphore
            if (led_state == true) {
                gpio_put(LED_PIN, 0);
                led_state = false;
            }
            // Release the semaphore
            xSemaphoreGive(xLedMutex);
        }

        // If the buzzer is active we want to turn it off
        /*
        if (xSemaphoreTake(xBuzzerMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (buzzer_state == true) {
                buzzer_off(BUZZER_PIN);
                buzzer_state = false;
            }
            xSemaphoreGive(xBuzzerMutex);
        }
        */
    }
}


// ========== TASK BLINK ==========
void vTaskBlink(void *pvParameters) {
    (void) pvParameters;

    while (1) {
        // Waits to be notified to start
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Loop for 10 seconds (50 cycles with 200ms)
        for (int i = 0; i < 50; i++) {

            // Check if an higher priority task (ALARM) sent a notification. If yes, it breaks the blink.
            if (ulTaskNotifyTake(pdTRUE, 0) != 0) {
                break;
            }

            // Attempts to take the LED semaphore
            if (xSemaphoreTake(xLedMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                // Switch the LED status
                led_state = !led_state;
                gpio_put(LED_PIN, led_state);

                // Release the semaphore
                xSemaphoreGive(xLedMutex);
            }

            // Waits for the next blinking cycle
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // Check if the LED is OFF
        if (xSemaphoreTake(xLedMutex, portMAX_DELAY) == pdTRUE) {
            gpio_put(LED_PIN, 0);
            led_state = false;
            xSemaphoreGive(xLedMutex);
        }
    }
}


// ========== TASK ALARM  ==========
void vTaskAlarm(void *pvParameters) {
    (void) pvParameters;

    while (1) {
        // Waits to be notified to start
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Take control of both resources.
        if (xSemaphoreTake(xLedMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (xSemaphoreTake(xBuzzerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {

                // Loop for 15 seconds (150 cycles with 100ms)
                for (int i = 0; i < 150; i++) {
                    // LED: quick blink
                    led_state = !led_state;
                    gpio_put(LED_PIN, led_state);

                    // Buzzer: intermittent sound
                    if (i % 4 < 2) {                    // Sound for 200ms, then silence for 200ms
                        if (!buzzer_state) {
                           buzzer_on(BUZZER_PIN, 1500); // Alarm frequency
                           buzzer_state = true;
                        }
                    } else {
                        if (buzzer_state) {
                           buzzer_off(BUZZER_PIN);
                           buzzer_state = false;
                        }
                    }

                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                // Turn off the buzzer and release the resource
                buzzer_off(BUZZER_PIN);
                buzzer_state = false;
                xSemaphoreGive(xBuzzerMutex);

                // Turn off the LED and release the resource
                gpio_put(LED_PIN, 0);
                led_state = false;
                xSemaphoreGive(xLedMutex);

            } else {
                // It didn't manage to take the buzzer, it release the LED and try again
                xSemaphoreGive(xLedMutex);
            }
        }
    }
}

// ========== TASK AUDIO STREAMING ==========
void vTaskAudioStream(void *pvParameters) {
    (void) pvParameters;
    uint16_t* p_buffer_to_send;

    while (1) {
        // Wait indefinitely until a full audio buffer is received from the queue.
        // Once available, store the pointer in p_buffer_to_send.
        if (xQueueReceive(xAudioBufferQueue, &p_buffer_to_send, portMAX_DELAY) == pdTRUE) {

            // Transmit the first audio sample in the buffer (index 0).
            // This one is tagged with MARKER_START to indicate the beginning of a packet.
            send_packed_sample(p_buffer_to_send[0], MARKER_START);

            // Transmit all intermediate samples.
            // Loop runs from index 1 up to BUFFER_SIZE - 2.
            // These samples are tagged with MARKER_NORMAL to indicate they are part of the ongoing packet.
            for (int i = 1; i < BUFFER_SIZE - 1; i++) {
                send_packed_sample(p_buffer_to_send[i], MARKER_NORMAL);
            }

            // Transmit the last audio sample in the buffer (index BUFFER_SIZE -1).
            // This one is tagged with MARKER_END to indicate the end of the packet.
            send_packed_sample(p_buffer_to_send[BUFFER_SIZE - 1], MARKER_END);

            // Ensure that all buffered output is flushed to the output stream.
            stdio_flush();
        }
    }
}


// ========== TASK SERIAL COMMAND RECEIVER ==========
void vTaskSerialReceiver(void *pvParameters) {
    (void) pvParameters;

    while (1) {
        // Attempt to read one character from the serial interface.
        // Timeout is 1000 microseconds; if no data arrives, PICO_ERROR_TIMEOUT is returned.
        int c = getchar_timeout_us(1000);

        // Check if the received character marks the end of a command (newline or carriage return).
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\n' || c == '\r') {
                // Terminate the command string.
                cmd_buffer[cmd_index] = '\0';

                // Check if the command starts with the expected prefix.
                if (strncmp(cmd_buffer, "CMD:", 4) == 0) {
                    // Extract the actual command after "CMD:"
                    char *command = cmd_buffer + 4;

                    // Match known commands and notify the corresponding task.
                    if (strcmp(command, "on") == 0) {
                        xTaskNotifyGive(xOnTaskHandle);
                    }
                    else if (strcmp(command, "off") == 0) {
                        xTaskNotifyGive(xOffTaskHandle);
                    }
                    else if (strcmp(command, "blink") == 0) {
                        xTaskNotifyGive(xBlinkTaskHandle);
                    }
                    else if (strcmp(command, "alarm") == 0) {
                        xTaskNotifyGive(xAlarmTaskHandle);
                    }
                }

                // Reset buffer index for the next command.
                cmd_index = 0;
            }
            // If the character is not a terminator, store it in the command buffer
            // as long as there is still space available.
            else if (cmd_index < CMD_BUFFER_SIZE - 1) {
                cmd_buffer[cmd_index++] = c;
            }
        }

        // Add a small delay to allow other RTOS tasks to run and avoid busy-looping.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ========== MAIN ==========
int main() {
    // Init hardware
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    setup_buzzer(BUZZER_PIN);

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    sleep_ms(2000);

    // Start timer ADC
    struct repeating_timer timer;
    add_repeating_timer_us(-1000000 / SAMPLE_RATE, adc_sample_callback, NULL, &timer);

    // Create mutexes
    xLedMutex = xSemaphoreCreateMutex();
    xBuzzerMutex = xSemaphoreCreateMutex();

    // Queue with NUM buffers pointers to buffer
    xAudioBufferQueue = xQueueCreate(NUM_BUFFERS, sizeof(uint16_t*));

    // Create tasks
    xTaskCreate(vTaskOn, "TaskON", 256, NULL, PRIORITY_ON, &xOnTaskHandle);
    xTaskCreate(vTaskOff, "TaskOFF", 256, NULL, PRIORITY_OFF, &xOffTaskHandle);
    xTaskCreate(vTaskBlink, "TaskBLINK", 256, NULL, PRIORITY_BLINK, &xBlinkTaskHandle);
    xTaskCreate(vTaskAlarm, "TaskALARM", 512, NULL, PRIORITY_ALARM, &xAlarmTaskHandle);
    xTaskCreate(vTaskAudioStream, "TaskAUDIO", 512, NULL, PRIORITY_AUDIO, NULL);
    xTaskCreate(vTaskSerialReceiver, "TaskSERIAL", 256, NULL, PRIORITY_RECEIVER, NULL);

    // Start scheduler
    vTaskStartScheduler();


    while (1) {
        tight_loop_contents();
    }
}
