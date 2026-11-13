#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es7210_adc.h"
#include "esp_heap_caps.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "model_path.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "driver/ledc.h"
#include "esp_timer.h"


static const char *TAG = "audio_test";

static void motor_execute(int cmd);

#define PIN_SDA     1
#define PIN_SCL     2
#define PIN_MCLK    38
#define PIN_BCLK    14
#define PIN_WS      13
#define PIN_DOUT    45   // ESP32 -> DIN del codec (bocina)
#define PIN_DIN     12   // DOUT del codec -> ESP32 (micrófono)
#define PIN_PA_EN   48

#define PIN_LED     17
#define LISTEN_DELAY_US  500000   // 0.5 s antes de empezar a escuchar el comando


#define SAMPLE_RATE 16000

/* Comandos de voz que reconoce el modelo */
enum { CMD_FORWARD = 1, CMD_BACK, CMD_LEFT, CMD_RIGHT, CMD_STOP };

static i2c_master_bus_handle_t i2c_bus;

/* Escanea el bus I2C para confirmar que el codec responde */
static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Escaneando I2C...");
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(i2c_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  Dispositivo en 0x%02X", addr);
            found++;
        }
    }
    if (!found) {
        ESP_LOGE(TAG, "No se detectó nada. Revisa SDA/SCL y alimentación.");
    }
}

static esp_codec_dev_handle_t audio_init(void)
{
    /* I2C */
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));
    i2c_scan();

    /* I2S full-duplex, ESP32 como maestro */
    /* I2S solo RX, ESP32 como maestro: genera MCLK, BCLK y WS */
    i2s_chan_handle_t rx_chan;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = PIN_MCLK, .bclk = PIN_BCLK, .ws = PIN_WS,
            .dout = I2S_GPIO_UNUSED, .din = PIN_DIN,
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));

    audio_codec_i2s_cfg_t i2s_if_cfg = {
        .port = I2S_NUM_0, .rx_handle = rx_chan, .tx_handle = NULL,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_if_cfg);

    audio_codec_i2c_cfg_t i2c_if_cfg = {
        .port = I2C_NUM_0,
        .addr = 0x82,
        .bus_handle = i2c_bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_if_cfg);

    es7210_codec_cfg_t es_cfg = {
        .ctrl_if = ctrl_if,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2,
    };
    const audio_codec_if_t *codec_if = es7210_codec_new(&es_cfg);
    if (!codec_if) {
        ESP_LOGE(TAG, "No se pudo inicializar el ES7210");
        abort();
    }

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = codec_if,
        .data_if  = data_if,
    };
    esp_codec_dev_handle_t dev = esp_codec_dev_new(&dev_cfg);

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 2,
        .sample_rate = SAMPLE_RATE,
    };
    int ret = esp_codec_dev_open(dev, &fs);
    if (ret != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "esp_codec_dev_open falló: %d", ret);
        abort();
    }
    esp_codec_dev_set_in_gain(dev, 36.0);   // dB, ajústalo si hace falta
    return dev;
}

static const esp_afe_sr_iface_t *afe;
static esp_afe_sr_data_t *afe_data;
static esp_codec_dev_handle_t codec;

/* Tarea 1: lee del micrófono y alimenta al AFE */
static void feed_task(void *arg)
{
    int chunk = afe->get_feed_chunksize(afe_data);
    int nch   = afe->get_feed_channel_num(afe_data);
    size_t bytes = chunk * nch * sizeof(int16_t);
    int16_t *buf = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL);

    while (1) {
        if (esp_codec_dev_read(codec, buf, bytes) == ESP_CODEC_DEV_OK) {
            afe->feed(afe_data, buf);
            static int cnt = 0;
            if (++cnt % 20 == 0) {
                int64_t sq = 0; int peak = 0;
                int n = chunk * nch;
                for (int i = 0; i < n; i++) {
                    int v = buf[i];
                    sq += (int64_t)v * v;
                    if (abs(v) > peak) peak = abs(v);
                }
                printf("RMS:%5d  PICO:%5d\n", (int)sqrt((double)sq / n), peak);
            }
        }
    }
}

/* Tarea 2: toma los resultados del AFE y busca la wake word */
static srmodel_list_t *models;

static void detect_task(void *arg)
{
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    if (!mn_name) {
        ESP_LOGE(TAG, "No hay modelo MultiNet en inglés flasheado");
        vTaskDelete(NULL);
    }
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *mn_data = multinet->create(mn_name, 6000);  // 6 s de ventana
    ESP_LOGI(TAG, "MultiNet: %s", mn_name);


    // Comandos de voz

    esp_mn_commands_alloc(multinet, mn_data);
    esp_mn_commands_clear();
    esp_mn_commands_add(CMD_FORWARD, "FORWARD");
    esp_mn_commands_add(CMD_BACK,    "GO BACK");
    esp_mn_commands_add(CMD_LEFT,    "GO LEFT");
    esp_mn_commands_add(CMD_RIGHT,   "RIGHT");
    esp_mn_commands_add(CMD_STOP,    "STOP");
    esp_mn_commands_update();

    bool listening = false;
    int64_t listen_from_us = 0;

    while (1) {
        afe_fetch_result_t *res = afe->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL) continue;

        if (res->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, ">>> WAKE WORD: di un comando <<<");
            gpio_set_level(PIN_LED, 1);                 // feedback
            multinet->clean(mn_data);
            afe->disable_wakenet(afe_data);
            listening = true;
            listen_from_us = esp_timer_get_time() + LISTEN_DELAY_US;
        }

        if (listening) {
            /* Durante la pausa se descarta el audio (sigue el fetch para no llenar el buffer) */
            if (esp_timer_get_time() < listen_from_us) continue;

            esp_mn_state_t st = multinet->detect(mn_data, res->data);
            if (st == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *r = multinet->get_results(mn_data);
                ESP_LOGI(TAG, "Comando id=%d  texto='%s'  prob=%.2f",
                         r->command_id[0], r->string, r->prob[0]);
                motor_execute(r->command_id[0]);
                gpio_set_level(PIN_LED, 0);
                afe->enable_wakenet(afe_data);
                listening = false;
            } else if (st == ESP_MN_STATE_TIMEOUT) {
                ESP_LOGI(TAG, "Timeout, vuelvo a esperar 'Hi ESP'");
                gpio_set_level(PIN_LED, 0);
                afe->enable_wakenet(afe_data);
                listening = false;
            }
        }
    }
}

/* 
-------------------------------- MOTORES --------------------------------
*/

#define ENA 4
#define IN1 5
#define IN2 6
#define IN3 7
#define IN4 15
#define ENB 16

static void motor_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<IN1)|(1ULL<<IN2)|(1ULL<<IN3)|(1ULL<<IN4),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT, .freq_hz = 1000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);

    int pins[2] = { ENA, ENB };
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t c = {
            .gpio_num = pins[i], .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = i, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
        };
        ledc_channel_config(&c);
    }
}

static void drive(int a1, int a2, int b1, int b2, int duty)
{
    gpio_set_level(IN1, a1); gpio_set_level(IN2, a2);
    gpio_set_level(IN3, b1); gpio_set_level(IN4, b2);
    for (int ch = 0; ch < 2; ch++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    }
}

/* TEST DE MOTORES
static void motor_test(void)
{
    ESP_LOGI(TAG, "Motores: adelante");  drive(1,0, 1,0, 200); vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "Motores: alto");      drive(0,0, 0,0, 0);   vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Motores: atrás");     drive(0,1, 0,1, 200); vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "Motores: alto");      drive(0,0, 0,0, 0);   vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Motores: izquierda"); drive(0,1, 1,0, 180); vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Motores: alto");      drive(0,0, 0,0, 0);   vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Motores: derecha");   drive(1,0, 0,1, 180); vTaskDelay(pdMS_TO_TICKS(1000));
    drive(0,0, 0,0, 0);
}
*/

#define MOTOR_TIMEOUT_MS  3000   // avance/retroceso: seguridad
#define TURN_TIME_MS      300    // giros: duración fija

#define SPEED_MOVE 180
#define SPEED_TURN 180


static esp_timer_handle_t motor_timer;

static void led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_LED, 0);
}

static void motor_timeout_cb(void *arg)
{
    ESP_LOGI(TAG, "Fin de movimiento: alto");
    drive(0,0, 0,0, 0);
}

static void motor_safety_init(void)
{
    const esp_timer_create_args_t args = {
        .callback = motor_timeout_cb,
        .name = "motor_safety",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &motor_timer));
}

static void motor_execute(int cmd)
{
    esp_timer_stop(motor_timer);   // cancela el temporizador anterior (si no corría, se ignora el error)

    int time_ms;
    switch (cmd) {
        case CMD_FORWARD: drive(0,1, 1,0, SPEED_MOVE); time_ms = MOTOR_TIMEOUT_MS; break;
        case CMD_BACK:    drive(1,0, 0,1, SPEED_MOVE); time_ms = MOTOR_TIMEOUT_MS; break;
        case CMD_LEFT:    drive(1,0, 1,0, SPEED_TURN); time_ms = TURN_TIME_MS;     break;
        case CMD_RIGHT:   drive(0,1, 0,1, SPEED_TURN); time_ms = TURN_TIME_MS;     break;
        case CMD_STOP:    drive(0,0, 0,0, 0);          return;
        default:          return;
    }
    esp_timer_start_once(motor_timer, (uint64_t)time_ms * 1000);
}

void app_main(void)
{
    //test de motores
    motor_init();
    //motor_test();
    led_init();
    motor_safety_init();

    codec = audio_init();

    models = esp_srmodel_init("model");
    if (!models) {
        ESP_LOGE(TAG, "No se pudieron cargar modelos. Revisa la partición 'model'.");
        return;
    }

    afe_config_t *afe_cfg = afe_config_init("MM", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    afe = esp_afe_handle_from_config(afe_cfg);
    afe_data = afe->create_from_config(afe_cfg);

    ESP_LOGI(TAG, "AFE listo: chunk=%d, canales=%d",
             afe->get_feed_chunksize(afe_data), afe->get_feed_channel_num(afe_data));
    ESP_LOGI(TAG, "Di: \"Hi ESP\"");

    xTaskCreatePinnedToCore(feed_task,   "feed",   8 * 1024, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(detect_task, "detect", 8 * 1024, NULL, 5, NULL, 1);
}