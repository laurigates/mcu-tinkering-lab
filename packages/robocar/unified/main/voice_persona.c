/**
 * @file voice_persona.c
 * @brief Voice persona table, runtime selection, and NVS persistence.
 */

#include "voice_persona.h"

#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "voice_persona";

#define NVS_NAMESPACE "voice"
#define NVS_KEY_PERSONA "persona"
#define NVS_KEY_VOICE "voice"

/* -------------------------------------------------------------------------- */
/* Table                                                                       */
/* -------------------------------------------------------------------------- */

/* The Finnish brief is a *register* brief, not a script: the model composes each
 * line, so the robot still comments on what it actually sees. The period markers
 * below are grounded in 1950s-60s Finnish film dialogue rather than invented —
 * chiefly the Komisario Palmu films (Mika Waltari / Matti Kassila), whose
 * hallmarks are consistent teitittely, "herra/rouva/neiti" address, the
 * sententious formal construction "Asianlaita on oikeastaan niin, että ...",
 * and the hesitation opener "Jaah" / "Jaahas".
 *
 * "ehtoo" (archaic/eastern-dialect *evening*) and the filler "tuota" come from
 * the examples this feature was requested with. Note they are dialectal/archaic
 * rather than standard 1950s written Finnish, so they are offered to the model
 * as flavour it *may* use, not as mandatory openers — forcing them into every
 * line reads as caricature. Adjust the brief freely; it is data, not logic. */
static const voice_persona_t s_personas[VOICE_PERSONA_COUNT] = {
    [VOICE_PERSONA_EN_DEFAULT] =
        {
            .slug = "en-default",
            .label = "English, plain contemporary",
            .language_code = "en-US",
            .voice = "Kore",
            .tts_style = "Say in a friendly, natural tone",
            .text_brief = "Speak plain, friendly contemporary English.",
            .fallback_ok = "Hi, I'm Robocar and all my systems are online.",
            .fallback_fmt = "Hi, I'm Robocar. These parts are not responding:%s.",
            .fault_wifi = " WiFi",
            .fault_camera = " camera",
            .fault_motors = " motors",
            .fault_audio = " audio",
        },
    [VOICE_PERSONA_FI_1950] =
        {
            .slug = "fi-1950",
            .label = "Finnish, 1950s film register",
            .language_code = "fi-FI",
            .voice = "Charon",
            .tts_style = "Puhu rauhallisesti, kohteliaasti ja hieman juhlallisesti, "
                         "1950-luvun suomalaisen elokuvan ja radiokuuluttajan tapaan",
            .text_brief =
                "Puhu kuin 1950-luvun suomalaisen elokuvan hahmo (vrt. komisario Palmu): "
                "kohteliasta, hieman vanhahtavaa yleiskieltä, teitittelyä ja herrasmiesmäistä "
                "sävyä. Saat käyttää ajan täytesanoja ja empimistä, kuten \"tuota\", \"jaahas\" "
                "ja \"no niin\", sekä vanhahtavia sanoja kuten \"ehtoo\" (ilta) — mausteena, et "
                "joka lauseessa. Ajan tapaan muotoiltu rakenne, esimerkiksi \"Asianlaita on "
                "oikeastaan niin, että ...\", sopii hyvin. Vältä nykyslangia, anglismeja, "
                "lyhenteitä ja emojeita. Kirjoita yksi lyhyt puhuttu lause.",
            .fallback_ok = "Hyvää päivää, minä olen Robocar, ja kaikki järjestelmät ovat kunnossa.",
            .fallback_fmt = "Hyvää päivää, minä olen Robocar. Asianlaita on tuota niin, "
                            "että nämä osat eivät vastaa:%s.",
            .fault_wifi = " verkkoyhteys",
            .fault_camera = " kamera",
            .fault_motors = " moottorit",
            .fault_audio = " ääni",
        },
};

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

/* Single-word selection: readers (planner, TTS, self-report tasks) never lock,
 * and a switch lands atomically. */
static volatile voice_persona_id_t s_current = VOICE_PERSONA_DEFAULT;

/* The override is mutable storage rather than a literal, so it needs a lock —
 * a reader must not copy a half-written name into a request body. */
static char s_voice_override[VOICE_PERSONA_VOICE_MAX];
static SemaphoreHandle_t s_lock;

static esp_err_t nvs_store(const char *key, const char *value)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }
    err = (value && value[0] != '\0') ? nvs_set_str(h, key, value) : nvs_erase_key(h, key);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK; /* erasing an absent key is not a failure */
    }
    nvs_close(h);
    return err;
}

/* -------------------------------------------------------------------------- */
/* API                                                                         */
/* -------------------------------------------------------------------------- */

void voice_persona_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no stored persona — using default '%s'", s_personas[s_current].slug);
        return;
    }

    char slug[24] = {0};
    size_t len = sizeof(slug);
    if (nvs_get_str(h, NVS_KEY_PERSONA, slug, &len) == ESP_OK) {
        for (size_t i = 0; i < VOICE_PERSONA_COUNT; ++i) {
            if (strcmp(s_personas[i].slug, slug) == 0) {
                s_current = (voice_persona_id_t)i;
                break;
            }
        }
    }

    len = sizeof(s_voice_override);
    if (nvs_get_str(h, NVS_KEY_VOICE, s_voice_override, &len) != ESP_OK) {
        s_voice_override[0] = '\0';
    }
    nvs_close(h);

    ESP_LOGI(TAG, "persona '%s' (%s), voice %s", s_personas[s_current].slug,
             s_personas[s_current].language_code,
             s_voice_override[0] ? s_voice_override : s_personas[s_current].voice);
}

const voice_persona_t *voice_persona_get(void)
{
    voice_persona_id_t id = s_current;
    if (id >= VOICE_PERSONA_COUNT) {
        id = VOICE_PERSONA_EN_DEFAULT;
    }
    return &s_personas[id];
}

void voice_persona_effective_voice(char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return;
    }
    const voice_persona_t *p = voice_persona_get();

    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        strlcpy(out, s_voice_override[0] ? s_voice_override : p->voice, out_len);
        xSemaphoreGive(s_lock);
    } else {
        /* Lock unavailable: the persona default is a literal and always safe. */
        strlcpy(out, p->voice, out_len);
    }
}

esp_err_t voice_persona_set(const char *slug, bool persist)
{
    if (!slug) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < VOICE_PERSONA_COUNT; ++i) {
        if (strcmp(s_personas[i].slug, slug) == 0) {
            s_current = (voice_persona_id_t)i;
            ESP_LOGI(TAG, "persona -> '%s' (%s)", s_personas[i].slug, s_personas[i].language_code);
            return persist ? nvs_store(NVS_KEY_PERSONA, slug) : ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t voice_persona_set_voice(const char *voice, bool persist)
{
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (voice && voice[0] != '\0') {
            strlcpy(s_voice_override, voice, sizeof(s_voice_override));
        } else {
            s_voice_override[0] = '\0';
        }
        xSemaphoreGive(s_lock);
    } else {
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "voice -> %s", (voice && voice[0]) ? voice : "(persona default)");
    return persist ? nvs_store(NVS_KEY_VOICE, voice) : ESP_OK;
}

const voice_persona_t *voice_persona_at(size_t index)
{
    return (index < VOICE_PERSONA_COUNT) ? &s_personas[index] : NULL;
}
