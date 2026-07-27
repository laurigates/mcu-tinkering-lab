/**
 * @file voice_persona.c
 * @brief Voice persona table, runtime selection, and NVS persistence.
 */

#include "voice_persona.h"

#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "esp_random.h"
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
 * line reads as caricature. Adjust the brief freely; it is data, not logic.
 *
 * Which is exactly why the period markers are NOT in `text_brief`. A brief that
 * names one construction gets that construction every single time: naming
 * "Asianlaita on oikeastaan niin, että ..." there made it the opening of
 * practically every spoken line. The markers live in the `openers` pool below
 * instead, so each one turns up on its share of utterances and the pool can
 * also say "no opener at all". Keep `text_brief` to what should hold for
 * *every* line, and put anything phrase-shaped in a pool. */

/* --- fi-1950 --------------------------------------------------------------- */

/* Directives are written in the persona's own language: a Finnish instruction
 * keeps the model inside the register while it follows the instruction, where
 * an English one tends to pull the reply toward English idiom. */
static const char *const s_fi_openers[] = {
    "Aloita suoraan asiasta, ilman mitään aloitussanaa.",
    "Aloita empivällä täytesanalla, esimerkiksi \"Jaahas\", \"Tuota noin\" tai \"No niin\".",
    "Aloita lyhyellä huudahduksella, esimerkiksi \"Kas\", \"Kappas\" tai \"Vai niin\".",
    "Aloita ajan juhlallisella rakenteella, esimerkiksi \"Asianlaita on oikeastaan niin, "
    "että ...\".",
    "Aloita puhuttelemalla kuulijaa kohteliaasti, esimerkiksi \"Hyvä herra\" tai "
    "\"Arvoisa kuulija\".",
    "Aloita kysymyksellä ja vastaa siihen itse samassa lauseessa.",
    "Aloita ajan tai paikan määreellä, esimerkiksi \"Täällä\", \"Nyt\" tai \"Tähän ehtooseen\".",
};

static const char *const s_fi_shapes[] = {
    "Pidä lause hyvin lyhyenä, korkeintaan kymmenen sanaa.",
    "Totea asia tyynesti ja vähäeleisesti.",
    "Anna lauseeseen kevyt, kuiva sävy — älä kuitenkaan vitsaile.",
    "Muotoile lause kohteliaana huomautuksena kuulijalle.",
    "Käytä yhtä vanhahtavaa sanaa (esimerkiksi \"ehtoo\", \"jokseenkin\", \"varsin\") mausteena.",
    "Sano asia suoraan ja koruttomasti, ilman kiertelyä.",
};

static const char *const s_fi_fallback_ok[] = {
    "Hyvää päivää, minä olen Robocar, ja kaikki järjestelmät ovat kunnossa.",
    "Jaahas. Robocar tässä, ja kaikki toimii moitteettomasti.",
    "Kas, Robocar valmiina. Ei valittamista, kaikki on kunnossa.",
    "No niin. Robocar raportoi: järjestelmät ovat kunnossa.",
};

static const char *const s_fi_fallback_prefix[] = {
    "Hyvää päivää, minä olen Robocar. Asianlaita on tuota niin, että nämä osat eivät vastaa:",
    "Jaahas. Robocar tässä, ja ikävä kyllä nämä osat eivät vastaa:",
    "Kas. Robocar raportoi vian, sillä nämä osat eivät vastaa:",
    "Tuota noin. Robocar tässä. Nämä osat ovat vaiti:",
};

/* --- en-default ------------------------------------------------------------ */

static const char *const s_en_openers[] = {
    "Start straight in on the observation, with no opening word at all.",
    "Open with a short filler, such as \"Well\", \"Right\" or \"So\".",
    "Open with a small exclamation, such as \"Ah\", \"Look at that\" or \"Huh\".",
    "Open by addressing the listener directly.",
    "Open with a question and answer it in the same sentence.",
    "Open with where or when you are, such as \"Over here\" or \"Right now\".",
};

static const char *const s_en_shapes[] = {
    "Keep it very short — ten words at most.",
    "Say it flatly and matter-of-factly.",
    "Give it a light, dry humour, but do not tell a joke.",
    "Phrase it as a polite remark to the listener.",
    "Say it plainly, without hedging.",
    "Let a little curiosity show.",
};

static const char *const s_en_fallback_ok[] = {
    "Hi, I'm Robocar and all my systems are online.",
    "Robocar here. Everything checks out.",
    "Well then. Robocar reporting, all systems nominal.",
    "All good on my end. Robocar, ready when you are.",
};

static const char *const s_en_fallback_prefix[] = {
    "Hi, I'm Robocar. These parts are not responding:",
    "Robocar here, with bad news. These parts are not responding:",
    "Well then. Robocar reporting a fault; these are not responding:",
    "Robocar checking in. Silent parts:",
};

static const voice_persona_t s_personas[VOICE_PERSONA_COUNT] = {
    [VOICE_PERSONA_EN_DEFAULT] =
        {
            .slug = "en-default",
            .label = "English, plain contemporary",
            .language_code = "en-US",
            .voice = "Kore",
            .tts_style = "Say in a friendly, natural tone",
            .text_brief = "Speak plain, friendly contemporary English. Vary how you phrase "
                          "things — do not reuse the same opening twice running.",
            .tag_brief = "You may mark delivery with a bracketed tag inside the sentence: "
                         "[sighs] [laughs] [whispers] [excited] [bored] [gasp]. Use them "
                         "sparingly and only where they fit what is being said — at most one "
                         "line in three, and never in a fault report. Write the tag exactly "
                         "as shown.",
            .openers = {s_en_openers, DIALOGUE_POOL_COUNT(s_en_openers)},
            .shapes = {s_en_shapes, DIALOGUE_POOL_COUNT(s_en_shapes)},
            .avoid_lead = "Do not begin the sentence with any of these:",
            .fallback_ok = {s_en_fallback_ok, DIALOGUE_POOL_COUNT(s_en_fallback_ok)},
            .fallback_prefix = {s_en_fallback_prefix, DIALOGUE_POOL_COUNT(s_en_fallback_prefix)},
            .fallback_suffix = ".",
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
            /* Naming the *era* alone produced correct Finnish that did not sound
             * period at all — the model has no reason to infer the delivery from
             * a date. What reads as "1950s" is a bundle of concrete, separable
             * traits, so they are asked for individually: theatre-derived
             * enunciation (nearly every film actor of the era came from the
             * stage), full yleiskieli forms rather than colloquial contractions,
             * a distinctly tapped /r/, held geminates, an unhurried tempo with
             * real pauses at clause boundaries, and the declamatory rise-and-fall
             * of a Yleisradio announcer.
             *
             * One caveat this directive cannot address: a good part of what makes
             * an old recording *sound* old is the recording chain, not the
             * speaking — optical/early-magnetic sound was roughly 100 Hz–5 kHz,
             * which thins the bass and pushes the midrange forward. TTS returns
             * clean full-band audio, so that character is simply absent. Closing
             * it would mean filtering the PCM on the way to I2S, not prompting
             * harder. See the skill for the sketch. */
            .tts_style = "Puhu kuin 1950-luvun suomalaisen elokuvan näyttelijä tai Yleisradion "
                         "kuuluttaja: huoliteltua yleiskieltä, jokainen tavu selvästi "
                         "artikuloituna, sorahtava ärrä, kaksoiskonsonantit pitkinä pidettyinä. "
                         "Rauhallinen, hieman verkkainen tempo ja selvät tauot lauseen osien "
                         "välissä. Juhlallinen, teatterista periytyvä lausunta — älä puhu "
                         "rennosti tai nykyaikaisen luontevasti",
            .text_brief =
                "Puhu kuin 1950-luvun suomalaisen elokuvan hahmo (vrt. komisario Palmu): "
                "kohteliasta, hieman vanhahtavaa yleiskieltä, teitittelyä ja herrasmiesmäistä "
                "sävyä. Vältä nykyslangia, anglismeja, lyhenteitä ja emojeita. Vaihtele "
                "sanontaa: älä toista samaa aloitusta tai fraasia kerrasta toiseen. "
                "Kirjoita yksi lyhyt puhuttu lause.",
            .tag_brief =
                "Voit halutessasi merkitä esitystapaa hakasulkeilla suoraan lauseen sisään: "
                "[sighs] [laughs] [whispers] [excited] [bored] [gasp]. Käytä niitä säästeliäästi "
                "ja vain silloin kun ne sopivat asiaan — korkeintaan yksi lause kolmesta, eikä "
                "koskaan vikailmoituksessa. Kirjoita tagi täsmälleen näin, englanniksi.",
            .openers = {s_fi_openers, DIALOGUE_POOL_COUNT(s_fi_openers)},
            .shapes = {s_fi_shapes, DIALOGUE_POOL_COUNT(s_fi_shapes)},
            .avoid_lead = "Älä aloita lausetta näillä sanoilla:",
            .fallback_ok = {s_fi_fallback_ok, DIALOGUE_POOL_COUNT(s_fi_fallback_ok)},
            .fallback_prefix = {s_fi_fallback_prefix, DIALOGUE_POOL_COUNT(s_fi_fallback_prefix)},
            .fallback_suffix = ".",
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

    /* Seeded here because this is where the voice subsystem comes up, ahead of
     * both consumers (the planner and self-report tasks). Without a seed the
     * draw sequence is identical after every boot, which is audible in exactly
     * the worst place: the self-introduction spoken at power-on would open the
     * same way every single time — the complaint that motivated the pools. */
    dialogue_style_seed(esp_random());

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
            /* The avoid-list holds openings in the outgoing persona's language;
             * carrying them into the new one asks the model not to start with
             * phrases it was never going to use anyway. */
            dialogue_style_reset_recent();
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
