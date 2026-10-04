/**
 * @file nvs_shim.c — in-memory NVS behind include/nvs.h.
 *
 * Fixed-size tables, no allocation. Writes land in the store immediately; the
 * real nvs_commit() flushes a write cache, which no test here can observe, so
 * it only validates the handle. Sizes are generous for the two namespaces
 * credentials_loader.c uses and abort when exceeded, so a test that outgrows
 * them fails loudly rather than losing a key.
 */

#include "nvs_shim.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SHIM_MAX_ENTRIES 16
#define SHIM_MAX_HANDLES 8
#define SHIM_NAME_LEN 16 /* NVS_KEY_NAME_MAX_SIZE: 15 characters + NUL */
#define SHIM_VALUE_LEN 128

typedef struct {
    bool used;
    char ns[SHIM_NAME_LEN];
    char key[SHIM_NAME_LEN]; /* "" marks a namespace that exists with no keys yet */
    char value[SHIM_VALUE_LEN];
} shim_entry_t;

typedef struct {
    bool open;
    nvs_open_mode_t mode;
    char ns[SHIM_NAME_LEN];
} shim_handle_t;

static shim_entry_t s_entries[SHIM_MAX_ENTRIES];
static shim_handle_t s_handles[SHIM_MAX_HANDLES]; /* handle value = index + 1 */
static esp_err_t s_open_result = ESP_OK;
static int s_write_count;

static void copy_name(char *dest, const char *src)
{
    if (strlen(src) >= SHIM_NAME_LEN) {
        fprintf(stderr, "nvs_shim: name '%s' exceeds the NVS 15-character limit\n", src);
        abort();
    }
    strcpy(dest, src);
}

static bool namespace_exists(const char *ns)
{
    for (int i = 0; i < SHIM_MAX_ENTRIES; i++) {
        if (s_entries[i].used && strcmp(s_entries[i].ns, ns) == 0) {
            return true;
        }
    }
    return false;
}

static shim_entry_t *find(const char *ns, const char *key)
{
    for (int i = 0; i < SHIM_MAX_ENTRIES; i++) {
        if (s_entries[i].used && strcmp(s_entries[i].ns, ns) == 0 &&
            strcmp(s_entries[i].key, key) == 0) {
            return &s_entries[i];
        }
    }
    return NULL;
}

static shim_entry_t *find_or_add(const char *ns, const char *key)
{
    shim_entry_t *e = find(ns, key);
    if (e) {
        return e;
    }
    for (int i = 0; i < SHIM_MAX_ENTRIES; i++) {
        if (!s_entries[i].used) {
            s_entries[i].used = true;
            copy_name(s_entries[i].ns, ns);
            copy_name(s_entries[i].key, key);
            s_entries[i].value[0] = '\0';
            return &s_entries[i];
        }
    }
    fprintf(stderr, "nvs_shim: store full\n");
    abort();
}

static void store(const char *ns, const char *key, const char *value)
{
    if (strlen(value) >= SHIM_VALUE_LEN) {
        fprintf(stderr, "nvs_shim: value for '%s' exceeds the shim's %d bytes\n", key,
                SHIM_VALUE_LEN);
        abort();
    }
    strcpy(find_or_add(ns, key)->value, value);
}

static shim_handle_t *lookup(nvs_handle_t handle)
{
    if (handle == 0 || handle > SHIM_MAX_HANDLES || !s_handles[handle - 1].open) {
        return NULL;
    }
    return &s_handles[handle - 1];
}

/* ---- include/nvs.h ------------------------------------------------------ */

esp_err_t nvs_open(const char *namespace_name, nvs_open_mode_t open_mode, nvs_handle_t *out_handle)
{
    if (!namespace_name || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_open_result != ESP_OK) {
        return s_open_result;
    }
    if (open_mode == NVS_READONLY && !namespace_exists(namespace_name)) {
        return ESP_ERR_NVS_NOT_FOUND;
    }
    for (int i = 0; i < SHIM_MAX_HANDLES; i++) {
        if (!s_handles[i].open) {
            if (open_mode == NVS_READWRITE) {
                find_or_add(namespace_name, ""); /* READWRITE creates the namespace */
            }
            s_handles[i].open = true;
            s_handles[i].mode = open_mode;
            copy_name(s_handles[i].ns, namespace_name);
            *out_handle = (nvs_handle_t)(i + 1);
            return ESP_OK;
        }
    }
    fprintf(stderr, "nvs_shim: out of handles (one is being leaked?)\n");
    abort();
}

esp_err_t nvs_get_str(nvs_handle_t handle, const char *key, char *out_value, size_t *length)
{
    const shim_handle_t *h = lookup(handle);
    if (!h) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    if (!key || key[0] == '\0' || !length) {
        return ESP_ERR_INVALID_ARG;
    }
    const shim_entry_t *e = find(h->ns, key);
    if (!e) {
        return ESP_ERR_NVS_NOT_FOUND;
    }
    const size_t needed = strlen(e->value) + 1;
    if (!out_value) {
        *length = needed;
        return ESP_OK;
    }
    if (*length < needed) {
        return ESP_ERR_NVS_INVALID_LENGTH;
    }
    memcpy(out_value, e->value, needed);
    *length = needed;
    return ESP_OK;
}

esp_err_t nvs_set_str(nvs_handle_t handle, const char *key, const char *value)
{
    const shim_handle_t *h = lookup(handle);
    if (!h) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    if (h->mode == NVS_READONLY) {
        return ESP_ERR_NVS_READ_ONLY;
    }
    if (!key || key[0] == '\0' || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    s_write_count++;
    store(h->ns, key, value);
    return ESP_OK;
}

esp_err_t nvs_erase_all(nvs_handle_t handle)
{
    const shim_handle_t *h = lookup(handle);
    if (!h) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }
    if (h->mode == NVS_READONLY) {
        return ESP_ERR_NVS_READ_ONLY;
    }
    s_write_count++;
    for (int i = 0; i < SHIM_MAX_ENTRIES; i++) {
        if (s_entries[i].used && strcmp(s_entries[i].ns, h->ns) == 0 &&
            s_entries[i].key[0] != '\0') {
            s_entries[i].used = false;
        }
    }
    return ESP_OK;
}

esp_err_t nvs_commit(nvs_handle_t handle)
{
    return lookup(handle) ? ESP_OK : ESP_ERR_NVS_INVALID_HANDLE;
}

void nvs_close(nvs_handle_t handle)
{
    shim_handle_t *h = lookup(handle);
    if (h) {
        h->open = false;
    }
}

/* ---- nvs_shim.h --------------------------------------------------------- */

void nvs_shim_reset(void)
{
    memset(s_entries, 0, sizeof(s_entries));
    memset(s_handles, 0, sizeof(s_handles));
    s_open_result = ESP_OK;
    s_write_count = 0;
}

void nvs_shim_put(const char *ns, const char *key, const char *value)
{
    store(ns, key, value);
}

const char *nvs_shim_get(const char *ns, const char *key)
{
    if (!key || key[0] == '\0') {
        return NULL;
    }
    const shim_entry_t *e = find(ns, key);
    return e ? e->value : NULL;
}

void nvs_shim_fail_open(esp_err_t err)
{
    s_open_result = err;
}

int nvs_shim_write_count(void)
{
    return s_write_count;
}

int nvs_shim_open_handles(void)
{
    int n = 0;
    for (int i = 0; i < SHIM_MAX_HANDLES; i++) {
        n += s_handles[i].open ? 1 : 0;
    }
    return n;
}
