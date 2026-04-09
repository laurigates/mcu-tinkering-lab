/**
 * @file ollama_discovery.c
 * @brief Service discovery for Ollama API endpoints using DNS SRV records
 */

#include "ollama_discovery.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "mdns.h"

static const char *TAG = "ollama_discovery";

static ollama_discovery_config_t s_config = {0};
static bool s_initialized = false;

esp_err_t ollama_discovery_init(const ollama_discovery_config_t *config)
{
    if (!config) {
        ESP_LOGE(TAG, "Configuration cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_config, config, sizeof(ollama_discovery_config_t));

    if (s_config.use_mdns) {
        esp_err_t err = mdns_init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize mDNS: %s", esp_err_to_name(err));
            // Continue without mDNS, fall back to regular DNS
        } else {
            ESP_LOGI(TAG, "mDNS initialized successfully");
        }
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Service discovery initialized with SRV record: %s",
             s_config.srv_record ? s_config.srv_record : "none");

    return ESP_OK;
}

/**
 * @brief Resolve hostname to IP address
 */
static esp_err_t resolve_hostname(const char *hostname, char *ip_buffer, size_t buffer_size)
{
    struct addrinfo hints = {0};
    struct addrinfo *result = NULL;

    hints.ai_family = AF_UNSPEC;  // Support both IPv4 and IPv6
    hints.ai_socktype = SOCK_STREAM;

    int err = getaddrinfo(hostname, NULL, &hints, &result);
    if (err != 0 || result == NULL) {
        ESP_LOGE(TAG, "Failed to resolve hostname %s: error %d", hostname, err);
        return ESP_ERR_NOT_FOUND;
    }

    void *addr_ptr = NULL;
    if (result->ai_family == AF_INET) {
        struct sockaddr_in *ipv4 = (struct sockaddr_in *)result->ai_addr;
        addr_ptr = &(ipv4->sin_addr);
    } else if (result->ai_family == AF_INET6) {
        struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)result->ai_addr;
        addr_ptr = &(ipv6->sin6_addr);
    }

    if (addr_ptr == NULL) {
        ESP_LOGE(TAG, "No valid address found for hostname %s", hostname);
        freeaddrinfo(result);
        return ESP_ERR_NOT_FOUND;
    }

    if (inet_ntop(result->ai_family, addr_ptr, ip_buffer, buffer_size) == NULL) {
        ESP_LOGE(TAG, "Failed to convert address to string");
        freeaddrinfo(result);
        return ESP_FAIL;
    }

    freeaddrinfo(result);
    ESP_LOGD(TAG, "Resolved %s to %s", hostname, ip_buffer);
    return ESP_OK;
}

/**
 * @brief Parse SRV record using mDNS
 */
static esp_err_t discover_via_mdns(const char *service_name, ollama_service_info_t *services,
                                   size_t max_services, size_t *num_found)
{
    *num_found = 0;

    if (!s_config.use_mdns) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    // For mDNS discovery, we need to browse for services
    // This is a simplified implementation - in practice, you'd use mdns_query_srv()
    ESP_LOGD(TAG, "Attempting mDNS discovery for service: %s", service_name);

    // Parse service name to extract service and protocol
    // Format: _service._protocol.domain
    char *service_copy = strdup(service_name);
    if (!service_copy) {
        return ESP_ERR_NO_MEM;
    }

    char *service_part = strtok(service_copy, ".");
    char *protocol_part = strtok(NULL, ".");

    if (!service_part || !protocol_part) {
        ESP_LOGE(TAG, "Invalid service name format: %s", service_name);
        free(service_copy);
        return ESP_ERR_INVALID_ARG;
    }

    // Remove leading underscore if present
    if (service_part[0] == '_')
        service_part++;
    if (protocol_part[0] == '_')
        protocol_part++;

    mdns_result_t *results = NULL;
    esp_err_t err = mdns_query_srv(NULL, service_part, protocol_part, 3000, &results);

    free(service_copy);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mDNS query failed: %s", esp_err_to_name(err));
        return err;
    }

    mdns_result_t *r = results;
    size_t count = 0;

    while (r && count < max_services) {
        if (r->instance_name && r->hostname && r->port > 0) {
            ollama_service_info_t *service = &services[count];

            // Copy hostname
            strncpy(service->hostname, r->hostname, sizeof(service->hostname) - 1);
            service->hostname[sizeof(service->hostname) - 1] = '\0';

            // Set port and priority
            service->port = r->port;
            service->priority = 0;  // mDNS doesn't provide priority/weight
            service->weight = 0;

            // Resolve hostname to IP
            if (resolve_hostname(service->hostname, service->ip_addr, sizeof(service->ip_addr)) ==
                ESP_OK) {
                service->is_valid = true;
                count++;
                ESP_LOGI(TAG, "Found service: %s:%d (%s)", service->hostname, service->port,
                         service->ip_addr);
            } else {
                ESP_LOGW(TAG, "Failed to resolve hostname: %s", service->hostname);
            }
        }
        r = r->next;
    }

    if (results) {
        mdns_query_results_free(results);
    }

    *num_found = count;
    return (count > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/**
 * @brief Fallback to configured static URL
 */
static esp_err_t use_fallback_service(ollama_service_info_t *service)
{
    if (!s_config.fallback_url) {
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Using fallback URL: %s", s_config.fallback_url);

    // Parse the fallback URL to extract hostname and port
    // Expected format: http://hostname:port/path
    const char *url = s_config.fallback_url;

    // Skip protocol
    if (strncmp(url, "http://", 7) == 0) {
        url += 7;
    } else if (strncmp(url, "https://", 8) == 0) {
        url += 8;
    }

    // Find hostname end (either ':' for port or '/' for path)
    const char *hostname_end = url;
    while (*hostname_end && *hostname_end != ':' && *hostname_end != '/') {
        hostname_end++;
    }

    size_t hostname_len = hostname_end - url;
    if (hostname_len >= sizeof(service->hostname)) {
        hostname_len = sizeof(service->hostname) - 1;
    }

    strncpy(service->hostname, url, hostname_len);
    service->hostname[hostname_len] = '\0';

    // Extract port
    service->port = 11434;  // Default Ollama port
    if (*hostname_end == ':') {
        service->port = atoi(hostname_end + 1);
    }

    // Resolve hostname to IP
    if (resolve_hostname(service->hostname, service->ip_addr, sizeof(service->ip_addr)) != ESP_OK) {
        // If resolution fails, try using the hostname as-is (might be an IP)
        strncpy(service->ip_addr, service->hostname, sizeof(service->ip_addr) - 1);
        service->ip_addr[sizeof(service->ip_addr) - 1] = '\0';
    }

    service->priority = 10;  // Lower priority than discovered services
    service->weight = 1;
    service->is_valid = true;

    ESP_LOGI(TAG, "Fallback service: %s:%d (%s)", service->hostname, service->port,
             service->ip_addr);
    return ESP_OK;
}

esp_err_t ollama_discovery_find_services(ollama_service_info_t *services, size_t max_services,
                                         size_t *num_found)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Service discovery not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!services || !num_found || max_services == 0) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    *num_found = 0;
    memset(services, 0, sizeof(ollama_service_info_t) * max_services);

    // Try mDNS discovery first if enabled and SRV record is provided
    if (s_config.srv_record && s_config.use_mdns) {
        esp_err_t err = discover_via_mdns(s_config.srv_record, services, max_services, num_found);
        if (err == ESP_OK && *num_found > 0) {
            ESP_LOGI(TAG, "Found %d service(s) via mDNS", *num_found);
            return ESP_OK;
        }
        ESP_LOGD(TAG, "mDNS discovery failed or found no services");
    }

    // TODO: Add standard DNS SRV record lookup here
    // This would use getaddrinfo() or custom DNS resolution for SRV records
    ESP_LOGD(TAG, "Standard DNS SRV lookup not yet implemented");

    // Fall back to configured static service
    if (*num_found == 0 && max_services > 0) {
        esp_err_t err = use_fallback_service(&services[0]);
        if (err == ESP_OK) {
            *num_found = 1;
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "No services discovered");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t ollama_discovery_get_best_service(const ollama_service_info_t *services,
                                            size_t num_services,
                                            ollama_service_info_t *best_service)
{
    if (!services || !best_service || num_services == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const ollama_service_info_t *best = NULL;

    // Find the service with highest priority (lowest priority number)
    for (size_t i = 0; i < num_services; i++) {
        if (!services[i].is_valid)
            continue;

        if (best == NULL || services[i].priority < best->priority) {
            best = &services[i];
        } else if (services[i].priority == best->priority && services[i].weight > best->weight) {
            // Same priority, prefer higher weight
            best = &services[i];
        }
    }

    if (best == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(best_service, best, sizeof(ollama_service_info_t));
    ESP_LOGI(TAG, "Best service: %s:%d (priority=%d, weight=%d)", best_service->hostname,
             best_service->port, best_service->priority, best_service->weight);

    return ESP_OK;
}

esp_err_t ollama_discovery_build_url(const ollama_service_info_t *service, const char *path,
                                     char *url_buffer, size_t buffer_size)
{
    if (!service || !service->is_valid || !path || !url_buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    // Use IP address if available, otherwise hostname
    const char *address = (strlen(service->ip_addr) > 0) ? service->ip_addr : service->hostname;

    // Handle IPv6 addresses (need brackets)
    bool is_ipv6 = (strchr(address, ':') != NULL && strchr(address, '.') == NULL);

    int ret;
    if (is_ipv6) {
        ret = snprintf(url_buffer, buffer_size, "http://[%s]:%d%s", address, service->port, path);
    } else {
        ret = snprintf(url_buffer, buffer_size, "http://%s:%d%s", address, service->port, path);
    }

    if (ret < 0 || (size_t)ret >= buffer_size) {
        ESP_LOGE(TAG, "URL buffer too small");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGD(TAG, "Built URL: %s", url_buffer);
    return ESP_OK;
}

void ollama_discovery_deinit(void)
{
    if (s_config.use_mdns) {
        mdns_free();
    }

    memset(&s_config, 0, sizeof(s_config));
    s_initialized = false;

    ESP_LOGI(TAG, "Service discovery deinitialized");
}
