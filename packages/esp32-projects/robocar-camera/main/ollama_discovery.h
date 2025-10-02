/**
 * @file ollama_discovery.h
 * @brief Service discovery for Ollama API endpoints using DNS SRV records
 */

#ifndef OLLAMA_DISCOVERY_H
#define OLLAMA_DISCOVERY_H

#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Structure to hold discovered service information
 */
typedef struct {
    char hostname[256];     /**< Hostname from SRV record */
    char ip_addr[46];       /**< Resolved IP address (IPv4 or IPv6) */
    uint16_t port;          /**< Port number from SRV record */
    uint16_t priority;      /**< Priority from SRV record */
    uint16_t weight;        /**< Weight from SRV record */
    bool is_valid;          /**< Whether this entry contains valid data */
} ollama_service_info_t;

/**
 * @brief Configuration for service discovery
 */
typedef struct {
    const char* srv_record;     /**< SRV record to query (e.g., "_ollama._tcp.local") */
    const char* fallback_url;   /**< Fallback URL if SRV discovery fails */
    uint32_t timeout_ms;        /**< DNS query timeout in milliseconds */
    bool use_mdns;              /**< Whether to use mDNS for local discovery */
} ollama_discovery_config_t;

/**
 * @brief Initialize the service discovery module
 * @param config Configuration for service discovery
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ollama_discovery_init(const ollama_discovery_config_t* config);

/**
 * @brief Discover Ollama services using SRV records
 * @param services Array to store discovered services
 * @param max_services Maximum number of services to discover
 * @param num_found Pointer to store the number of services found
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ollama_discovery_find_services(ollama_service_info_t* services, 
                                         size_t max_services, 
                                         size_t* num_found);

/**
 * @brief Get the best available service based on priority and weight
 * @param services Array of discovered services
 * @param num_services Number of services in the array
 * @param best_service Pointer to store the best service info
 * @return ESP_OK if a service was found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t ollama_discovery_get_best_service(const ollama_service_info_t* services,
                                           size_t num_services,
                                           ollama_service_info_t* best_service);

/**
 * @brief Build a complete URL from service information
 * @param service Service information
 * @param path API path (e.g., "/api/generate")
 * @param url_buffer Buffer to store the complete URL
 * @param buffer_size Size of the URL buffer
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ollama_discovery_build_url(const ollama_service_info_t* service,
                                     const char* path,
                                     char* url_buffer,
                                     size_t buffer_size);

/**
 * @brief Clean up resources used by service discovery
 */
void ollama_discovery_deinit(void);

#endif // OLLAMA_DISCOVERY_H