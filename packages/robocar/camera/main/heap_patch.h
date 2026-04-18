/**
 * @file heap_patch.h
 * @brief Temporary patch for ESP-IDF 6.0 TLSF heap compatibility
 *
 * This header provides a workaround for the missing tlsf_find_containing_block
 * function until ESP-IDF submodules are properly updated.
 */

#ifndef HEAP_PATCH_H
#define HEAP_PATCH_H

#ifdef __cplusplus
extern "C" {
#endif

// Temporary workaround - disable heap debugging features that use TLSF functions
// This should be removed once ESP-IDF submodules are updated
#ifdef CONFIG_HEAP_TRACING
#undef CONFIG_HEAP_TRACING
#endif

#ifdef CONFIG_HEAP_CORRUPTION_DETECTION
#undef CONFIG_HEAP_CORRUPTION_DETECTION
#endif

#ifdef __cplusplus
}
#endif

#endif  // HEAP_PATCH_H
