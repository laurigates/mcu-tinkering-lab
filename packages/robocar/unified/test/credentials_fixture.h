/**
 * @file credentials_fixture.h — the credentials.h test_credentials_loader builds against.
 *
 * Stands in for the untracked main/credentials.h, whose contents depend on
 * whoever built last: the CMake stub, or a developer's real broker password.
 * test/CMakeLists.txt copies credentials_loader.c into the build tree and
 * copies this file next to it as credentials.h, so the loader's
 * `#include "credentials.h"` resolves here through the same-directory rule.
 *
 * It is not named credentials.h in the source tree because that name is
 * ignored by the repo and refused by the check-credentials pre-commit hook.
 *
 * Built twice. Without CREDENTIALS_FIXTURE_NO_MQTT this is a developer build
 * with broker credentials compiled in; with it, it matches the CMake stub,
 * which leaves both MQTT values undefined so the loader's "" defaults apply.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_CREDENTIALS_FIXTURE_H
#define ROBOCAR_UNIFIED_HOST_TEST_CREDENTIALS_FIXTURE_H

#define WIFI_SSID "fixture-ssid"
#define WIFI_PASSWORD "fixture-wifi-pass"

#ifndef CREDENTIALS_FIXTURE_NO_MQTT
#define MQTT_USERNAME "file-user"
#define MQTT_PASSWORD "file-pass"
#endif

#endif /* ROBOCAR_UNIFIED_HOST_TEST_CREDENTIALS_FIXTURE_H */
