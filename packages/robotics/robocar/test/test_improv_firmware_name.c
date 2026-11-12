/**
 * @file test_improv_firmware_name.c
 * @brief The improv-wifi component's IMPROV_FIRMWARE_NAME survives `"` and `\`.
 *
 * The component reads its project's flasher.json `name` in CMake and passes it
 * to improv_wifi_device_info.c as IMPROV_FIRMWARE_NAME. It used to paste the
 * name between quotes unescaped, so a name holding a double quote or a
 * backslash broke the compile of every project building the component.
 * test/CMakeLists.txt builds this file with the definition made the way the
 * component makes it (improv_c_string_literal() in improv_c_string.cmake,
 * target_compile_definitions) from a name holding both. Without the escaping
 * this file does not compile, which fails the host suite's build.
 */

#include <stdio.h>
#include <string.h>

#ifndef IMPROV_FIRMWARE_NAME
#error "test/CMakeLists.txt must define IMPROV_FIRMWARE_NAME"
#endif

int main(void)
{
    static const char expected[] = "Robo \"Car\" C:\\bench\\";
    const char *const got = IMPROV_FIRMWARE_NAME;
    printf("improv firmware name: %s\n", got);
    if (strcmp(got, expected) != 0) {
        printf("FAIL: IMPROV_FIRMWARE_NAME is [%s], expected [%s]\n", got, expected);
        return 1;
    }
    printf("1/1 passed\n");
    return 0;
}
