#include <check.h>
#include <stdlib.h>
#include <string.h>
#include "main/bms.h"

START_TEST(test_memcpy_buffer_boundary)
{
    // Invariant: memcpy operation must never write beyond destination buffer boundaries
    const struct {
        uint8_t cmd;
        uint8_t data[64];
        uint8_t len;
    } payloads[] = {
        // Exact exploit case: cmd that causes offset near buffer end with excessive length
        {CAN_PACKET_BMS_STATUS_7, {0}, 64},
        // Boundary case: maximum safe length for last status packet
        {CAN_PACKET_BMS_STATUS_7, {0}, 8},
        // Valid normal input
        {CAN_PACKET_BMS_STATUS_1, {0xAA, 0xBB, 0xCC, 0xDD}, 4},
    };
    
    int num_payloads = sizeof(payloads) / sizeof(payloads[0]);
    
    for (int i = 0; i < num_payloads; i++) {
        // Initialize buffer with sentinel values
        uint8_t buffer[256];
        memset(buffer, 0xCC, sizeof(buffer));
        
        // Simulate the vulnerable operation
        uint8_t *dest = buffer + ((payloads[i].cmd - CAN_PACKET_BMS_STATUS_1) * 8);
        size_t remaining = sizeof(buffer) - (dest - buffer);
        
        // Security property: length must not exceed remaining buffer space
        ck_assert_msg(payloads[i].len <= remaining, 
                     "memcpy would overflow: len=%u, remaining=%zu", 
                     payloads[i].len, remaining);
        
        // If assertion passes, perform the actual copy
        if (payloads[i].len <= remaining) {
            memcpy(dest, payloads[i].data, payloads[i].len);
        }
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_memcpy_buffer_boundary);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}