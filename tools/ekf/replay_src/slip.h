#pragma once

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

static uint8_t slip_decode(uint8_t in_len, uint8_t *in_buf, uint8_t *out_buf) {
    uint8_t i;
    uint8_t out_len = 0;
    uint8_t esc_flag = 0;

    for (i=0; i<in_len; i++) {
        if (esc_flag) {
            if (in_buf[i] == SLIP_ESC_ESC) {
                out_buf[out_len++] = SLIP_ESC;
            } else if (in_buf[i] == SLIP_ESC_END) {
                out_buf[out_len++] = SLIP_END;
            } else {
                // invalid escape character
                return 0;
            }
            esc_flag = 0;
        } else if (in_buf[i] == SLIP_ESC) {
            esc_flag = 1;
        } else if (in_buf[i] == SLIP_END) {
            return out_len;
        } else {
            out_buf[out_len++] = in_buf[i];
        }
    }

    return 0;
}
