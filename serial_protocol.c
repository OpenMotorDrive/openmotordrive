#include "serial_protocol.h"
#include "serial.h"

static int32_t encode_slip(char* buf_in, uint16_t len, char* buf_out, uint16_t max_len);
static bool serial_protocol_send_frame(char* buf, uint16_t len);

static bool serial_protocol_send_frame(char* buf, uint16_t len)
{
    char encoded[256];
    int32_t encoded_len;
    encoded_len = encode_slip(buf, len, encoded, 256);
    if (encoded_len == -1) {
        return false;
    }

    return serial_send_dma(encoded_len, encoded);
}

static int32_t encode_slip(char* buf_in, uint16_t len_in, char* buf_out, uint16_t max_len_out)
{
    uint16_t i;
    uint16_t len_out = 0;
    for (i=0; i<len_in; i++) {
        if (buf_in[i] == SLIP_END) {
            if (len_out > max_len_out-3) return -1;
            buf_out[len_out++] = SLIP_ESC;
            buf_out[len_out++] = SLIP_ESC_END;
        } else if (buf_in[i] == SLIP_ESC) {
            if (len_out > max_len_out-3) return -1;
            buf_out[len_out++] = SLIP_ESC;
            buf_out[len_out++] = SLIP_ESC_ESC;
        } else {
            if (len_out > max_len_out-2) return -1;
            buf_out[len_out++] = buf_in[i];
        }
    }
    buf_out[len_out++] = SLIP_END;
    return len_out;
}
