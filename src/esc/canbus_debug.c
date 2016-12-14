#include <esc/canbus_debug.h>
#include <esc/can.h>
#include <esc/ringbuf.h>

#define DBGBUF_LEN 256

static volatile char _dbgbuf[DBGBUF_LEN];
static volatile struct ringbuf_t dbgbuf = {_dbgbuf, DBGBUF_LEN, 0, 0};

void canbus_debug_update(void) {
    uint8_t i;
    struct canbus_msg msg;
    msg.dlc = ringbuf_size(&dbgbuf);

    if (msg.dlc == 0) {
        return;
    }

    if (msg.dlc > 8) {
        msg.dlc = 8;
    }

    msg.id = 0x7FF;
    msg.ide = 0;
    msg.rtr = 0;

    for (i=0; i<msg.dlc; i++) {
        ringbuf_peek(&dbgbuf, i, (char*)&msg.data[i]);
    }

    if (!canbus_send_message(&msg)) {
        return;
    }

    for (i=0; i<msg.dlc; i++) {
        uint8_t temp;
        ringbuf_pop(&dbgbuf, (char*)&temp);
    }
}

void canbus_debug_write(uint8_t len, char* buf) {
    uint8_t i;
    for (i=0; i<len; i++) {
        ringbuf_push(&dbgbuf, buf[i]);
    }
}
