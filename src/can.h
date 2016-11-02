#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <stdbool.h>

struct canbus_msg {
    uint32_t id;
    bool ide;
    bool rtr;
    uint8_t dlc;
    uint8_t data[8];
};

void canbus_init(void);
void canbus_update(void);
bool canbus_send_message(struct canbus_msg* msg);
bool canbus_recv_message(struct canbus_msg* msg);


#endif // CAN_H
