#define BX_CAN1_BASE BX_CAN_BASE

#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "can.h"

void canbus_init(void) {
    // Enable peripheral clock
    rcc_periph_clock_enable(RCC_CAN);
    rcc_periph_clock_enable(RCC_GPIOA);

    // Enable GPIO
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11|GPIO12);
    gpio_set_af(GPIOA, GPIO_AF9, GPIO11|GPIO12);

    can_reset(CAN1);
    can_init(
        CAN1,             /* CAN register base address */
        false,            /* TTCM: Time triggered comm mode? */
        true,             /* ABOM: Automatic bus-off management? */
        false,            /* AWUM: Automatic wakeup mode? */
        false,            /* NART: No automatic retransmission? */
        false,            /* RFLM: Receive FIFO locked mode? */
        false,            /* TXFP: Transmit FIFO priority? */
        CAN_BTR_SJW_1TQ,  /* Resynchronization time quanta jump width.*/
        CAN_BTR_TS1_10TQ, /* Time segment 1 time quanta width. */
        CAN_BTR_TS2_7TQ,  /* Time segment 2 time quanta width. */
        2,                /* Baud rate prescaler. */
        false,            /* Loopback */
        false             /* Silent */
    );


    can_filter_id_mask_32bit_init(
        CAN1,  /* CAN register base address */
        0,     /* Filter ID */
        0,     /* CAN ID */
        0,     /* CAN ID mask */
        0,     /* FIFO assignment (here: FIFO0) */
        true
    );
}

bool canbus_send_message(struct canbus_msg* msg) {
    return can_transmit(
        CAN1,
        msg->id,  /* (EX/ST)ID: CAN ID */
        msg->ide, /* IDE: CAN ID extended? */
        msg->rtr, /* RTR: Request transmit? */
        msg->dlc, /* DLC: Data length */
        msg->data
    ) != -1;
}

bool canbus_recv_message(struct canbus_msg* msg) {
    if ((CAN_RF0R(CAN1)&0b11) == 0) {
        return false;
    }
    uint32_t fmi;
    can_receive(
        CAN1,
        0,
        true,
        &(msg->id),
        &(msg->ide),
        &(msg->rtr),
        &fmi,
        &(msg->dlc),
        msg->data);
    return true;
}
