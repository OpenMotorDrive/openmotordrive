/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <esc/ringbuf.h>

#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <esc/can.h>

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
        true,             /* NART: No automatic retransmission? */
        false,            /* RFLM: Receive FIFO locked mode? */
        true,            /* TXFP: Transmit FIFO priority? */
        CAN_BTR_SJW_1TQ,  /* Resynchronization time quanta jump width.*/
        CAN_BTR_TS1_15TQ, /* Time segment 1 time quanta width. */
        CAN_BTR_TS2_2TQ,  /* Time segment 2 time quanta width. */
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
