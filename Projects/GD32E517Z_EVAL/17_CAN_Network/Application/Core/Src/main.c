/*!
    \file    main.c
    \brief   CAN network communication

    \version 2025-02-13, V1.2.0, demo for GD32E51x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32e51x.h"
#include <stdio.h>
#include "gd32e517z_eval.h"
#include "systick.h"

#define DEV_CAN0_ID          0xaabb

can_trasnmit_message_struct g_transmit_message;
can_receive_message_struct g_receive_message;
FlagStatus can0_receive_flag;
/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_gpio_config(void)
{
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_AF);

    /* configure CAN0 GPIO */
    gpio_init(GPIOD, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    gpio_pin_remap_config(GPIO_CAN0_FULL_REMAP, ENABLE);
}

/*!
    \brief      configure BSP
    \param[in]  none
    \param[out] none
    \retval     none
*/
void bsp_board_config(void)
{
    /* configure USART */
    gd_eval_com_init(EVAL_COM0);

    /* configure KEY_B key */
    gd_eval_key_init(KEY_B, KEY_MODE_GPIO);

    /* configure leds */
    gd_eval_led_init(LED2);
    gd_eval_led_off(LED2);

}

/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_config(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    /* initialize CAN register */
    can_deinit(CAN0);

    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    /* configure CAN0 baud rate 1Mbps */
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    can_parameter.prescaler = 9U;
    /* initialize CAN */
    can_init(CAN0, &can_parameter);

    /* initialize filter */
    /* configure filter mode */
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    /* configure filter ID, only the 0xaabb expand ID can be received */
    can_filter.filter_list_high = 0x0000U;
    can_filter.filter_list_low = 0x0000U;
    /* configure filter mask */
    can_filter.filter_mask_high = 0x0000U;
    can_filter.filter_mask_low = 0x0000U;
    /* select receiver fifo */
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_number = 0U;
    can_filter.filter_enable = ENABLE;
    can_filter_init(CAN0, &can_filter);

    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0U, 0U);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}

void communication_check(void)
{
    uint8_t i = 0U;
    /* CAN0 receive data correctly, the received data is printed */
    if(SET == can0_receive_flag) {
        can0_receive_flag = RESET;
        printf("\r\n can0 receive data:");
        for(i = 0U; i < g_receive_message.rx_dlen; i++) {
            printf(" %02x", g_receive_message.rx_data[i]);
        }
        gd_eval_led_toggle(LED2);
    }
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    uint8_t i = 0U;
    /* configure systick */
    systick_config();
    /* configure board */
    bsp_board_config();
    /* configure GPIO */
    can_gpio_config();
    /* initialize CAN and filter */
    can_config();

    printf("\r\n communication test CAN0, please press KEY_B key to start! \r\n");

    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &g_transmit_message);
    g_transmit_message.tx_sfid = 0x00U;
    g_transmit_message.tx_efid = 0x00U;
    g_transmit_message.tx_ft = CAN_FT_DATA;
    g_transmit_message.tx_ff = CAN_FF_EXTENDED;
    g_transmit_message.tx_dlen = 8U;
    /* initialize receive message */
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &g_receive_message);

    while(1) {
        /* test whether the KEY_B key is pressed */
        if(0 == gd_eval_key_state_get(KEY_B)) {
            delay_ms(100);
            if(0 == gd_eval_key_state_get(KEY_B)) {
                g_transmit_message.tx_efid = DEV_CAN0_ID;
                g_transmit_message.tx_data[0] = 0xA0U;
                g_transmit_message.tx_data[1] = 0xA1U;
                g_transmit_message.tx_data[2] = 0xA2U;
                g_transmit_message.tx_data[3] = 0xA3U;
                g_transmit_message.tx_data[4] = 0xA4U;
                g_transmit_message.tx_data[5] = 0xA5U;
                g_transmit_message.tx_data[6] = 0xA6U;
                g_transmit_message.tx_data[7] = 0xA7U;
                printf("\r\n can0 transmit data:");
                for(i = 0U; i < g_transmit_message.tx_dlen; i++) {
                    printf(" %02x", g_transmit_message.tx_data[i]);
                }
                /* transmit message */
                can_message_transmit(CAN0, &g_transmit_message);
                /* waiting for the KEY_B key up */
                while(0 == gd_eval_key_state_get(KEY_B));
            }
        }
        communication_check();
    }
}

/* retarget the C library printf function to the usart */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    return ch;
}
