#include "CAN_bus.h"
#include "enabled.h"
#include "serial_monitor.h"

bool PL_CANBus_Init(PL_CANBus_Handler *c1, CAN_HandleTypeDef *hcan_1, PL_CANBus_Handler *c2, CAN_HandleTypeDef *hcan_2){
#if CAN_BUS_ENABLED
    // Initialize CAN bus handlers
    c1->hcan = hcan_1;
    c1->command_ready = false;
    c2->hcan = hcan_2;
    c2->command_ready = false;

    // Configure transmit headers for CAN1
    for (int i = 0; i < N_MESSAGES; i++)
    {
        c1->Tx_headers[i].DLC = 8;
        c1->Tx_headers[i].ExtId = 0;
        c1->Tx_headers[i].IDE = CAN_ID_STD;
        c1->Tx_headers[i].RTR = CAN_RTR_DATA;
        c1->Tx_headers[i].StdId = CAN_BASE_ID + i;
        c1->Tx_headers[i].TransmitGlobalTime = DISABLE;

        // Configure transmit headers for CAN2
		c2->Tx_headers[i].DLC = 8;
		c2->Tx_headers[i].ExtId = 0;
		c2->Tx_headers[i].IDE = CAN_ID_STD;
		c2->Tx_headers[i].RTR = CAN_RTR_DATA;
		c2->Tx_headers[i].StdId = 0x05; //RESET PAYLOAD
		c2->Tx_headers[i].TransmitGlobalTime = DISABLE;
    }

    // Configure filter banks
    uint32_t command_ids[2] = {RESET_PAYLOAD, TOGGLE_SAMPLING};
    CAN_FilterTypeDef filter_config;
    bool filter_config_success = true;

    // CAN1 filters
    for (int i = 0; i < 2; i++)
    {
        filter_config.FilterActivation = CAN_FILTER_ENABLE;
        filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
        filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
        filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter_config.FilterBank = i;
        filter_config.SlaveStartFilterBank = 13;

        filter_config.FilterIdHigh = command_ids[i] << 5;
        filter_config.FilterIdLow = 0x0000;
        filter_config.FilterMaskIdHigh = 0xFFFF << 5;
        filter_config.FilterMaskIdLow = 0x0000;

        filter_config_success &= (HAL_CAN_ConfigFilter(hcan_1, &filter_config) == HAL_OK);
    }

    // CAN2 filters
    for (int i = 0; i < N_ADAPTER_MSGS; i++)
    {
        filter_config.FilterActivation = CAN_FILTER_ENABLE;
        filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
        filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
        filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter_config.FilterBank = 13 + i;
        filter_config.SlaveStartFilterBank = 13;

        filter_config.FilterIdHigh = (ADAPTER_CAN_BASE_ID + i) << 5;
        filter_config.FilterIdLow = 0x0000;
        filter_config.FilterMaskIdHigh = 0xFFFF << 5;
        filter_config.FilterMaskIdLow = 0x0000;

        filter_config_success &= (HAL_CAN_ConfigFilter(hcan_2, &filter_config) == HAL_OK);
    }

    // Start CAN peripherals
    HAL_StatusTypeDef start_status_1 = HAL_CAN_Start(hcan_1);
    HAL_StatusTypeDef start_status_2 = HAL_CAN_Start(hcan_2);

    // Enable receive interrupts
    HAL_StatusTypeDef interrupt_status_1 = HAL_CAN_ActivateNotification(hcan_1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_StatusTypeDef interrupt_status_2 = HAL_CAN_ActivateNotification(hcan_2, CAN_IT_RX_FIFO0_MSG_PENDING);

    return (start_status_1 == HAL_OK) &&
           (start_status_2 == HAL_OK) &&
           (interrupt_status_1 == HAL_OK) &&
           (interrupt_status_2 == HAL_OK) &&
           filter_config_success;

#else
    return true;
#endif
}

bool PL_CANBus_Receive(PL_CANBus_Handler *c)
{
#if CAN_BUS_ENABLED
    bool received = HAL_CAN_GetRxMessage(c->hcan, CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *) &(c->Rx_header), (uint8_t *) c->Rx_data) == HAL_OK;
    c->command_ready = received;
    return received;
#else
    return true;
#endif
}

void PL_CANBus_ForwardToFC(PL_CANBus_Handler *fc, PL_CANBus_Handler *adapter)
{
#if CAN_BUS_ENABLED
    // Sensor index and per-sample index of the frame just received from the adapter.
    // Raw frame layout: bytes[0..5] = x,y,z (int16 LE), bytes[6..7] = sample index (uint16 LE).
    uint16_t sensor = (uint16_t)(adapter->Rx_header.StdId - ADAPTER_CAN_BASE_ID);
    uint16_t index  = (uint16_t)(adapter->Rx_data[6] | (adapter->Rx_data[7] << 8));

    // A new half-buffer burst begins at the first sensor's first sample. Decide once per
    // burst whether this whole burst is forwarded, so its frames stay contiguous.
    static uint32_t burst_count = 0;
    static uint32_t relay_select = 0;
    static bool forwarding = false;
    if (sensor == 0 && index == 0)
    {
        forwarding = false;
        if (burst_count % IEPE_RELAY_EVERY_N_BURSTS == 0)
        {
            // Of the bursts picked by the 1-in-N gate, forward every other one
            // so the relay runs at 50% of its original (pre-throttle) rate.
            forwarding = (relay_select++ % 4 != 3);
        }
        burst_count++;
    }

    if (!forwarding)
        return;

    // The FC only retains the latest x/y/z value for each adapter and ignores
    // the sample index. Forward one frame per sensor from a selected burst
    // instead of relaying the entire raw burst and overflowing the FC queue.
    if (index != 0)
        return;

    CAN_TxHeaderTypeDef header;
    header.DLC = 8;
    header.ExtId = 0;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.StdId = RELAY_CAN_BASE_ID + sensor;
    header.TransmitGlobalTime = DISABLE;

    // Non-blocking: only loads a mailbox. Safe from the RX ISR; frame dropped if all full.
    uint32_t Tx_mailbox;
    HAL_StatusTypeDef tx = HAL_CAN_AddTxMessage(fc->hcan, &header, (uint8_t *)adapter->Rx_data, &Tx_mailbox);

    // Trace once per forwarded burst (its first frame), NOT per frame — logging every
    // relayed frame from this ISR would flood the UART. burst_count was already advanced
    // at the boundary above, so the burst being forwarded now is (burst_count - 1).
    if (sensor == 0 && index == 0)
    {
        int16_t x = (int16_t)(adapter->Rx_data[0] | (adapter->Rx_data[1] << 8));
        int16_t y = (int16_t)(adapter->Rx_data[2] | (adapter->Rx_data[3] << 8));
        int16_t z = (int16_t)(adapter->Rx_data[4] | (adapter->Rx_data[5] << 8));
        PL_Log(LOG_CAN_BUS, LOG_NONE, (tx == HAL_OK) ? LOG_OK : LOG_ERROR,
               "Relay -> FC: burst #%lu -> 0x%03lX first x/y/z=%d/%d/%d (1 of every %d)",
               (unsigned long)(burst_count - 1),
               (unsigned long)(RELAY_CAN_BASE_ID + sensor),
               x, y, z, IEPE_RELAY_EVERY_N_BURSTS);
    }
#endif
}

void PL_CANBus_LogMessage(const char *bus_name, PL_CANBus_Handler *c)
{
#if CAN_BUS_ENABLED
    uint8_t dlc = (uint8_t)(c->Rx_header.DLC > 8 ? 8 : c->Rx_header.DLC);
    uint32_t id = (c->Rx_header.IDE == CAN_ID_EXT) ? c->Rx_header.ExtId
                                                   : c->Rx_header.StdId;

    // Format the data bytes into a small buffer: "AA BB CC ..." (up to 8 bytes).
    char data_str[3 * 8 + 1];
    int pos = 0;
    for (uint8_t i = 0; i < dlc; i++)
    {
        pos += snprintf(&data_str[pos], sizeof(data_str) - pos,
                        "%02X ", c->Rx_data[i]);
    }
    if (pos == 0)
    {
        data_str[0] = '\0';
    }

    PL_Log(LOG_CAN_BUS, LOG_NONE, LOG_OK,
           "%s: RX ID=0x%03lX DLC=%u data=[ %s]",
           bus_name, (unsigned long)id, (unsigned)dlc, data_str);
#endif
}

struct command PL_CANBus_ParseCommand_1(PL_CANBus_Handler *c)
{
    struct command com;
    com.type = NONE;

#if CAN_BUS_ENABLED
    union command_data data;
    data.on = 0;
    if (c->command_ready)
    {
        switch (c->Rx_header.StdId)
        {
        case RESET_PAYLOAD:
            com.type = RESET_PAYLOAD;
            break;
        case TOGGLE_SAMPLING:
            // toggled state is tracked in main.c
            com.type = TOGGLE_SAMPLING;
            break;
        default:
            com.type = INVALID;
        }
    }
    com.data = data;
    c->command_ready = false;
#endif
    return com;
}

struct command PL_CANBus_ParseCommand_2(PL_CANBus_Handler *c)
{
    struct command com;
    com.type = NONE;

#if CAN_BUS_ENABLED
    if (c->command_ready)
    {
        switch (c->Rx_header.StdId)
        {
        case ADAPTER_CAN_BASE_ID + 0:
            com.type = ADAPTER_CMD_0;
            break;
        case ADAPTER_CAN_BASE_ID + 1:
            com.type = ADAPTER_CMD_1;
            break;
        case ADAPTER_CAN_BASE_ID + 2:
            com.type = ADAPTER_CMD_2;
            break;
        case ADAPTER_CAN_BASE_ID + 3:
            com.type = ADAPTER_CMD_3;
            break;
        case ADAPTER_CAN_BASE_ID + 4:
            com.type = ADAPTER_CMD_4;
            break;
        case ADAPTER_CAN_BASE_ID + 5:
            com.type = ADAPTER_CMD_5;
            break;
        case ADAPTER_CAN_BASE_ID + 6:
            com.type = ADAPTER_CMD_6;
            break;
        case ADAPTER_CAN_BASE_ID + 7:
            com.type = ADAPTER_CMD_7;
            break;
        case ADAPTER_CAN_BASE_ID + 8:
            com.type = ADAPTER_CMD_8;
            break;
        case ADAPTER_CAN_BASE_ID + 9:
            com.type = ADAPTER_CMD_9;
            break;
        default:
            com.type = INVALID;
            break;
        }
        c->command_ready = false;
    }
#endif

    return com;
}

bool PL_CANBus_Send(
    PL_CANBus_Handler *c,
    bool ok,
	bool sampling_state,
    uint16_t frequency_x,
    uint16_t frequency_y,
    uint16_t frequency_z,
    uint16_t amplitude_x,
    uint16_t amplitude_y,
    uint16_t amplitude_z,
    uint32_t time_elapsed
)
{
#if CAN_BUS_ENABLED
    struct CAN_msg_1_s msg1;
    msg1.ok = ok;
    msg1.sampling_state=sampling_state;

    struct CAN_msg_2_s msg2;
    msg2.ampX = amplitude_x;
    msg2.frqX = frequency_x;
    msg2.frqY = frequency_y;
    msg2.frqZ = frequency_z;

    struct CAN_msg_3_s msg3;
    msg3.ampY = amplitude_y;
    msg3.ampZ = amplitude_z;
    msg3.time_elapsed = time_elapsed;

    union CAN_msg_1_u msg1_u;
    msg1_u.msg = msg1;

    union CAN_msg_2_u msg2_u;
    msg2_u.msg = msg2;

    union CAN_msg_3_u msg3_u;
    msg3_u.msg = msg3;

    HAL_StatusTypeDef status1;
    HAL_StatusTypeDef status2;
    HAL_StatusTypeDef status3;

    uint32_t Tx_mailbox;

    status1 = HAL_CAN_AddTxMessage(c->hcan, &((c->Tx_headers)[0]), msg1_u.bytes, &Tx_mailbox);
    uint32_t time_message_sent = HAL_GetTick();
    uint32_t current;
    do
    {
        current = HAL_GetTick();
    } while (HAL_CAN_IsTxMessagePending(c->hcan, Tx_mailbox) && current - time_message_sent < CAN_SEND_TIMEOUT);

    status2 = HAL_CAN_AddTxMessage(c->hcan, &((c->Tx_headers)[1]), msg2_u.bytes, &Tx_mailbox);
    time_message_sent = HAL_GetTick();
    do
    {
        current = HAL_GetTick();
    } while (HAL_CAN_IsTxMessagePending(c->hcan, Tx_mailbox) && current - time_message_sent < CAN_SEND_TIMEOUT);

    status3 = HAL_CAN_AddTxMessage(c->hcan, &((c->Tx_headers)[2]), msg3_u.bytes, &Tx_mailbox);
    time_message_sent = HAL_GetTick();
    do
    {
        current = HAL_GetTick();
    } while (HAL_CAN_IsTxMessagePending(c->hcan, Tx_mailbox) && current - time_message_sent < CAN_SEND_TIMEOUT);

    return status1 == HAL_OK && status2 == HAL_OK && status3 == HAL_OK;
#else
    return true;
#endif
}
