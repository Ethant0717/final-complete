
#include "app.h"
#include "main.h"
#include "control.h"
#include <string.h>
#include <stdio.h>

/* =========================================================
   EXTERNAL UART HANDLES
   ========================================================= */
extern UART_HandleTypeDef huart1; // ESP32
extern UART_HandleTypeDef huart2; // Debug

/* =========================================================
   RX SYSTEM
   ========================================================= */
static char rxBuffer[64];

static volatile uint8_t rxChar;
static volatile uint16_t idx = 0;
static volatile uint8_t packetReady = 0;
static uint8_t rxHeartbeat = 0;


/* =========================================================
   CONTROL VALUES (parsed output)
   ========================================================= */
static float U = 0, D = 0, P = 0, R = 0, Y = 0;



/* =========================================================
   TIMING / FAILSAFE
   ========================================================= */
static uint32_t lastPacketTime = 0;
static const uint32_t PACKET_TIMEOUT_MS = 200;

/* =========================================================
   INIT
   ========================================================= */
void App_Init(void)
{
    memset(rxBuffer, 0, sizeof(rxBuffer));

    idx = 0;
    packetReady = 0;
    lastPacketTime = HAL_GetTick();

    /* Clear UART errors */
    __HAL_UART_CLEAR_OREFLAG(&huart1);

    /* Start UART interrupt reception */
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxChar, 1);
}

/* =========================================================
   UART RX INTERRUPT CALLBACK (KEEP MINIMAL)
   ========================================================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        char c = (char)rxChar;

        if (c == '\n' || c == '\r')
        {
            if (idx > 0)
            {
                rxBuffer[idx] = '\0';
                packetReady = 1;
            }
            idx = 0;
        }
        else
        {
            if (idx < (sizeof(rxBuffer) - 1))
            {
                rxBuffer[idx++] = c;
            }
            else
            {
                /* overflow protection */
                idx = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxChar, 1);
    }
}

/* =========================================================
   UART ERROR HANDLER (RECOVERY)
   ========================================================= */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);

        idx = 0;
        packetReady = 0;

        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxChar, 1);
    }
}

/* =========================================================
   PACKET PARSER
   Expected format:
   U1.0 D0.0 P0.1 R0.2 Y-0.3
   ========================================================= */
static void App_ParsePacket(char *msg)
{
    float u, d, p, r, y;

    int ok = sscanf(msg,
        "U%f, D%f, P%f, R%f, Y%f",
        &u, &d, &p, &r, &y);

    if (ok == 5)
    {
        U = u;
        D = d;
        P = p;
        R = r;
        Y = y;

        sp.throttle = 1000+(U - D)*400.0f;
        sp.pitch = P;
        sp.roll  = R;
        sp.yaw   = Y;

        lastPacketTime = HAL_GetTick();
        rxHeartbeat = 1;
    }
}

/* =========================================================
   MAIN UPDATE FUNCTION (CALL IN WHILE LOOP)
   ========================================================= */
void App_Update(void)
{
    /* -----------------------------
       Handle new packet
       ----------------------------- */
	if (packetReady)
	{
	    packetReady = 0;

	    HAL_UART_Transmit(&huart2,
	        (uint8_t*)rxBuffer,
	        strlen(rxBuffer),
	        100);

	    HAL_UART_Transmit(&huart2,
	        (uint8_t*)"\r\n",
	        2,
	        10);

	    App_ParsePacket(rxBuffer);
	}


    /* -----------------------------
       FAILSAFE TIMEOUT
       ----------------------------- */
	if ((HAL_GetTick() - lastPacketTime) > PACKET_TIMEOUT_MS)
	{
	    sp.throttle = 1100;   // must be above MOTOR_MIN
	    sp.pitch = 0;
	    sp.roll = 0;
	    sp.yaw = 0;
	}

    /* -----------------------------
       RX STATUS DEBUG
       ----------------------------- */
    static uint32_t lastDebug = 0;

    if (HAL_GetTick() - lastDebug >= 500)
    {
        lastDebug = HAL_GetTick();

//        if (rxHeartbeat)
//        {
//            HAL_UART_Transmit(&huart2,
//                (uint8_t*)"RX DATA OK\r\n",
//                12,
//                10);
//        }
//        else
//        {
//            HAL_UART_Transmit(&huart2,
//                (uint8_t*)"NO DATA\r\n",
//                9,
//                10); }

        rxHeartbeat = 0;
    }

}
