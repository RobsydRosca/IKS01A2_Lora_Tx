/**
 ******************************************************************************
 * @file    main.cpp
 * @author  AST / EST
 * @version V0.0.1
 * @date    14-August-2015
 * @brief   Simple Example application for using the X_NUCLEO_IKS01A1 
 *          MEMS Inertial & Environmental Sensor Nucleo expansion board.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/

/* Includes */
#include "mbed.h"
#include "x_nucleo_iks01a1.h"
#include "main.h"
#include "sx1276-hal.h"

// ==================== LoRa settings ===============================


/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE 1

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    868000000 // Hz
#define TX_OUTPUT_POWER                                 14        // 14 dBm

#if USE_MODEM_LORA == 1

    #define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       12        // [SF7..SF12] default 7
    #define LORA_CODINGRATE                             1         // [1: 4/5,
                                                                  //  2: 4/6,
                                                                  //  3: 4/7,
                                                                  //  4: 4/8]
    #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
    #define LORA_SYMBOL_TIMEOUT                         5         // Symbols
    #define LORA_FIX_LENGTH_PAYLOAD_ON                  false
    #define LORA_FHSS_ENABLED                           false
    #define LORA_NB_SYMB_HOP                            4
    #define LORA_IQ_INVERSION_ON                        false
    #define LORA_CRC_ENABLED                            true

#elif USE_MODEM_FSK == 1

    #define FSK_FDEV                                    25000     // Hz
    #define FSK_DATARATE                                19200     // bps
    #define FSK_BANDWIDTH                               50000     // Hz
    #define FSK_AFC_BANDWIDTH                           83333     // Hz
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_FIX_LENGTH_PAYLOAD_ON                   false
    #define FSK_CRC_ENABLED                             true

#else
    #error "Please define a modem in the compiler options."
#endif

#define RX_TIMEOUT_VALUE                                3500000   // in us
#define BUFFER_SIZE                                     32        // Define the payload size here

#if( defined ( TARGET_KL25Z ) || defined ( TARGET_LPC11U6X ) )
DigitalOut led(LED2);
#else
DigitalOut led(LED1);
#endif

/*
 *  Global variables declarations
 */
typedef enum
{
    LOWPOWER = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
}AppStates_t;

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
SX1276MB1xAS Radio( NULL );

const uint8_t PingMsg[] = "PING"; // da pulire
const uint8_t PongMsg[] = "PONG";

char Messaggio[32];

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

// ======================= end LoRa settings ========================


/* Instantiate the expansion board */
static X_NUCLEO_IKS01A1 *mems_expansion_board = X_NUCLEO_IKS01A1::Instance(D14, D15);

/* Retrieve the composing elements of the expansion board */
static GyroSensor *gyroscope = mems_expansion_board->GetGyroscope();
static MotionSensor *accelerometer = mems_expansion_board->GetAccelerometer();
static MagneticSensor *magnetometer = mems_expansion_board->magnetometer;
static HumiditySensor *humidity_sensor = mems_expansion_board->ht_sensor;
static PressureSensor *pressure_sensor = mems_expansion_board->pt_sensor;
static TempSensor *temp_sensor1 = mems_expansion_board->ht_sensor;
static TempSensor *temp_sensor2 = mems_expansion_board->pt_sensor;

/* Helper function for printing floats & doubles */
static char *printDouble(char* str, double v, int decimalDigits=2)
{
  int i = 1;
  int intPart, fractPart;
  int len;
  char *ptr;

  /* prepare decimal digits multiplicator */
  for (;decimalDigits!=0; i*=10, decimalDigits--);

  /* calculate integer & fractinal parts */
  intPart = (int)v;
  fractPart = abs((int)((v-(double)(int)v)*i));

  /* fill in integer part */
  sprintf(str, "%i.", intPart);

  /* prepare fill in of fractional part */
  len = strlen(str);
  ptr = &str[len];

  /* fill in leading fractional zeros */
  for (i/=10;i>1; i/=10, ptr++) {
    if(fractPart >= i) break;
    *ptr = '0';
  }

  /* fill in (rest of) fractional part */
  sprintf(ptr, "%i", fractPart);

  return str;
}


/* Simple main function */
int main()
{
// ======================= LoRa init ================================
//    bool isMaster = true;
//    debug( "\n\n\r     SX1276 Ping Pong Demo Application \n\n\r" );

    // Initialize Radio driver
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.RxError = OnRxError;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	Radio.Init( &RadioEvents );

	// verify the connection with the board
	while( Radio.Read( REG_VERSION ) == 0x00  )
	{
		printf( "Radio could not be detected!\n\r");
		wait( 1 );
	}

//    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1LAS ) ) , "\n\r > Board Type: SX1276MB1LAS < \n\r" );
//    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1MAS ) ) , "\n\r > Board Type: SX1276MB1MAS < \n\r" );

    Radio.SetChannel( RF_FREQUENCY );

#if USE_MODEM_LORA == 1

//    debug_if( LORA_FHSS_ENABLED, "\n\n\r             > LORA FHSS Mode < \n\n\r");
//    debug_if( !LORA_FHSS_ENABLED, "\n\n\r             > LORA Mode < \n\n\r");

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, 2000000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                         LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                         LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, true );

#elif USE_MODEM_FSK == 1

    debug("\n\n\r              > FSK Mode < \n\n\r");
    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                         FSK_DATARATE, 0,
                         FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                         FSK_CRC_ENABLED, 0, 0, 0, 2000000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                         0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                         0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, FSK_CRC_ENABLED,
                         0, 0, false, true );

#else

#error "Please define a modem in the compiler options."

#endif

//    debug_if( DEBUG_MESSAGE, "Starting Ping-Pong loop\r\n" );
//    led = 0;
    Radio.Rx( RX_TIMEOUT_VALUE );



// ==================================================================
	int i;
	uint8_t id;
	float value1, value2;
	char buffer1[32], buffer2[32];
	int32_t axes[3];

	printf("\r\n--- Starting new run ---\r\n");

	humidity_sensor->read_id(&id);
	printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
	pressure_sensor->read_id(&id);
	printf("LPS25H  pressure & temperature    = 0x%X\r\n", id);
	magnetometer->read_id(&id);
	printf("LIS3MDL magnetometer              = 0x%X\r\n", id);
	gyroscope->read_id(&id);
	printf("LSM6DS0 accelerometer & gyroscope = 0x%X\r\n", id);

	wait(3);
	char zip1[5], zip2[5], zip3[7];
 
	while(1)
	{
		printf("\r\n");

		temp_sensor1->get_temperature(&value1);
		humidity_sensor->get_humidity(&value2);
		printf("HTS221: [temp] %7s°C, [hum] %s%%\r\n", printDouble(buffer1, value1), printDouble(buffer2, value2));
		printDouble(zip1, value1);
		printDouble(zip2, value2);

		temp_sensor2->get_fahrenheit(&value1);
		pressure_sensor->get_pressure(&value2);
		printf("LPS25H: [temp] %7s°F, [press] %smbar\r\n", printDouble(buffer1, value1), printDouble(buffer2, value2));
		printDouble(zip3, value2);
		printf("---\r\n");

		magnetometer->get_m_axes(axes);
		printf("LIS3MDL [mag/mgauss]:  %7ld, %7ld, %7ld\r\n", axes[0], axes[1], axes[2]);

		accelerometer->get_x_axes(axes);
		printf("LSM6DS0 [acc/mg]:      %7ld, %7ld, %7ld\r\n", axes[0], axes[1], axes[2]);

		gyroscope->get_g_axes(axes);
		printf("LSM6DS0 [gyro/mdps]:   %7ld, %7ld, %7ld\r\n", axes[0], axes[1], axes[2]);

		wait(1.5);

		// ===================== LoRa output ============================
		//    led = 1;
		sprintf((char*)Messaggio, "%5s, %5s, %7s", zip1, zip2, zip3);
		strcpy( ( char* )Buffer, ( char* )Messaggio );
		for( i = sizeof(Messaggio); i < BufferSize; i++ )
		{
			Buffer[i] = (uint8_t)'o';
		}
		strcpy( ( char* )Buffer, ( char* )Messaggio );
		wait_ms( 10 );
		Radio.Send(Buffer, BufferSize);
		/*
		sprintf((char*)Messaggio, "P=%7s mbar", zip3);
		strcpy( ( char* )Buffer, ( char* )Messaggio );
		wait_ms( 10 );
		Radio.Send(Buffer, BufferSize);
		*/
		//    led = 0;
		State = LOWPOWER;
		wait_ms( 5000 );

		// ==============================================================

	}
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
//    debug_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
//    debug_if( DEBUG_MESSAGE, "> OnRxDone\n\r" );
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
//    debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    Buffer[ BufferSize ] = 0;
    State = RX_TIMEOUT;
//    debug_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
//    debug_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
}


