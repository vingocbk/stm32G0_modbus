#include <Arduino.h> 
#include <MCP41_Simple.h>

#include <SPI.h>
#include <ModbusRtu.h>

#define PIN_SPI_MOSI_DEFINE   PA7
#define PIN_SPI_MISO_DEFINE   PA6
#define PIN_SPI_SCK_DEFINE    PA5

#define PIN_RX    (PB7)
#define PIN_TX    (PB6)

// Which pin is connected to CS
const uint8_t  CS_PIN_1      = PA12;
const uint8_t  CS_PIN_2      = PA11;

SPIClass spiClass = SPIClass(PIN_SPI_MOSI_DEFINE, PIN_SPI_MISO_DEFINE, PIN_SPI_SCK_DEFINE);
SPIClass spiClass2 = SPIClass(PIN_SPI_MOSI_DEFINE, PIN_SPI_MISO_DEFINE, PIN_SPI_SCK_DEFINE);
MCP41_Simple MyPot1 = MCP41_Simple(spiClass);
MCP41_Simple MyPot2 = MCP41_Simple(spiClass2);
// MCP41_Simple MyPot1, MyPot2;
#define MAX_VALUE_RES       1200
#define DEVICE_ID           1
#define BAUD_RATE           9600
#define MAX_SIZE_REGISTER   16

// data array for modbus network sharing
uint16_t au16data[MAX_SIZE_REGISTER] = {1, 2};
uint16_t inCount = 0, preInCount = 0; 


Modbus slave(1,Serial,0); // this is slave @1 and RS-232 or USB-FTDI
void setup() {
  Serial.setRx(PIN_RX);
  Serial.setTx(PIN_TX);
  Serial.begin(BAUD_RATE); /* generic variant: TX = PA2, RX = PA3 */
  Serial.setRx(PIN_RX);
  Serial.setTx(PIN_TX);
   // put your setup code here, to run once:
  MyPot1.begin(CS_PIN_1);
  MyPot2.begin(CS_PIN_2);
  slave.start();
  
}

void loop() {
  slave.poll( au16data, MAX_SIZE_REGISTER );
  inCount = slave.getInCnt();
  if(inCount != preInCount){
    preInCount = inCount;
    for(int i = 0; i < 10; i++){
    float value_1 = (float)au16data[0]/1200.0*255.0;
    float value_2 = (float)au16data[1]/1200.0*255.0;
    MyPot1.setWiper(uint8_t(value_1));
    MyPot2.setWiper(uint8_t(value_2));
  }
  }
}


/* addition for G0 board: instead of generic reset clock (8MHz), use HSI+PLL to 64Mhz */
#if defined(ARDUINO_GENERIC_G030F6PX)
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

/* addition for F0 board: instead of generic reset clock (8MHz), use HSI+PLL to 48Mhz */
#if defined(ARDUINO_GENERIC_F031F6PX)
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif 