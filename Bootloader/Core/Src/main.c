/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal_crc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define APPLICATION_ADDRESS 0x08008000  // Application start address in Flash
#define BUFFER_SIZE 1024  // Reception buffer size
#define MAX_BUFFER_SIZE 1024 // Maximum segment size

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;  // UART handle for communication
CRC_HandleTypeDef hcrc;     // CRC handle for CRC calculation

/* USER CODE BEGIN PV */
FLASH_EraseInitTypeDef EraseInitStruct;  // Flash erase structure
uint32_t PAGEError;  // Page erase error variable

uint8_t fileData[MAX_BUFFER_SIZE]; // Buffer for binary file data
uint32_t receivedFileSize; // Received file size
uint32_t receivedCRC; // Received CRC
uint32_t calculatedCRC; // Calculated CRC
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
void JumpToApplication(void);
void ReceiveAndFlashFirmware(void);
void processReceivedData(void);
void processReceivedDataSegment(uint8_t *data, uint32_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Function to receive and flash firmware */
void ReceiveAndFlashFirmware(void) {
    uint8_t buffer[BUFFER_SIZE];  // Buffer for receiving data
    uint32_t* applicationAddress = (uint32_t*)APPLICATION_ADDRESS;  // Destination address for writing

    // Configure and erase the Flash sector
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t sectorError;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.Sector = FLASH_SECTOR_2;  // Adjust based on address
    eraseInitStruct.NbSectors = 6;  // Adjust based on file size
    eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();  // Unlock Flash access
    HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError);  // Erase sector

    size_t remainingBytes = receivedFileSize;  // Number of bytes left to receive
    while (remainingBytes > 0) {
        size_t bytesToReceive = remainingBytes < BUFFER_SIZE ? remainingBytes : BUFFER_SIZE;  // Determine the number of bytes to receive
        HAL_UART_Receive(&huart1, buffer, bytesToReceive, HAL_MAX_DELAY);  // Receive data

        // Write data to Flash
        for (size_t i = 0; i < bytesToReceive; i += 4) {
            uint32_t dataToWrite = *(uint32_t*)(buffer + i);  // Read 4 bytes at a time
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(applicationAddress + i / 4), dataToWrite);  // Write to Flash
        }

        remainingBytes -= bytesToReceive;  // Reduce the number of remaining bytes
    }

    HAL_FLASH_Lock();  // Lock Flash access
}

/* Function to jump to the application */
void JumpToApplication(void) {
    uint32_t jumpAddress = *(__IO uint32_t*)(APPLICATION_ADDRESS + 4);  // Read the application start address
    void (*resetHandler)(void) = (void (*)(void))jumpAddress;  // Get the pointer to the reset handler

    __set_MSP(*(__IO uint32_t*)APPLICATION_ADDRESS);  // Initialize stack pointer
    resetHandler();  // Call the reset handler
}

/* Function to process received data segments */
void processReceivedDataSegment(uint8_t *data, uint32_t length) {
    // Calculate CRC on the received data
    calculatedCRC = HAL_CRC_Accumulate(&hcrc, (uint32_t*)data, length / 4);
    // If the data length is not a multiple of 4, process the remaining bytes
    uint32_t remainingBytes = length % 4;
    if (remainingBytes > 0) {
        uint32_t remainingData = 0;
        memcpy(&remainingData, data + length - remainingBytes, remainingBytes);
        calculatedCRC = HAL_CRC_Accumulate(&hcrc, &remainingData, 1);
    }
}

/* Function to process received data */
void processReceivedData(void) {
    uint32_t bytesRemaining = receivedFileSize;
    uint32_t bytesToReceive;

    // Reset CRC before starting
    __HAL_CRC_DR_RESET(&hcrc);

    while (bytesRemaining > 0) {
        bytesToReceive = (bytesRemaining > sizeof(fileData)) ? sizeof(fileData) : bytesRemaining;

        // Receive a segment of the file
        if (HAL_UART_Receive(&huart1, fileData, bytesToReceive, HAL_MAX_DELAY) != HAL_OK) {
            // Handle reception errors
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // LED12 on error
            return;
        }

        // Process the received segment
        processReceivedDataSegment(fileData, bytesToReceive);

        bytesRemaining -= bytesToReceive;
    }

    // Compare the calculated CRC with the received CRC
    if (calculatedCRC == receivedCRC) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // LED13 on
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // LED12 off
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // LED13 off
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // LED12 on
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
  // Receive the file size (4 bytes)
  HAL_UART_Receive(&huart1, (uint8_t*)&receivedFileSize, sizeof(receivedFileSize), HAL_MAX_DELAY);

  // Receive the CRC (4 bytes for CRC-32)
  HAL_UART_Receive(&huart1, (uint8_t*)&receivedCRC, sizeof(receivedCRC), HAL_MAX_DELAY);

  // Process the received data in segments
  processReceivedData();

  // Flash the firmware
  ReceiveAndFlashFirmware();

  // Jump to the updated application
  JumpToApplication();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Optional: Add any background tasks here
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{
  /* USER CODE BEGIN CRC_Init 0 */
  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */
  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;

  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */
  /* USER CODE END CRC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void Error_Handler(void)
{
  while(1) {
    // Add error handling code here
  }
}
/* USER CODE END 4 */
