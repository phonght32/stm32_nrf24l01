/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "nrf24l01.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NRF24L01_TX_SPI                 &hspi1
#define NRF24L01_TX_SPI_CS_PORT         GPIOB
#define NRF24L01_TX_SPI_CS_PIN_NUM      GPIO_PIN_7
#define NRF24L01_TX_SPI_CE_PORT         GPIOB
#define NRF24L01_TX_SPI_CE_PIN_NUM      GPIO_PIN_6
#define NRF24L01_TX_IRQ_PORT            GPIOB
#define NRF24L01_TX_IRQ_PIN_NUM         GPIO_PIN_8

#define NRF24L01_RX_SPI                 &hspi2
#define NRF24L01_RX_SPI_CS_PORT         GPIOC
#define NRF24L01_RX_SPI_CS_PIN_NUM      GPIO_PIN_4
#define NRF24L01_RX_SPI_CE_PORT         GPIOC
#define NRF24L01_RX_SPI_CE_PIN_NUM      GPIO_PIN_5
#define NRF24L01_RX_IRQ_PORT            GPIOA
#define NRF24L01_RX_IRQ_PIN_NUM         GPIO_PIN_6

#define HW_SERIAL_LOG_UART_HANDLE       huart4
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
nrf24l01_handle_t nrf24l01_tx_handle;
nrf24l01_handle_t nrf24l01_rx_handle;
uint8_t tx_data[8] = {'0', '0', '0', '0', '0', '0', '0', '0'};
uint8_t rx_data[8] = {'0', '0', '0', '0', '0', '0', '0', '0'};
uint8_t log_buf[50];
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
err_code_t hw_intf_nrf24l01_tx_spi_send(uint8_t *buf_send, uint16_t len, uint32_t timeout_ms);
err_code_t hw_intf_nrf24l01_tx_spi_recv(uint8_t *buf_recv, uint16_t len, uint32_t timeout_ms);
err_code_t hw_intf_nrf24l01_tx_set_cs(uint8_t level);
err_code_t hw_intf_nrf24l01_tx_set_ce(uint8_t level);

err_code_t hw_intf_nrf24l01_rx_spi_send(uint8_t *buf_send, uint16_t len, uint32_t timeout_ms);
err_code_t hw_intf_nrf24l01_rx_spi_recv(uint8_t *buf_recv, uint16_t len, uint32_t timeout_ms);
err_code_t hw_intf_nrf24l01_rx_set_cs(uint8_t level);
err_code_t hw_intf_nrf24l01_rx_set_ce(uint8_t level);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_UART4_Init();
    /* USER CODE BEGIN 2 */
    nrf24l01_tx_handle = nrf24l01_init();
    nrf24l01_cfg_t nrf24l01_tx_cfg = {
        .channel = 2500,
		.payload_len = 8,
        .crc_len = 1,
        .addr_width = 5,
        .retrans_cnt = 3,
        .retrans_delay = 250,
        .data_rate = NRF24L01_DATA_RATE_1Mbps,
        .output_pwr = NRF24L01_OUTPUT_PWR_0dBm,
        .mode = NRF24L01_MODE_TRANSMITTER,
        .spi_send = hw_intf_nrf24l01_tx_spi_send,
        .spi_recv = hw_intf_nrf24l01_tx_spi_recv,
        .set_cs = hw_intf_nrf24l01_tx_set_cs,
        .set_ce = hw_intf_nrf24l01_tx_set_ce
    };
    nrf24l01_set_config(nrf24l01_tx_handle, nrf24l01_tx_cfg);
    nrf24l01_config(nrf24l01_tx_handle);

    nrf24l01_rx_handle = nrf24l01_init();
    nrf24l01_cfg_t nrf24l01_rx_cfg = {
        .channel = 2500,
		.payload_len = 8,
        .crc_len = 1,
        .addr_width = 5,
        .retrans_cnt = 3,
        .retrans_delay = 250,
        .data_rate = NRF24L01_DATA_RATE_1Mbps,
        .output_pwr = NRF24L01_OUTPUT_PWR_0dBm,
        .mode = NRF24L01_MODE_RECEIVER,
        .spi_send = hw_intf_nrf24l01_rx_spi_send,
        .spi_recv = hw_intf_nrf24l01_rx_spi_recv,
        .set_cs = hw_intf_nrf24l01_rx_set_cs,
        .set_ce = hw_intf_nrf24l01_rx_set_ce
    };
    nrf24l01_set_config(nrf24l01_rx_handle, nrf24l01_rx_cfg);
    nrf24l01_config(nrf24l01_rx_handle);
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        static uint8_t cnt = 0;

        if (cnt > 8)
        {
            cnt = 0;
            for (int i = 0; i < 8; i++)
            {
                tx_data[i] = '0';
            }
        }

        for (int i = 0; i < 8; i++)
        {
            tx_data[i]++;
        }

        nrf24l01_transmit(nrf24l01_tx_handle, tx_data);

        cnt++;

        HAL_Delay(100);
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
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
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}
/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == NRF24L01_RX_IRQ_PIN_NUM)
    {
        nrf24l01_receive(nrf24l01_rx_handle, rx_data); // read data when data ready flag is set

        sprintf((char*)log_buf, (const char*)"\r\nTransmitted data: %s", tx_data);
        HAL_UART_Transmit(&HW_SERIAL_LOG_UART_HANDLE, (uint8_t*)log_buf, 28, 100);

        sprintf((char*)log_buf, (const char*)"\r\nReceived data   : %s", rx_data);
        HAL_UART_Transmit(&HW_SERIAL_LOG_UART_HANDLE, (uint8_t*)log_buf, 28, 100);

        sprintf((char*)log_buf, (const char*)"\r\n**************************");
        HAL_UART_Transmit(&HW_SERIAL_LOG_UART_HANDLE, (uint8_t*)log_buf, 28, 100);
    }
    else if (GPIO_Pin == NRF24L01_TX_IRQ_PIN_NUM)
    {
        nrf24l01_transmit_irq(nrf24l01_tx_handle);
    }
    else
    {

    }
}

err_code_t hw_intf_nrf24l01_tx_spi_send(uint8_t *buf_send, uint16_t len, uint32_t timeout_ms)
{
    HAL_SPI_Transmit(NRF24L01_TX_SPI, buf_send, len, timeout_ms);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_tx_spi_recv(uint8_t *buf_recv, uint16_t len, uint32_t timeout_ms)
{
    HAL_SPI_Receive(NRF24L01_TX_SPI, buf_recv, len, timeout_ms);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_tx_set_cs(uint8_t level)
{
    HAL_GPIO_WritePin(NRF24L01_TX_SPI_CS_PORT, NRF24L01_TX_SPI_CS_PIN_NUM, level);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_tx_set_ce(uint8_t level)
{
    HAL_GPIO_WritePin(NRF24L01_TX_SPI_CE_PORT, NRF24L01_TX_SPI_CE_PIN_NUM, level);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_rx_spi_send(uint8_t *buf_send, uint16_t len, uint32_t timeout_ms)
{
    HAL_SPI_Transmit(NRF24L01_RX_SPI, buf_send, len, timeout_ms);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_rx_spi_recv(uint8_t *buf_recv, uint16_t len, uint32_t timeout_ms)
{
    HAL_SPI_Receive(NRF24L01_RX_SPI, buf_recv, len, timeout_ms);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_rx_set_cs(uint8_t level)
{
    HAL_GPIO_WritePin(NRF24L01_RX_SPI_CS_PORT, NRF24L01_RX_SPI_CS_PIN_NUM, level);

    return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_nrf24l01_rx_set_ce(uint8_t level)
{
    HAL_GPIO_WritePin(NRF24L01_RX_SPI_CE_PORT, NRF24L01_RX_SPI_CE_PIN_NUM, level);

    return ERR_CODE_SUCCESS;
}
/* USER CODE END 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
