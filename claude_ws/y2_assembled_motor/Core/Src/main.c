/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - 电机控制与串口通信
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
// 板ID定义
#define BOARD_ID_1     1
#define BOARD_ID_2     2

// 配置当前板子的ID
#define THIS_BOARD_ID  BOARD_ID_1

// 心跳超时时间 (ms)
#define HEARTBEAT_TIMEOUT_MS   500

// 通信协议定义
#define PKT_HEADER_1       0xAA
#define PKT_HEADER_2       0x55
#define PKT_TYPE_HEARTBEAT 0x01
#define PKT_TYPE_CONTROL   0x02

// 状态定义
#define STATE_OFF     0  // 停止
#define STATE_FORWARD 1  // 正转
#define STATE_REVERSE 2  // 反转

// 按键消抖时间 (ms)
#define KEY_DEBOUNCE_MS    20

/* Private macro ------------------------------------------------------------*/
#define LOG_INFO(...) do { \
    char _buf[128]; \
    snprintf(_buf, sizeof(_buf), __VA_ARGS__); \
    HAL_UART_Transmit(&huart1, (uint8_t*)_buf, strlen(_buf), 0xFFFF); \
} while(0)

/* Private variables ---------------------------------------------------------*/

// 串口接收缓冲区
#define RX_BUF_SIZE    32
static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t rx_index = 0;

// 发送缓冲区
static uint8_t tx_buf[24];

// 电机状态: 0=停止, 1=正转, 2=反转
static uint8_t motor_state = STATE_OFF;

// 电机方向: 0=正转, 1=反转
static uint8_t motor_direction = 0;

// 对端连接状态
static uint8_t remote_connected = 0;
static uint32_t last_heartbeat_tick = 0;

// 反转按键状态 (PA3)
static uint8_t last_reverse_key = 1;  // 默认高电平(未按下)
static uint8_t reverse_key_pressed = 0;
static uint32_t last_reverse_key_time = 0;

// PB0 状态
static uint8_t last_pb0 = 1;
static uint8_t pb0_pressed = 0;
static uint32_t last_pb0_time = 0;

// PD1 状态
static uint8_t last_pd1 = 1;
static uint8_t pd1_pressed = 0;
static uint32_t last_pd1_time = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Protocol_Init(void);
void Protocol_SendHeartbeat(void);
void Protocol_ProcessPacket(uint8_t *data, uint8_t len);
uint8_t Protocol_CalcChecksum(uint8_t *data, uint8_t len);
void Motor_SetDirection(uint8_t direction);
void Motor_Stop(void);

/* Private user code ---------------------------------------------------------*/

// 设置PWM占空比
void Set_PWM_Duty(uint16_t duty)
{
    if(duty > 65535) duty = 65535;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

// 电机设置方向并启动
void Motor_SetDirection(uint8_t direction)
{
    motor_direction = direction;
    motor_state = (direction == 0) ? STATE_FORWARD : STATE_REVERSE;
    
    // 使能电机
    HAL_GPIO_WritePin(GPIOB, DRV_SLEEP_Pin, GPIO_PIN_SET);
    Set_PWM_Duty(6000);
    
    // 根据方向设置电机转向
    if(motor_direction == 0) {
        HAL_GPIO_WritePin(GPIOB, MOTOR_PH_Pin, GPIO_PIN_SET);  // 正转
        LOG_INFO("[MOTOR] Forward\r\n");
    } else {
        HAL_GPIO_WritePin(GPIOB, MOTOR_PH_Pin, GPIO_PIN_RESET); // 反转
        LOG_INFO("[MOTOR] Reverse\r\n");
    }
    
    Set_PWM_Duty(6000);
}

// 电机停止
void Motor_Stop(void)
{
    motor_state = STATE_OFF;
    HAL_GPIO_WritePin(GPIOB, DRV_SLEEP_Pin, GPIO_PIN_RESET);
    Set_PWM_Duty(0);
    LOG_INFO("[MOTOR] Stopped\r\n");
}

// 初始化
void Protocol_Init(void)
{
    motor_state = STATE_OFF;
    remote_connected = 0;
    last_heartbeat_tick = 0;
}

// 计算校验和
uint8_t Protocol_CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for(uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

// 发送心跳包
void Protocol_SendHeartbeat(void)
{
    uint8_t pkt_index = 0;
    uint8_t data_len = 2;
    
    tx_buf[pkt_index++] = PKT_HEADER_1;
    tx_buf[pkt_index++] = PKT_HEADER_2;
    tx_buf[pkt_index++] = data_len;
    tx_buf[pkt_index++] = PKT_TYPE_HEARTBEAT;
    tx_buf[pkt_index++] = THIS_BOARD_ID;
    tx_buf[pkt_index++] = motor_state;
    tx_buf[pkt_index++] = Protocol_CalcChecksum(tx_buf + 3, data_len);
    
    HAL_UART_Transmit(&huart1, tx_buf, pkt_index, 0xFFFF);
}

// 处理接收到的数据包
void Protocol_ProcessPacket(uint8_t *data, uint8_t len)
{
    if(len < 2) return;
    
    uint8_t pkt_type = data[0];
    
    if(pkt_type == PKT_TYPE_HEARTBEAT) {
        uint8_t remote_board_id = data[1];
        last_heartbeat_tick = HAL_GetTick();
        
        if(remote_connected == 0) {
            remote_connected = 1;
            LOG_INFO("[LINK] Board %d connected\r\n", remote_board_id);
        }
    }
    else if(pkt_type == PKT_TYPE_CONTROL) {
        uint8_t cmd = data[1];
        if(cmd == STATE_OFF) {
            Motor_Stop();
        } else if(cmd == STATE_FORWARD) {
            Motor_SetDirection(0);
        } else if(cmd == STATE_REVERSE) {
            Motor_SetDirection(1);
        }
        LOG_INFO("[CTRL] State: %d\r\n", cmd);
    }
}

// 串口接收中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        uint8_t ch = rx_buf[rx_index];
        
        if(rx_index == 0 && ch != PKT_HEADER_1) {
            HAL_UART_Receive_IT(&huart1, &rx_buf[0], 1);
            return;
        }
        if(rx_index == 1 && ch != PKT_HEADER_2) {
            rx_index = 0;
            HAL_UART_Receive_IT(&huart1, &rx_buf[0], 1);
            return;
        }
        
        rx_index++;
        
        if(rx_index >= 4) {
            uint8_t data_len = rx_buf[2];
            if(rx_index >= data_len + 5) {
                uint8_t checksum = Protocol_CalcChecksum(rx_buf + 3, data_len);
                if(checksum == rx_buf[3 + data_len]) {
                    Protocol_ProcessPacket(&rx_buf[3], data_len);
                }
                rx_index = 0;
            }
        }
        
        if(rx_index >= RX_BUF_SIZE) rx_index = 0;
        
        HAL_UART_Receive_IT(&huart1, &rx_buf[rx_index], 1);
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  Protocol_Init();
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  
  // 启动PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  Set_PWM_Duty(0);
  
  // 初始化后电机保持静止
  Motor_Stop();
  
  // 启动串口接收
  HAL_UART_Receive_IT(&huart1, &rx_buf[0], 1);
  
  // 初始化按键状态
  last_reverse_key = HAL_GPIO_ReadPin(REVERSE_BTN_GPIO_Port, REVERSE_BTN_Pin);
  last_pb0 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
  last_pd1 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);

  /* Infinite loop */
  uint32_t last_heartbeat_send = 0;
  uint32_t last_connection_check = 0;
  
  while (1)
  {
    uint32_t currentTick = HAL_GetTick();
    
    // 定时发送心跳包 (100ms)
    /*if (currentTick - last_heartbeat_send >= 100) {
        last_heartbeat_send = currentTick;
        Protocol_SendHeartbeat();
    }
    
    // 检查连接状态 (500ms)
    if (currentTick - last_connection_check >= 500) {
        last_connection_check = currentTick;
        
        if(remote_connected && (currentTick - last_heartbeat_tick > HEARTBEAT_TIMEOUT_MS)) {
            remote_connected = 0;
            LOG_INFO("[LINK] Connection lost\r\n");
        }
    }*/
    
    // ========== 按键处理 (PA3 反转按键) ==========
    {
        uint8_t current_key = HAL_GPIO_ReadPin(REVERSE_BTN_GPIO_Port, REVERSE_BTN_Pin);
        
        // 检测下降沿
        if(current_key == 0 && last_reverse_key == 1 && !reverse_key_pressed) {
            last_reverse_key_time = currentTick;
            reverse_key_pressed = 1;
        }
        
        // 消抖判断
        if(reverse_key_pressed && (currentTick - last_reverse_key_time) >= KEY_DEBOUNCE_MS) {
            // 第一次按下或停止后按下：根据当前方向决定下一步
            if(motor_state == STATE_OFF) {
                // 停止状态：根据上次方向决定（由PB0/PD1设置）
                Motor_SetDirection(motor_direction);
            } else {
                // 运行中：反转
                motor_direction ^= 1;
                Motor_SetDirection(motor_direction);
            }
            reverse_key_pressed = 0;
        }
        
        // 按键释放
        if(current_key == 1) {
            reverse_key_pressed = 0;
        }
        
        last_reverse_key = current_key;
    }
    
    // ========== PB0 处理 (停止后反转) ==========
    {
        uint8_t current_pb0 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
        
        if(current_pb0 == 0 && last_pb0 == 1 && !pb0_pressed) {
            last_pb0_time = currentTick;
            pb0_pressed = 1;
        }
        
        if(pb0_pressed && (currentTick - last_pb0_time) >= KEY_DEBOUNCE_MS) {
            // PB0 低电平：停止电机，设置下次反转方向
            if(motor_state != STATE_OFF) {
                Motor_Stop();
                motor_direction = 1;  // 下次按键反转
                LOG_INFO("[PB0] Stop, next will reverse\r\n");
            }
            pb0_pressed = 0;
        }
        
        if(current_pb0 == 1) {
            pb0_pressed = 0;
        }
        
        last_pb0 = current_pb0;
    }
    
    // ========== PD1 处理 (停止后正转) ==========
    {
        uint8_t current_pd1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
        
        if(current_pd1 == 0 && last_pd1 == 1 && !pd1_pressed) {
            last_pd1_time = currentTick;
            pd1_pressed = 1;
        }
        
        if(pd1_pressed && (currentTick - last_pd1_time) >= KEY_DEBOUNCE_MS) {
            // PD1 低电平：停止电机，设置下次正转方向
            if(motor_state != STATE_OFF) {
                Motor_Stop();
                motor_direction = 0;  // 下次按键正转
                LOG_INFO("[PB1] Stop, next will forward\r\n");
            }
            pd1_pressed = 0;
        }
        
        if(current_pd1 == 1) {
            pd1_pressed = 0;
        }
        
        last_pd1 = current_pd1;
    }
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
  
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
