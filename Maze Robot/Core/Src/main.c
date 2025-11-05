/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file               : main.c
  * @brief              : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensors.h"  
#include <stdlib.h>   
#include <stdbool.h>  
#include <math.h>     
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Cau truc de luu tru trang thai cua mot bo dieu khien PID
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
} PID_Controller;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// =================================================================
// CAC THAM SO CO THE TINH CHINH
// =================================================================

// --- Toc do Motor (Gia tri PWM, 0-9999) ---
#define MAX_SPEED               9999 // Toc do toi da
#define BASE_SPEED              7999 // Toc do co ban khi chay thang
#define BASE_SPEED_TURN         6999 // Toc do khi re 

// --- Nguong cam bien (don vi: mm) ---
#define WALL_THRESHOLD          110  // Khoang cach toi da cam bien hai ben thay tuong
#define WALL_THRESHOLD_FRONT    90  // Khoang cach toi da cam bien truoc thay tuong
#define STRAIGHT_ERROR_DEADZONE 4    // Vung loi chap nhan khi di thang (+-4mm), loi nho hon se bi bo qua

// --- Hieu chinh (Offset) cho tung cam bien ---
#define SENSOR_FRONT_OFFSET     0
#define SENSOR_LEFT_OFFSET      -9
#define SENSOR_RIGHT_OFFSET     0

// --- Muc tieu Encoder ---
#define ENCODER_TARGET_TURN_90      2100  // So xung de re 90 do
#define ENCODER_CENTER_OFFSET       1600   // So xung tien them de can giua o truoc khi re
#define ENCODER_REVERSE_OFFSET_180  1000   // So xung lui lai khi quay 180 do

// --- Tham so PID cho viec DI THANG ---
#define KP_STRAIGHT 15.3f
#define KI_STRAIGHT 0.5f
#define KD_STRAIGHT 12.5f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Bien PID toan cuc cho viec di thang
static PID_Controller pid_straight;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// =================================================================
// KHAI BAO CAC HAM DIEU KHIEN ROBOT 
// =================================================================

void robot_init(void);
void stop_motors(void);
void turn_left_90(void);
void turn_right_90(void);
void turn_180(void);
void move_forward_pulses(int32_t pulses);
void move_backward_pulses(int32_t pulses);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// =================================================================
// CAC HAM DIEU KHIEN CAP THAP
// =================================================================

/**
 * @brief Dat toc do va chieu quay cho dong co trai.
 * @param speed: Toc do (-MAX_SPEED den MAX_SPEED). > 0 la tien, < 0 la lui.
 */
static void set_left_motor_speed(int16_t speed) {
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < -MAX_SPEED) speed = -MAX_SPEED;

    if (speed > 0) { // Quay tien
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
    } else if (speed < 0) { // Quay lui
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
    } else { // Dung
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}

/**
 * @brief Dat toc do va chieu quay cho dong co phai.
 */
static void set_right_motor_speed(int16_t speed) {
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < -MAX_SPEED) speed = -MAX_SPEED;

    if (speed > 0) { // Quay tien
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    } else if (speed < 0) { // Quay lui
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -speed);
    } else { // Dung
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

/**
 * @brief Dat lai gia tri cua ca hai bo dem encoder ve 0.
 */
static void reset_encoders(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

/**
 * @brief Doc gia tri tu encoder banh trai.
 */
static int16_t get_left_encoder_val(void) {
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief Doc gia tri tu encoder banh phai.
 * @note Phan cung da duoc noi day nguoc de ca hai encoder cung dem tien khi robot di thang.
 */
static int16_t get_right_encoder_val(void) {
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

/**
 * @brief Tinh toan gia tri dau ra cho bo dieu khien PID.
 */
static float pid_compute(PID_Controller* pid, float error) {
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

/**
 * @brief Dat lai trang thai cua bo dieu khien PID.
 */
static void pid_reset(PID_Controller* pid) {
    pid->integral = 0;
    pid->prev_error = 0;
}


// =================================================================
// CAC HAM CHUC NANG CHINH CUA ROBOT
// =================================================================

/**
 * @brief Phanh cung ca hai dong co (active braking).
 */
void stop_motors(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

/**
 * @brief Khoi tao toan bo he thong robot.
 */
void robot_init(void) {
    sensors_init(); // Khoi dong module cam bien
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    pid_straight.Kp = KP_STRAIGHT;
    pid_straight.Ki = KI_STRAIGHT;
    pid_straight.Kd = KD_STRAIGHT;

    reset_encoders();
    pid_reset(&pid_straight);
    stop_motors();
}

/**
 * @brief Re phai 90 do tai cho (chi dung encoder, khong dung cam bien).
 */
void turn_right_90(void) {
    reset_encoders();
    // Quay cho den khi encoder trai dat muc tieu
    while (abs(get_left_encoder_val()) < ENCODER_TARGET_TURN_90) {
        set_left_motor_speed(BASE_SPEED_TURN);
        set_right_motor_speed(-BASE_SPEED_TURN);
    }
    stop_motors();
}

/**
 * @brief Re trai 90 do tai cho (chi dung encoder, khong dung cam bien).
 */
void turn_left_90(void) {
    reset_encoders();
    // Quay cho den khi encoder phai dat muc tieu
    while (abs(get_right_encoder_val()) < ENCODER_TARGET_TURN_90) {
        set_left_motor_speed(-BASE_SPEED_TURN);
        set_right_motor_speed(BASE_SPEED_TURN);
    }
    stop_motors();
}

/**
 * @brief Quay 180 do tai cho de thoat ngo cut.
 */
void turn_180(void) {
    turn_right_90();
    HAL_Delay(100);
    move_backward_pulses(ENCODER_REVERSE_OFFSET_180); // Lui lai mot chut
    HAL_Delay(100);
    turn_right_90();
}

/**
 * @brief Di chuyen robot ve phia truoc mot so xung encoder nhat dinh.
 * @param pulses: So xung encoder muc tieu.
 */
void move_forward_pulses(int32_t pulses) {
    reset_encoders();
    while ((abs(get_left_encoder_val()) + abs(get_right_encoder_val())) / 2 < pulses) {
        set_left_motor_speed(BASE_SPEED);
        set_right_motor_speed(BASE_SPEED);
    }
    stop_motors();
}

/**
 * @brief Di chuyen robot lui mot so xung encoder nhat dinh.
 * @param pulses: So xung encoder muc tieu.
 */
void move_backward_pulses(int32_t pulses) {
    reset_encoders();
    while ((abs(get_left_encoder_val()) + abs(get_right_encoder_val())) / 2 < pulses) {
        set_left_motor_speed(-BASE_SPEED);
        set_right_motor_speed(-BASE_SPEED);
    }
    stop_motors();
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  robot_init();
  HAL_Delay(1000); // Doi 1 giay de ban dat robot vao me cung

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    // =======================================================================
    // VONG LAP CHINH CUA THUAT TOAN GIAI ME CUNG
    // =======================================================================

    // === BUOC 1: DI THANG LIEN TUC CHO DEN KHI GAP GIAO LO ===

    pid_reset(&pid_straight);

    while(1) {

        sensors_read_all();

        // Kiem tra cac dieu kien de dung lai
        bool front_wall_detected = (get_front_distance() + SENSOR_FRONT_OFFSET < WALL_THRESHOLD_FRONT);
        bool left_opening_detected = (get_left_distance() + SENSOR_LEFT_OFFSET > WALL_THRESHOLD);
        bool right_opening_detected = (get_right_distance() + SENSOR_RIGHT_OFFSET > WALL_THRESHOLD);

        // Dung lai neu co tuong truoc, hoac neu dang o trong hanh lang ma mat tuong
        if (front_wall_detected || 
            (!front_wall_detected && !left_opening_detected && right_opening_detected )) {
            stop_motors();
            break; // Thoat vong lap di thang de bat dau ra quyet dinh
        }

        // Logic PID de di thang bam tuong
        int16_t left_dist = get_left_distance() + SENSOR_LEFT_OFFSET;
        int16_t right_dist = get_right_distance() + SENSOR_RIGHT_OFFSET;
        float error = 0;

        if (left_dist < 1000 && right_dist < 1000) {
            error = left_dist - right_dist;
        }
        if (fabsf(error) <= STRAIGHT_ERROR_DEADZONE) {
            error = 0;
        }

        float correction = pid_compute(&pid_straight, error);
        set_left_motor_speed(BASE_SPEED - correction);
        set_right_motor_speed(BASE_SPEED + correction);
        HAL_Delay(5);
    }

    // === BUOC 2: DUNG LAI VA PHAN TICH GIAO LO ===
    HAL_Delay(500);
    sensors_read_all(); // Doc lai cam bien lan cuoi de co du lieu chinh xac

    bool hasFrontWall = (get_front_distance() + SENSOR_FRONT_OFFSET < WALL_THRESHOLD_FRONT);
    bool hasLeftWall  = (get_left_distance() + SENSOR_LEFT_OFFSET < WALL_THRESHOLD);
    bool hasRightWall = (get_right_distance() + SENSOR_RIGHT_OFFSET < WALL_THRESHOLD);

    // === BUOC 3: RA QUYET DINH DUA TREN CAC QUY TAC BAN DA DAT RA ===
    
    if (hasFrontWall && hasLeftWall && hasRightWall) {
        // Truong hop 1: Ngo cut (3 tuong) -> Quay 180 do
        HAL_Delay(200);
        turn_180();
        HAL_Delay(200);
    }
    else if (hasFrontWall && hasLeftWall && !hasRightWall) {
        // Truong hop 2: Tuong phia truoc va ben trai -> Re phai
        HAL_Delay(100);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
        turn_right_90();
        HAL_Delay(200);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
    }
    else if (hasFrontWall && !hasLeftWall && hasRightWall) {
        // Truong hop 3: Tuong phia truoc va ben phai -> Re trai
        HAL_Delay(100);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
        turn_left_90();
        HAL_Delay(200);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
    }
    else if (hasFrontWall && !hasLeftWall && !hasRightWall) {
         // Truong hop 4: Chi co tuong phia truoc -> Re phai
        HAL_Delay(100);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
        turn_right_90();
        HAL_Delay(200);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
    }
    else if (!hasFrontWall && hasLeftWall && !hasRightWall) {
        // Truong hop 5: Chi co tuong ben trai -> Re phai
        HAL_Delay(100);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
        turn_right_90();
        HAL_Delay(200);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
    }
        else if (!hasFrontWall && !hasLeftWall && hasRightWall) {
        // Truong hop 6: Chi co tuong ben phai -> Re trai
        HAL_Delay(100);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
        turn_left_90();
        HAL_Delay(200);
        move_forward_pulses(ENCODER_CENTER_OFFSET);
        HAL_Delay(200);
    }
    else {
        // Cac truong hop con lai deu la DI THANG:
        // Vong lap se tu dong bat dau lai va robot tiep tuc di thang.
    }

  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint8_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */