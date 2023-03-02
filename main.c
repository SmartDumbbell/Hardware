/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ******************************************************************************
  */
//IMU: https://github.com/talhaSr/mpu9250
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt_stm32_delay.h"
#include "MPU9250.h"//nextqure
#include <math.h>
#include "mpu9255.h"//LAB


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI   3.14159265359f

#define READTIME 1
#define AVERAGE 100
#define MEDIAN 51

#define IMUAVERAGE 50//x

#define ADDR_FLASH_PAGE_127 ((uint32_t)0x803F800) //2 Kbytes
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU9250_t mpu9250; //existing
MPU9255_t MPU9255; //modify
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int readData();
uint16_t movingAverage(uint16_t rawdata);
void loadCellInit();
uint16_t median(uint16_t averagedata);

void Calibration();//x

void append(char *dst, char c);
HAL_StatusTypeDef whoAmI_Check(MPU9250_t *mpu9250);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE Test BEGIN*/

/* USER CODE Test End*/


volatile uint16_t timer_ms=0, timer_s=0, timer_ss=0; //1s = 1000ms
uint8_t transmitBuf[255] = {0,};
uint8_t testBuf[1000] = {0,};
uint8_t statusFlag = 0;
uint8_t powerFlag = 0;
volatile uint8_t startFlag = 0; //1: handgrip, 2: exercise
uint8_t ledFlag = 1;
uint8_t offsetFlag = 0;
volatile uint8_t testFlag = 0;
uint8_t flashFlag = 0;
uint8_t printFlag = 0;
uint8_t uartFlag = 0;
uint8_t timer_count = 50;

uint8_t startHex = 0x02;
uint8_t endHex = 0x03;
//mode 1 -> LoadCell
//mode 2 -> IMU

int32_t writeFlashMemoryData = 0;
int32_t getFlashMemoryData = 0;


uint16_t rawData=1, offsetData = 1, movingAverageCount = 0, movingAverageData=1;
uint16_t movingAverageResultData = 1, medianCount=0 , medianData=1;
uint16_t medianBuf[MEDIAN]={0,}, movingAverageBuf[AVERAGE]={0,};

int32_t resultData=1;

uint32_t rawDataSumValue = 0;

//double scale = 0.480833, weightData=0.0, ADC_Value = 4.0; //0.464, 0.476
//loadcell 1: 0.481
//loadcell 3: 0.455, 0.470400(SD3)
double scale = 0.0, weightData=1.0, ADC_Value = 4.2, bat_shutdown = 3.7; //0.464, 0.476
volatile uint8_t txd='9';

//MPU9250 parameter
uint8_t isDeviceConnected = 0;

int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Mag_X, Mag_Y, Mag_Z;
int16_t temperature_d;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float GyroXX, GyroYY, GyroZZ;
float MagX, MagY, MagZ;
float temperature;

float offset_gryo_x, offset_gryo_y, offset_gryo_z;
float offset_mag_x, offset_mag_y, offset_mag_z;

float temp_gryo_x, temp_gryo_y, temp_gryo_z;
float temp_mag_x, temp_mag_y, temp_mag_z;



//gyro
float pitch = 0.0, roll=0.0, yaw=0.0;
float gyro_pitch, gyro_roll, gyro_yaw;
float acc_total_vector = 0.0, acc_pitch = 0.0, acc_roll = 0.0;
float acc_pitch_output = 0.0, acc_roll_output = 0.0;
float result_pitch = 0.0, result_roll = 0.0, result_yaw = 0.0, pitch_offset = 0.0, roll_offset = 0.0, yaw_offset = 0.0;
//acc


//IMU Flag
int8_t set_gyro_angles = 0;

int a(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
	return len;
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);   //timer interrupt enable
  DWT_Delay_Init();   //1us timer init
  HAL_UART_Receive_IT(&huart2, &txd, 1);   //uart interrupt enable


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //MCU LED enable

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); //Bluetooth Reset PIN -> HIGH

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //Blue LED off

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); //Red LED off

  //read scale data
  getFlashMemoryData = *(__IO uint32_t *)ADDR_FLASH_PAGE_127;

  if(getFlashMemoryData < 0)
  {
	  scale = 1.0;
	  //sd3: 0.470400
  }
  else
  {
	 scale = getFlashMemoryData/10000000.0;
	/*
	scale = 0.470400; //sd3
	HAL_FLASH_Unlock();

	FLASH_PageErase(ADDR_FLASH_PAGE_127);
	CLEAR_BIT (FLASH->CR, (FLASH_CR_PER)); //https://blog.naver.com/jjp1386/222559572690

	//write
	writeFlashMemoryData = scale * 10000000; //Need to fix
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_FLASH_PAGE_127, writeFlashMemoryData);

	HAL_FLASH_Lock();
	//read(use as needed)
	getFlashMemoryData = *(__IO uint32_t *)ADDR_FLASH_PAGE_127;

	scale = getFlashMemoryData/10000000.0; //Need to fix

	//buf
	sprintf(transmitBuf, "%d,%d,%lf\r\n", writeFlashMemoryData, getFlashMemoryData, scale); //Off
	HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
	*/
  }

  while (1)
  {
     HAL_UART_Receive_IT(&huart2, &txd, 1); //UART Interupt
     /*
      * GPIOA-11
      * Switch Push -> LOW(0)
      * Switch No Push -> HIGH(1)
      * */
     statusFlag = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11); //input value of Power_On/Off_switch

     //power on/off code
     if(statusFlag == 0 && timer_s > 999 && powerFlag == 0)   //Power on -> Power not turned on && Press the switch for 2 seconds
     {
        timer_s=0;   //timer init
        powerFlag = 2;   //powerFlag to prevent turning off again when the switch is held down
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // NPN Transistor On -> Continues to supply power without pressing the switch
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //LED On -> Power on
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); //Red LED disable

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //Blue LED on

        loadCellInit(); //Function to fill empty load cell raw data buffer



        /* x, y, z code
        while(MPU9255_Init(&hi2c1) == 1); //modify
		*/


        /* existing x, y code */
        MPU9250_Init(&mpu9250, MPU9250_Device_0, ACCEL_SCALE_2G, GYRO_SCALE_250dps, MAG_SCALE_16bit);

        if(whoAmI_Check(&mpu9250) != HAL_ERROR)
        {
        	isDeviceConnected = 1;
            sprintf(transmitBuf, "ON, %lf\n", scale); //On
            HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
        }
        else
        {
        	isDeviceConnected = 0;
        	sprintf(transmitBuf, "MPU9250 Error\n"); //Error
			HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
        }
     }
     else if(statusFlag == 0 && timer_s > 999 && powerFlag == 1)   //power Off -> Power On && Press the switch for 2 seconds
     {
        timer_s=0;   //timer init
        powerFlag = 3;
        ledFlag = 0;
        startFlag = 0;
        if(flashFlag == 1)
        {
			flashFlag = 0;
			sprintf(transmitBuf, "flash memory write! %d,%d,%lf\n", writeFlashMemoryData, getFlashMemoryData, scale); //Off
			HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
			//erase
			HAL_FLASH_Unlock();

			FLASH_PageErase(ADDR_FLASH_PAGE_127);
			CLEAR_BIT (FLASH->CR, (FLASH_CR_PER)); //https://blog.naver.com/jjp1386/222559572690

			//write
			writeFlashMemoryData = scale * 10000000; //Need to fix
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_FLASH_PAGE_127, writeFlashMemoryData);

			HAL_FLASH_Lock();
			//read(use as needed)
			getFlashMemoryData = *(__IO uint32_t *)ADDR_FLASH_PAGE_127;

			scale = getFlashMemoryData/10000000.0; //Need to fix

			//buf
			sprintf(transmitBuf, "flash memory write! %d,%d,%lf\n", writeFlashMemoryData, getFlashMemoryData, scale); //Off
			HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // NPN Transistor Off -> Power Off
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);   //MCU LED Off -> Power Off
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //Blue LED off
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); //Red LED off

        sprintf(transmitBuf, "OFF\n"); //Off
        HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
     }
     else if(statusFlag && powerFlag == 2)   //When the switch is pressed for more than 2 seconds
     {
        timer_s = 0;   //timer init
        powerFlag = 1;   //powerFlag to prevent turning off again when the switch is held down
     }
     else if(statusFlag && powerFlag == 3)
     {
    	 timer_s = 0;
    	 powerFlag = 0;
     }
     else if(timer_s > 999) //timer Max Value: 2^16 = 65536 -> overflow prevent
     {
        timer_s = 0;
     }
     else if(timer_s > 1999)
     {
    	 timer_ss = 0;
     }

     if(ADC_Value <= bat_shutdown) //dropout voltage
     {
        startFlag = 0;
        ledFlag = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //Blue LED off
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); //Red LED on
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //MCU LED off
        readADC();   //battery ADC read
     }


     if(powerFlag) //firmware start
     {
    	 //readAll(&hi2c1, &MPU9255);

        /*Bluetooth LED control*/
        if(ledFlag)
        {
        	// 1 == connected
        	// 0 == disconnected
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 0) //=>SPP
//        	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1) // => BLE
            {
               HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //Blue LED ON
            }
            else
            {
               if(timer_ss > 999)
               {
            	   timer_ss = 0;
            	   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); //Blue LED on
               }

            }
        }

        readADC();   //battery ADC read

        if(printFlag)
        {
        	printFlag = 0;
        	sprintf(transmitBuf, "%lf,%d,%d,%s\n", scale, flashFlag, timer_count, testBuf); //Off
			HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
        }

        if(startFlag == 1) //handgrip mode
        {

            rawData=
(); //load cell raw data read function
            /**/
            medianData = median(rawData);   //Apply median filtering filtering to moving average data

            movingAverageData=movingAverage(medianData); //Apply moving average filtering to raw data

            resultData = movingAverageData-offsetData;

            weightData = ((double)resultData/scale)/1000.0;


            if(offsetFlag == 1)
            {
               offsetFlag = 0;
               offsetData = movingAverageData;
            }

            if(timer_ms >= timer_count && testFlag  == 0) //current code => U
			{
			   timer_ms=0;
			   if(weightData == INFINITY)
			   {
				   weightData = 1.0;
			   }
			   //sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,b\n",ADC_Value,pitch,roll,yaw,weightData);
			   sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,b\n",ADC_Value,result_pitch,result_roll,result_yaw,weightData); //test code
			   HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
			   //sprintf(transmitBuf, "a,%.2lf,%.2lf,%.0lf,b\r\n", result_pitch, result_roll,ADC_Value);
			   //sprintf(transmitBuf, "%c,%.2lf,%.0lf,%c\r\n",startHex,weightData, ADC_Value,endHex); //"startBit switchValue rawData endBit
			   //sprintf(transmitBuf, "%.3lfkg %.2lfV\r\n", weightData, ADC_Value); //"startBit switchValue rawData endBit
			   //sprintf(transmitBuf, "%05d %05d %05d %05d %05d %05d %05d %05d %05d\r\n", Accel_X, Accel_Y, Accel_Z, Gyro_X, Gyro_Y, Gyro_Z, MagX, MagY, MagZ);
			}

            else if(timer_ms >= timer_count && testFlag != 0) //raw data 1 => S
            {
               timer_ms=0;
               sprintf(transmitBuf, "a,%.2lf,%05d,%05d,%05d,%lf,%.2lf,b\n",ADC_Value, movingAverageData, offsetData, resultData, scale, weightData); //"startBit switchValue rawData endBit
               HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
               //sprintf(transmitBuf, "%c,%.2lf,%.0lf,%c\r\n",startHex,weightData, ADC_Value,endHex); //"startBit switchValue rawData endBit
               //sprintf(transmitBuf, "%.3lfkg %.2lfV\r\n", weightData, ADC_Value); //"startBit switchValue rawData endBit
               //sprintf(transmitBuf, "%05d %05d %05d %05d %05d %05d %05d %05d %05d\r\n", Accel_X, Accel_Y, Accel_Z, Gyro_X, Gyro_Y, Gyro_Z, MagX, MagY, MagZ);
            }
        }
        else if(startFlag == 2) // exercise(nextqure)
        {
            //only gyro
            MPU9250_ReadAcc(&mpu9250);
            //MPU9250_ReadGyro(&mpu9250);
            //MPU9250_ReadMag(&mpu9250);


/*            result_pitch = atan2f(mpu9250.acc[1],sqrtf((mpu9250.acc[0] * mpu9250.acc[0])+((mpu9250.acc[2] * mpu9250.acc[2])))) * 57.3;
            result_roll = atan2f(-mpu9250.acc[0],sqrtf((mpu9250.acc[1] * mpu9250.acc[1])+((mpu9250.acc[2] * mpu9250.acc[2])))) * 57.3;

            float Yh = (mpu9250.mag[1] * cosf(result_roll)) - (mpu9250.mag[2] * sinf(result_roll));
            float Xh = (mpu9250.mag[0] * cosf(result_pitch)) + (mpu9250.mag[1] * sinf(result_roll)*sinf(result_pitch)) + (mpu9250.mag[2] * cosf(result_roll) * sinf(result_pitch));
            result_yaw = atan2f(Yh, Xh) * 57.3;*/
            result_pitch = atan2f(mpu9250.acc[1],sqrtf((mpu9250.acc[0] * mpu9250.acc[0])+((mpu9250.acc[2] * mpu9250.acc[2])))) * (180/PI);
			result_roll = atan2f(-mpu9250.acc[0],sqrtf((mpu9250.acc[1] * mpu9250.acc[1])+((mpu9250.acc[2] * mpu9250.acc[2])))) * (180/PI);

			//float Yh = (mpu9250.mag[1] * cosf(result_roll)) - (mpu9250.mag[2] * sinf(result_roll));
			//float Xh = (mpu9250.mag[0] * cosf(result_pitch)) + (mpu9250.mag[1] * sinf(result_roll)*sinf(result_pitch)) + (mpu9250.mag[2] * cosf(result_roll) * sinf(result_pitch));
			//result_yaw = atan2f(Yh, Xh);
			//result_yaw = 0.0;

/*
            acc_total_vector = sqrtf((mpu9250.acc[0] * mpu9250.acc[0]) + (mpu9250.acc[1] * mpu9250.acc[1]) + (mpu9250.acc[2] * mpu9250.acc[2]));
            AccX = asinf((float)mpu9250.acc[1] / acc_total_vector) * 57.296;
            AccY = asinf((float)mpu9250.acc[0] / acc_total_vector) * -57.296;

            acc_pitch_output =  acc_pitch_output * 0.99 + AccX * 0.01;
            acc_roll_output =  acc_roll_output * 0.99 + AccY * 0.01;

            pitch = acc_pitch_output;
            roll = acc_roll_output;

            result_pitch = pitch - pitch_offset;
            result_roll = roll - roll_offset;
*/

            if(offsetFlag == 1)
            {
               offsetFlag = 0;
               pitch_offset = pitch;
               roll_offset = roll;
            }

            /*
            MPU9250_ReadGyro(&mpu9250);
            //MPU9250_ReadMag(&mpu9250);
            //MPU9250_ReadTemperature(&mpu9250);

            mpu9250.gyro[0] -= offset_gryo_x;
            mpu9250.gyro[1] -= offset_gryo_y;
            mpu9250.gyro[2] -= offset_gryo_z;

            GyroX += mpu9250.gyro[0] * 0.05; //only gyro X (rawdata/131*0.05)
            GyroY += mpu9250.gyro[1] * 0.05; //only gyro Y
            GyroZ += mpu9250.gyro[2] * 0.05; //only gyro Z

            //GyroYY -= GyroY * sinf(GyroZ * (PI/180));
            //gyro Z and gyro x => gyro y
            //GyroXX += GyroX * sinf(GyroZ * (PI/180));
            //gyro Z and gyro y => gyro x

            pitch = GyroX * 0.9996 + AccX * 0.0004;
            roll = GyroY * 0.9996 + AccY * 0.0004;

            //mpu9250.acc[0] -= offset_acc_x;
            //mpu9250.acc[1] -= offset_acc_y;
            //mpu9250.acc[2] -= offset_acc_z;
            */
            if(timer_ms >= timer_count && testFlag == 0) //test code 1 => U
            {
               timer_ms=0;
//               sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,b\n",ADC_Value,result_pitch,result_roll,result_yaw,weightData); //existing
               sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,b\n",ADC_Value,result_pitch,result_roll,result_yaw,weightData); //test code
               //sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,b,",ADC_Value,result_pitch,result_roll,weightData); //"startBit switchValue rawData endBit
               HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
            }
            else if(timer_ms >= timer_count && testFlag == 1) //raw code acc, mag => S
            {
               timer_ms=0;
               sprintf(transmitBuf, "a,%.2lf,%05d,%05d,%05d,%05d,%05d,%05d,b\n",ADC_Value,mpu9250.acc_raw[0], mpu9250.acc_raw[1], mpu9250.acc_raw[2], mpu9250.mag_raw[0], mpu9250.mag_raw[1], mpu9250.mag_raw[2]);
               HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
            }
            else if(timer_ms >= timer_count && testFlag == 2) //raw code acc, gyro, mag => D
            {
            	timer_ms=0;
			   sprintf(transmitBuf, "a,%.2lf,%05d,%05d,%05d,%05d,%05d,%05d,%05d,%05d,%05d,b\n",ADC_Value,mpu9250.acc_raw[0], mpu9250.acc_raw[1], mpu9250.acc_raw[2], mpu9250.gyro_raw[0], mpu9250.gyro_raw[1], mpu9250.gyro_raw[2], mpu9250.mag_raw[0], mpu9250.mag_raw[1], mpu9250.mag_raw[2]);
			   HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
            }
        }
        else if(startFlag == 3)
        {
        	readAll(&hi2c1, &MPU9255);

        	pitch = MPU9255.pitch;
        	roll = MPU9255.roll;
        	yaw = MPU9255.yaw;

        	if(timer_ms >= timer_count && testFlag == 0) //test code 1 => U
			{
			   timer_ms=0;
//               sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,b\n",ADC_Value,result_pitch,result_roll,result_yaw,weightData); //existing
			   sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,b\r\n",ADC_Value,pitch,roll,yaw,weightData); //test code
			   //sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,b,",ADC_Value,result_pitch,result_roll,weightData); //"startBit switchValue rawData endBit
			   HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
			}
        }
     }


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RED_LED_Pin|BLUE_LED_Pin|BT_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCU_LED_Pin|SW_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SC_Pin */
  GPIO_InitStruct.Pin = SC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_Pin BLUE_LED_Pin BT_RESET_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|BLUE_LED_Pin|BT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_LED_Pin SW_OUT_Pin */
  GPIO_InitStruct.Pin = MCU_LED_Pin|SW_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_INT_Pin */
  GPIO_InitStruct.Pin = SW_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if ((htim->Instance == TIM2))
    {
       timer_ms++;
       timer_s++;
       timer_ss++;
      }
}

int readADC()
{
   HAL_ADC_Start(&hadc1);
   HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
   ADC_Value = HAL_ADC_GetValue(&hadc1);
   HAL_ADC_Stop(&hadc1);
   //ADC_Value = (((ADC_Value/4096*3.3*2)+0.15)-3.7)*200; //battery voltage


   //bluetooth3 board +0.11
   //ADC_Value = (ADC_Value/4096*3.3*2)+0.11; //battery voltage

   //bluetooth4 board -1.48
   //ADC_Value = (ADC_Value/4096*3.3*2)-1.48; //battery voltage

   //bluetooth6 board +0.16
   ADC_Value = (ADC_Value/4096*3.3*2); //battery voltage
   /*
   if(ADC_Value > 100.0)
   {
      ADC_Value = 100.0;
   }
   */
}

void append(char *dst, char c)
{
	char *p = dst;
	while(*p != '\0') p++;
	*p = c;
	*(p+1) = '\0';
}

int readData() //loadcell raw data get -> SPI
{
   uint8_t dataS = 0, dataF = 0, dataM = 0, dataL = 0;
   uint16_t dataF_16 = 0, dataM_16 = 0, dataL_16 = 0;

   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);   //nss select
   DWT_Delay_us(READTIME);

   HAL_SPI_TransmitReceive(&hspi2, &dataS, &dataF, 1, 10);
   HAL_SPI_TransmitReceive(&hspi2, &dataS, &dataM, 1, 10);
   HAL_SPI_TransmitReceive(&hspi2, &dataS, &dataL, 1, 10);

   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   //nss deselect
   DWT_Delay_us(READTIME);

   dataF_16 = dataF << 14;
   dataM_16 = dataM << 6;
   dataL_16 = (dataL >> 2) & 0x3E;

   return (dataF_16) + (dataM_16) + (dataL_16);
}

void loadCellInit()
{
   for(int i=0; i<AVERAGE; i++)
   {
      movingAverageBuf[i] = readData();
      rawDataSumValue = rawDataSumValue + movingAverageBuf[i];
   }
}

uint16_t movingAverage(uint16_t rawdata)
{
   rawDataSumValue = rawDataSumValue - movingAverageBuf[movingAverageCount];

   movingAverageBuf[movingAverageCount] = rawdata;

   rawDataSumValue = rawDataSumValue + movingAverageBuf[movingAverageCount];

   movingAverageResultData = rawDataSumValue/AVERAGE;

   movingAverageCount++;
   if(movingAverageCount >= AVERAGE)
   {
      movingAverageCount=0;
   }

   return movingAverageResultData;
}

uint16_t median(uint16_t averagedata)
{
   int temp;

   medianBuf[medianCount] = averagedata;

   for(int i=0; i<MEDIAN-1; i++)
   {
      for(int j=0; j<MEDIAN-1-i; j++)
      {
         if(medianBuf[j]>medianBuf[j+1])
         {
            temp = medianBuf[j];
            medianBuf[j] = medianBuf[j+1];
            medianBuf[j+1] = temp;
         }
      }
   }
   medianCount++;
   if(medianCount >= MEDIAN)
   {
      medianCount=0;
   }

   return medianBuf[(MEDIAN-1)/2];
}

HAL_StatusTypeDef whoAmI_Check(MPU9250_t *mpu9250)
{
   uint8_t data;
   /* MPU9250 Who Am I Register Check */
   if (readByte(&hi2c1, mpu9250->I2C_Addr, WHO_AM_I, &data) != HAL_OK)
   {
      if (data != 0x71)
         return HAL_ERROR;
   }
   /* AK8963 Who Am I Register Check */
   if (readByte(&hi2c1, mpu9250->I2C_Addr_Mag, WIA, &data) != HAL_OK)
   {
      if (data != 0x48)
         return HAL_ERROR;
   }
   return HAL_OK;
}

void Calibration()
{
   MPU9250_ReadGyro(&mpu9250);
   MPU9250_ReadMag(&mpu9250);
   for(int i=0; i<IMUAVERAGE; i++)
   {
      temp_gryo_x += mpu9250.gyro[0];
      temp_gryo_y += mpu9250.gyro[1];
      temp_gryo_z += mpu9250.gyro[2];

      temp_mag_x += mpu9250.mag[0];
      temp_mag_y += mpu9250.mag[1];
      temp_mag_z += mpu9250.mag[2];
   }
   offset_gryo_x = temp_gryo_x / IMUAVERAGE;
   offset_gryo_y = temp_gryo_y / IMUAVERAGE;
   offset_gryo_z = temp_gryo_z / IMUAVERAGE;

   offset_mag_x = temp_mag_x / IMUAVERAGE;
   offset_mag_y = temp_mag_y / IMUAVERAGE;
   offset_mag_z = temp_mag_z / IMUAVERAGE;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   //if(huart->Instance == USART2 || huart->Instance == USART3)
   if(huart->Instance == USART2)
   {
      HAL_UART_Receive_IT(&huart2, &txd, 1);
      append(testBuf, txd);
      //sprintf(transmitBuf, "UART: %c\r\n", txd);
      //HAL_UART_Transmit(&huart2, transmitBuf, strlen(transmitBuf), 10);
      if(txd == 'O' && uartFlag == 2) //offset
      {
		 offsetFlag = 1;
		 uartFlag = 0;
      }
      else if(txd == 'Z' && uartFlag == 2)   //scale(0.5kg)
      {
         scale=(rawData-offsetData)/500.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'Y' && uartFlag == 2)   //scale(1kg)
      {
         scale=(rawData-offsetData)/1000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'X' && uartFlag == 2)   //scale(5kg)
      {
         scale=(rawData-offsetData)/5000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'W' && uartFlag == 2)   //scale(10kg)
      {
         scale=(rawData-offsetData)/10000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'V' && uartFlag == 2)   //scale(15kg)
      {
         scale=(rawData-offsetData)/15000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'R' && uartFlag == 2)   //scale(20kg)
      {
         scale=(rawData-offsetData)/20000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'Q' && uartFlag == 2)   //scale(25kg)
      {
         scale=(rawData-offsetData)/25000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'N' && uartFlag == 2)   //scale(30kg)
      {
         scale=(rawData-offsetData)/30000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'M' && uartFlag == 2)   //scale(35kg)
      {
         scale=(rawData-offsetData)/35000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'L' && uartFlag == 2)   //scale(40kg)
      {
         scale=(rawData-offsetData)/40000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'K' && uartFlag == 2)   //scale(45kg)
      {
         scale=(rawData-offsetData)/45000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'J' && uartFlag == 2)   //scale(50kg)
      {
         scale=(rawData-offsetData)/50000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'G' && uartFlag == 2)   //scale(55kg)
      {
         scale=(rawData-offsetData)/55000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'F' && uartFlag == 2)   //scale(60kg)
      {
         scale=(rawData-offsetData)/60000.0;
         startFlag = 0;
         flashFlag = 1;
         uartFlag = 0;
      }
      else if(txd == 'I' && uartFlag == 2)   //Init
      {
         //scale=0.455;
         offsetData = 0;
         pitch_offset = 0.0;
         roll_offset = 0.0;
         yaw_offset = 0.0;
         uartFlag = 0;
      }
      else if(txd == 'P' && uartFlag == 2)   //print
      {
    	  printFlag = 1;
    	  uartFlag = 0;
      }
      else if(txd == 'H' && uartFlag == 2)   //data transmit start, S = 0x53, s = 0x73, handgrip = h, H
      {
         startFlag = 1;
         offsetFlag = 1;
         uartFlag = 0;
         HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_9);
      }
      else if(txd == 'E' && uartFlag == 2)   //data transmit start, S = 0x53, s = 0x73, exercise = e, E
      {
         startFlag = 3;		//Lab
         //startFlag = 2; //nextqure
         uartFlag = 0;
         HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_9);
      }
      else if(txd == 'T' && uartFlag == 2)   //data transmit end, T = 0x54, t = 0x74
      {
         startFlag = 0;
         offsetData = 0;
         pitch_offset = 0.0;
         roll_offset = 0.0;
         uartFlag = 0;
         HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_9);
      }
      //sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,b\r\n",ADC_Value,result_pitch,result_roll,weightData); //"startBit switchValue rawData endBit
		else if(txd == 'C' && uartFlag == 2) //no use
		{
		  testFlag = 3;
		  uartFlag = 0;
		  HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_9);
		}
      //sprintf(transmitBuf, "a,%.2lf,%.2lf,%.2lf,%.2lf,b,\r\n",ADC_Value,result_pitch,result_roll,weightData); //"startBit switchValue rawData endBit
      else if(txd == 'D' && uartFlag == 2) //raw Data 2
      {
    	  testFlag = 2;
    	  uartFlag = 0;
    	  HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_9);
	  }
      else if(txd == 'S' && uartFlag == 2) //raw Data 1
      {
         testFlag = 1;
         uartFlag = 0;
         HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_9);
      }

      else if(txd == 'U' && uartFlag == 2) //current code
      {
         testFlag = 0;
         uartFlag = 0;
         HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_9);
      }
      else if(txd == '1' && uartFlag == 2) //10Hz
		{
		  timer_count = 100;
		  uartFlag = 0;
		}
      else if(txd == '2' && uartFlag == 2) //20Hz
		{
		  timer_count = 50;
		  uartFlag = 0;
		}
      else if(txd == '3' && uartFlag == 2) //25Hz
		{
		  timer_count = 40;
		  uartFlag = 0;
		}
      else if(txd == '4' && uartFlag == 2) //40Hz
		{
		  timer_count = 25;
		  uartFlag = 0;
		}
      else if(txd == '5' && uartFlag == 2) //50Hz
		{
		  timer_count = 20;
		  uartFlag = 0;
		}
      else if(txd == '6' && uartFlag == 2) //bat_shutdown 3.7
		{
		  bat_shutdown = 3.7;
		  uartFlag = 0;
		}
      else if(txd == '7' && uartFlag == 2) //bat_shutdown 3.65
		{
    	  bat_shutdown = 3.65;
		  uartFlag = 0;
		}
      else if(txd == '8' && uartFlag == 2) //bat_shutdown 3.6
		{
    	  bat_shutdown = 3.60;
		  uartFlag = 0;
		}
      else if(txd == 'A' && uartFlag == 0)
      {
    	  uartFlag++;
      }
      else if(txd == 'B' && (uartFlag == 1 || uartFlag == 2))
      {
    	  uartFlag++;
      }
      else if(txd == 'A' && (uartFlag == 1 || uartFlag == 2))
      {
    	  uartFlag = 1;

      }
      else if(uartFlag >= 3)
      {
    	  uartFlag = 0;
      }

   }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
