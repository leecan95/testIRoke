/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "IRremote.h"
#include "MQTTClient.h"
#include "MQTTConnect.h"
#include "amw_uart.h"
#include "svm30.h"
#include "sensirion_i2c.h"
#include "sm_uart-04l.h"
#include "fan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MQTT_PUB_TOPIC              "leetest"
#define MQTT_SUB_TOPIC              "diem"
#define MQTT_CLIENT_ID              "Zentri"
#define MQTT_USER                   "myuncjoa"
#define MQTT_PASSWORD               "SvVx1BPwTVqk"
#define MQTT_HOST                   "m24.cloudmqtt.com"
#define MQTT_PORT                   14022
#define MQTT_PUBLISH_PERIOD         5000
#define MQTT_MESSAGE                "{\"name\": \"ERV\", \"id\": \"hethong123\"}"
//** Define SSID và PASSWORD để kết nối wifi **//
#define SSID_WIFI "Ceidanang open"
#define PASS_WIFI "ceidanang"
//** Define MQTT TOPIC cần subcribse từ Broker để giao tiếp vs app **//
#define TOPIC_DEVICE_ID "esp/request/collect/device-info"
#define TOPIC_AIR_QUALITY "esp/request/collect/air-quality"
#define TOPIC_CONTROL_STATE "esp/request/collect/control-state"
#define TOPIC_POWER_ON "esp/request/control/power-on"
#define TOPIC_POWER_OFF "esp/request/control/power-off"
#define TOPIC_NIGHT_MODE "esp/request/control/night_mode"
#define TOPIC_ION "esp/request/control/ion"
#define TOPIC_UV "esp/request/control/uv"
#define TOPIC_FILTER "esp/request/control/filter_mode"
#define TOPIC_CONTROL "esp/request/control/control_mode"
#define TOPIC_SPEED_LOW "esp/request/control/low"
#define TOPIC_SPEED_MED "esp/request/control/med"
#define TOPIC_SPEED_HIGH "esp/request/control/high"
//** Define MQTT Topic cần để publish lên Broker**//
#define TOPIC_PUB_AIR_QUALITY 	"esp/response/collect/air-quality"
#define TOPIC_PUB_CONTROL_STATE  "esp/response/collect/control-state"
#define TOPIC_PUB_INFO "esp/response/collect/device-info"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId SensorTaskHandle;
osThreadId ControlTaskHandle;
osThreadId DisplayTaskHandle;
osThreadId ServerTaskHandle;
/* USER CODE BEGIN PV */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartSensor_Task(void const * argument);
void StartControl_Task(void const * argument);
void StartDisplay_task(void const * argument);
void StartServer_task(void const * argument);

/* USER CODE BEGIN PFP */
// biến mqtt
char  Rx_data[2], Rx_Buffer[40];
unsigned char uart_reponse[512];
int Transfer_cplt = 0;
int Rx_indx = 0;
MQTTClient client = DefaultClient;
Network network;
MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
static MQTTClient client1 = DefaultClient;
uint8_t start_subcribe = 0;
int i = 0;
int ret;
unsigned char send_buf[512];
unsigned char recv_buf[2048];
char AQI[50];
char state[50];
//biến chế độ hệ thống
int night_mode;
int filter_mode;
int speed;
int power;
int Vanst = 0;
int control_mode;
int UV_mode;
int ION_mode;
int ion_val;
bool huong;
char powerc[10];
char speedc[10];
char night[10];
char filter[10];
char control[10];
char uv[10];
char ion[10];
// biến đ�?c giá trị từ sensor

int16_t err;
uint16_t tvoc_ppb = 1, co2_ppm = 2;
uint32_t iaq_baseline;
int32_t temperature = 0, humidity = 0;
uint16_t PM25;
float temp, hum;
float PM25f;
int tmpIntPM1,tmpIntte1,tmpInthu1,tmpIntPM2,tmpIntte2,tmpInthu2;

struct amphenol amphenol1;
// Biến xử lí IR
int SwitchIR;
int ModeIR;
int SpeedIR;
int IonIR;
int flag = 0;
int flagSW, flagSP, flagMode, flagIon;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
int count = 0;


static int publish(void* payload,void* topic)
{
    char buf[100];
    MQTTMessage message;
    int rc;
    sprintf(buf, payload, (long unsigned int) HAL_GetTick()/1000);
    message.qos = QOS1;
    message.retained = 1;
    message.dup = 0;
    message.payload = (void*)buf;
    message.payloadlen = strlen(buf);

    rc = MQTTPublish(&client, topic, &message);
    return rc;
}
static void receive_handler(MessageData* rx_msg)
{

    char* topic = rx_msg->topicName->lenstring.data;
    //topic[rx_msg->topicName->lenstring.len] = '\0';
    printf("receive mqtt topic %s\r\n", topic);
    char *message = rx_msg->message->payload;
	message[rx_msg->message->payloadlen] = '\0';
	printf("receive mqtt %s\r\n", rx_msg->message->payload);
    if (!strncmp(topic,TOPIC_DEVICE_ID, strlen(TOPIC_DEVICE_ID))){
    	publish(MQTT_MESSAGE,TOPIC_PUB_INFO);
    }
    else if (!strncmp(topic,TOPIC_AIR_QUALITY, strlen(TOPIC_AIR_QUALITY))){
    	if (!strncmp(rx_msg->message->payload, "get-hethong123", rx_msg->message->payloadlen)){
    		publish(AQI,TOPIC_PUB_AIR_QUALITY);
    	}
    }
    else if (!strncmp(topic,TOPIC_CONTROL_STATE, strlen(TOPIC_CONTROL_STATE))){
		if (!strncmp(rx_msg->message->payload, "get-hethong123", rx_msg->message->payloadlen)){
			publish(state,TOPIC_PUB_CONTROL_STATE);
		}
    }
    else if (!strncmp(topic,TOPIC_POWER_ON, strlen(TOPIC_POWER_ON))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
			power = Power_On;
			speed = med;
			//fan_12_run(medspeed);
			xiaomi_fan_run(MED_SPEED);
			SwitchIR = 1;
		}
	}
    else if (!strncmp(topic,TOPIC_POWER_OFF, strlen(TOPIC_POWER_OFF))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
			power = Power_Off;
			speed = Power_Off;
			//fan_1_stop();
			//fan_2_stop();
			xiaomi_fan_stop();
			SwitchIR = 2;
		}
	}
    else if (!strncmp(topic,TOPIC_SPEED_LOW, strlen(TOPIC_POWER_OFF))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
			speed = low;
			//fan_12_run(lowspeed);
			xiaomi_fan_run(LOW_SPEED);
			SpeedIR = 1;
		}
	}
    else if (!strncmp(topic, TOPIC_SPEED_MED, strlen(TOPIC_POWER_OFF))){
   		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
   			speed = med;
   			//fan_12_run(medspeed);
   			xiaomi_fan_run(MED_SPEED);
   			SpeedIR=2;
   		}
   	}
    else if (!strncmp(topic, TOPIC_SPEED_HIGH, strlen(TOPIC_SPEED_HIGH))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123", rx_msg->message->payloadlen)){
			speed = hight;
			//fan_12_run(hightspeed);
			xiaomi_fan_run(HIGH_SPEED);
			SpeedIR=3;
		}
	}
    else if (!strncmp(topic,TOPIC_NIGHT_MODE, strlen(TOPIC_NIGHT_MODE))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123 on", rx_msg->message->payloadlen)){
			speed = low;
			//fan_12_run(lowspeed);
			xiaomi_fan_run(LOW_SPEED);
			night_mode = Nightmode_On;
			ModeIR = 3;

		}
		else if (!strncmp(rx_msg->message->payload, "control-hethong123 off", rx_msg->message->payloadlen)){
			speed = med;
			//fan_12_run(medspeed);
			xiaomi_fan_run(MED_SPEED);
			night_mode = Nightmode_Off;
		}
	}
    else if (!strncmp(topic,TOPIC_FILTER, strlen(TOPIC_FILTER))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123 fresh_air", rx_msg->message->payloadlen)){
			filter_mode = Fresh_Air;
			Vanst = 1;
			ModeIR = 1;
		}
		else if (!strncmp(rx_msg->message->payload, "control-hethong123 indoor", rx_msg->message->payloadlen)){
			filter_mode = Indoor;
			Vanst = 0;
			ModeIR = 2;
		}
	}
    else if (!strncmp(topic, TOPIC_CONTROL, strlen(TOPIC_CONTROL))){
  		if (!strncmp(rx_msg->message->payload, "control-hethong123 auto", rx_msg->message->payloadlen)){
  			control_mode = Auto ;
  		}
  		else if (!strncmp(rx_msg->message->payload, "control-hethong123 manual", rx_msg->message->payloadlen)){
  			control_mode = Manual;
  		}
  	}
    else if (!strncmp(topic, TOPIC_UV, strlen(TOPIC_UV))){
   		if (!strncmp(rx_msg->message->payload, "control-hethong123 on", rx_msg->message->payloadlen)){
   			UV_mode = UVon;
   			HAL_GPIO_WritePin(UV_GPIO_Port, UV_Pin, GPIO_PIN_SET);

   		}
   		else if (!strncmp(rx_msg->message->payload, "control-hethong123 off", rx_msg->message->payloadlen)){
   			UV_mode = UVoff;
   			HAL_GPIO_WritePin(UV_GPIO_Port, UV_Pin, GPIO_PIN_RESET);

   		}
   	}
    else if (!strncmp(topic, TOPIC_ION, strlen(TOPIC_ION))){
		if (!strncmp(rx_msg->message->payload, "control-hethong123 on", rx_msg->message->payloadlen)){
			ION_mode = IONon;
			HAL_GPIO_WritePin(Ion_GPIO_Port, Ion_Pin, GPIO_PIN_SET);
			IonIR = 1;

		}
		else if (!strncmp(rx_msg->message->payload, "control-hethong123 off", rx_msg->message->payloadlen)){
			ION_mode = IONoff;
			HAL_GPIO_WritePin(Ion_GPIO_Port, Ion_Pin, GPIO_PIN_SET);
			IonIR = 2;
		}
	}

}
void Convertvalue(void){

	//PM25 = 22.33;
	tmpIntPM1 = (int)PM25;
	float tmpFPM = PM25 - tmpIntPM1;
	tmpIntPM2 = trunc(tmpFPM * 100);
	//temperature = 9977;
	temp = (float)(temperature/1000.0f);
	tmpIntte1 = (int)temp;
	float tmpT = temp - tmpIntte1;
	tmpIntte2 = trunc(tmpT * 100);
	//humidity = 9876;
	hum = (float)(humidity/1000.0f);
	tmpInthu1 = (int) hum;
	float tmpH = hum - tmpInthu1;
	tmpInthu2 = trunc(tmpH * 100);
}
void Control_IR(void){

	 if (SwitchIR == 1 && flagSW == 1){
			  power = Power_On;
			  speed = med;
			  //fan_12_run(medspeed);
			  xiaomi_fan_run(MED_SPEED);
			  flagSW = 0;
		  }
		  else if (SwitchIR == 2 && flagSW == 1) {
			  power = Power_Off;
			  speed = Power_Off;
			  //fan_1_stop();
			  //fan_2_stop();
			  xiaomi_fan_stop();
			  flagSW = 0;
		  }
		  if (SpeedIR == 1 && flagSP == 1 && SwitchIR == 1){
			  //fan_12_run(lowspeed);
			  xiaomi_fan_run(LOW_SPEED);
			  flagSP = 0;
		  }
		  else if (SpeedIR == 2 && flagSP == 1 && SwitchIR == 1){
			  //fan_12_run(medspeed);
			  xiaomi_fan_run(MED_SPEED);
			  flagSP = 0;
		  }
		  else if (SpeedIR == 3 && flagSP == 1 && SwitchIR == 1){
			 //fan_12_run(hightspeed);
			  xiaomi_fan_run(HIGH_SPEED);
			  flagSP = 0;
		  }
		  if (ModeIR == 1 && flagMode == 1 && SwitchIR == 1){
			  filter_mode = Fresh_Air;
			  Vanst = 1;
			  night_mode = Nightmode_Off;
			  flagMode = 0;
		  }
		  else if (ModeIR == 2 && flagMode == 1 && SwitchIR == 1){
			  filter_mode = Indoor;
			  Vanst = 0;
			  night_mode = Nightmode_Off;
			  flagMode = 0;
		  }
		  else if (ModeIR == 3 && flagMode == 1 && SwitchIR == 1){
			  speed = low;
			  //fan_12_run(lowspeed);
			  xiaomi_fan_run(LOW_SPEED);
			  night_mode = Nightmode_On;
			  flagMode = 0;

		  }
		  if (IonIR == 1 && flagIon == 1 && SwitchIR == 1){
			  ION_mode = IONon;
			  HAL_GPIO_WritePin(Ion_GPIO_Port, Ion_Pin, GPIO_PIN_SET);
			  flagIon = 0;
		  }
		  else if (IonIR == 2 && flagIon == 1 && SwitchIR == 1) {
			  ION_mode = IONoff;
			  HAL_GPIO_WritePin(Ion_GPIO_Port, Ion_Pin, GPIO_PIN_RESET);
			  flagIon = 0;
		  }
}
void checkState(){
	if (power == 0){
		sprintf(powerc,"OFF");
	}
	else {
		sprintf(powerc,"ON");
	}
	if (speed == 0){
		sprintf(speedc,"0");
		}
	else if (speed == 1) {
		sprintf(speedc,"1");
	}
	else if (speed == 2) {
		sprintf(speedc,"2");
	}
	else {
		sprintf(speedc,"3");
	}
	if (night_mode == 0){
		sprintf(night,"off");
	}
	else {
		sprintf(night,"on");
	}

	if (filter_mode == 0){
		sprintf(filter,"fresh");
	}
	else {
		sprintf(filter,"indoor");
	}

	if (control_mode == 0){
		sprintf(control,"manual");
	}
	else {
		sprintf(control,"auto");
	}

	if (UV_mode == 0){
		sprintf(uv,"off");
	}
	else {
		sprintf(uv,"on");
	}
	if (ION_mode == 0){
		sprintf(ion,"off");
	}
	else {
		sprintf(ion,"on");
	}
	sprintf(state,"%s-%s-%s-%s-%s-%s-%s",powerc,speedc,night,filter,control,uv,ion);

}
/* mình không sử dụng cái này*/
void testMQTT(){


	Network network;
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;

	Timer timer;
	// Doan nay co the thay bang ham
	  memset(uart_reponse, 0, sizeof(uart_reponse));
	  amw_write("set system.cmd.format machine\r\n");
	  i = getCmdResponse(uart_reponse);
	  printf("%d, %s\r\n", i, uart_reponse);

	  data.MQTTVersion = 4;
	  data.clientID.cstring = MQTT_CLIENT_ID;
	  data.username.cstring = MQTT_USER;
	  data.password.cstring = MQTT_PASSWORD;
	  HAL_Delay(100);
	  if(setupNetwork(SSID_WIFI, PASS_WIFI)){
		  printf("setup network failed\r\n");
	  }
	  HAL_Delay(100);
	  if(NetworkInit(&network))
		  printf("init failed\r\n");
	  else
		  printf("network up\r\n");
	  HAL_Delay(100);
	  ret = isTCPPortOpen();
	  if(ret >= 0){
		  amw_write("close 0\r\n");
		  i = getCmdResponse(uart_reponse);
		  printf("%d, %s\r\n", i, uart_reponse);
	  }
	  HAL_Delay(100);
	  ret = NetworkConnect(&network, MQTT_HOST, MQTT_PORT);
	  if(ret == FAILURE)
	  {
		  // error, return
		  printf("network connect tcp failed\r\n");
		  HAL_Delay(100);
		  amw_write("close 0\r\n");
		  i = getCmdResponse(uart_reponse);
		  printf("%d, %s\r\n", i, uart_reponse);
		  return ret;
	  }
	  printf("network connect tcp success\r\n");
	  HAL_Delay(250);

	  MQTTClientInit(&client, &network, 10000, send_buf, sizeof(send_buf), recv_buf, sizeof(recv_buf));
	  HAL_Delay(250);

	  ret = MQTTConnect(&client, &data);
	  if(ret != SUCCESS)
	  {
		  // error, return
		  printf("failed to connect mqtt\r\n");
		  NetworkDisconnect(&network);
		  return ret;
	  }
	  printf("test success to connect mqtt\r\n");
	  HAL_Delay(250);
	  MQTTSubscribe(&client, MQTT_SUB_TOPIC, QOS1, receive_handler);
	  start_subcribe = 1;
	  HAL_Delay(250);
	  publish(MQTT_MESSAGE,MQTT_PUB_TOPIC);
	  TimerInit(&timer);
	  TimerCountdownMS(&timer, 90000);
	  while (!TimerIsExpired(&timer))
	  {
	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
		  MQTTYield(&client, 500);
	  }
	  printf("test mqtt disconnect\r\n");
	  MQTTDisconnect(&client);
	  NetworkDisconnect(&network);
	  /* USER CODE END 3 */
}

void subcribeMQTT(){
	printf("subcribe\r\n");
	MQTTSubscribe(&client, "lee", QOS1, receive_handler);
	HAL_Delay(250);
	MQTTSubscribe(&client, MQTT_SUB_TOPIC, QOS1, receive_handler);
	printf("finish subcribe\r\n");
}

void initMQTT(){

	Timer timer;
	// Doan nay co the thay bang ham
	  memset(uart_reponse, 0, sizeof(uart_reponse));
	  amw_write("set system.cmd.format machine\r\n");
	  i = getCmdResponse(uart_reponse);
	  printf("%d, %s\r\n", i, uart_reponse);

	  data.MQTTVersion = 4;
	  data.clientID.cstring = MQTT_CLIENT_ID;
	  data.username.cstring = MQTT_USER;
	  data.password.cstring = MQTT_PASSWORD;
	  HAL_Delay(100);
	  if(setupNetwork(SSID_WIFI, PASS_WIFI)){
		  printf("setup network failed\r\n");
	  }
	  HAL_Delay(100);
	  if(NetworkInit(&network))
		  printf("init failed\r\n");
	  else
		  printf("network up\r\n");
	  HAL_Delay(100);
	  ret = isTCPPortOpen();
	  if(ret >= 0){
		  amw_write("close 0\r\n");
		  i = getCmdResponse(uart_reponse);
		  printf("%d, %s\r\n", i, uart_reponse);
	  }
	  HAL_Delay(100);
	  ret = NetworkConnect(&network, MQTT_HOST, MQTT_PORT);
	  if(ret == FAILURE)
	  {
		  // error, return
		  printf("network connect tcp failed\r\n");
		  HAL_Delay(100);
		  amw_write("close 0\r\n");
		  i = getCmdResponse(uart_reponse);
		  printf("%d, %s\r\n", i, uart_reponse);
		  return ret;
	  }
	  printf("network connect tcp success\r\n");
	  HAL_Delay(250);

	  MQTTClientInit(&client, &network, 10000, send_buf, sizeof(send_buf), recv_buf, sizeof(recv_buf));
	  HAL_Delay(250);

	  ret = MQTTConnect(&client, &data);
	  if(ret != SUCCESS)
	  {
		  // error, return
		  printf("failed to connect mqtt\r\n");
		  NetworkDisconnect(&network);
		  return ret;
	  }
	  printf("test success to connect mqtt\r\n");
	  HAL_Delay(250);
	  /*printf("subcribe init\r\n");
	MQTTSubscribe(&client, MQTT_SUB_TOPIC, QOS1, receive_handler);
	HAL_Delay(250);
	printf("finish subcribe init\r\n");
	publish(MQTT_MESSAGE,MQTT_PUB_TOPIC);*/
	  //subcribeMQTT();
	  /* USER CODE END 3 */
}


/* Khởi tạo kết nối MQTT */
void init_MQTT(void){
		//Network network;
		//MQTTPacket_connectData data = MQTTPacket_connectData_initializer;

		Timer timer;
		// Doan nay co the thay bang ham
		  memset(uart_reponse, 0, sizeof(uart_reponse));
		  amw_write("set system.cmd.format machine\r\n");
		  i = getCmdResponse(uart_reponse);
		  printf("%d, %s\r\n", i, uart_reponse);

		  data.MQTTVersion = 4;
		  data.clientID.cstring = MQTT_CLIENT_ID;
		  data.username.cstring = MQTT_USER;
		  data.password.cstring = MQTT_PASSWORD;
		  HAL_Delay(100);
		  if(setupNetwork(SSID_WIFI, PASS_WIFI)){
			  printf("setup network failed\r\n");
		  }
		  HAL_Delay(100);
		  if(NetworkInit(&network))
			  printf("init failed\r\n");
		  else
			  printf("network up\r\n");
		  HAL_Delay(100);
		  ret = isTCPPortOpen();
		  if(ret >= 0){
			  amw_write("close 0\r\n");
			  i = getCmdResponse(uart_reponse);
			  printf("%d, %s\r\n", i, uart_reponse);
		  }
		  HAL_Delay(100);
		  ret = NetworkConnect(&network, MQTT_HOST, MQTT_PORT);
		  if(ret == FAILURE)
		  {
			  // error, return
			  printf("network connect tcp failed\r\n");
			  HAL_Delay(100);
			  amw_write("close 0\r\n");
			  i = getCmdResponse(uart_reponse);
			  printf("%d, %s\r\n", i, uart_reponse);
			  return ret;
		  }

		  printf("network connect tcp success\r\n");
		  HAL_Delay(250);

		  MQTTClientInit(&client1, &network, 10000, send_buf, sizeof(send_buf), recv_buf, sizeof(recv_buf));
		  HAL_Delay(250);

		  ret = MQTTConnect(&client1, &data);
		  if(ret != SUCCESS)
		  {
			  // error, return
			  printf("failed to connect mqtt\r\n");
			  NetworkDisconnect(&network);
			  return ret;
		  }
		  printf("success to connect mqtt\r\n");

}
void Subcribes_MQTT(void){
	MQTTSubscribe(&client1, MQTT_SUB_TOPIC, QOS1, receive_handler);
	HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_DEVICE_ID, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_AIR_QUALITY, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_CONTROL_STATE, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_POWER_ON, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_POWER_OFF, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_NIGHT_MODE, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_ION, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_UV, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_FILTER, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_CONTROL, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_SPEED_LOW, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_SPEED_MED, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, TOPIC_SPEED_HIGH, QOS0, receive_handler);
	 HAL_Delay(250);
	 MQTTSubscribe(&client1, "can", QOS0, receive_handler);
	 printf("finish subcribe\r\n");

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  //khởi tạo kết nối cho IR
  my_enableIRIn();
  printf("test\r\n");
   //testMQTT();
  // Khởi tạo kết nối đến cảm biến
   //i2c_init_sensirion(&hi2c2);
   //SM_UART_04L_Init(&huart6);
   //err = sgp30_iaq_init();
   //startToRevSM04L();

  // Khởi tạo PWM cho quạt
  	 fan_init();



  // Khởi tạo kết nối MQTT
  	  //init_MQTT();
  	 //initMQTT();



  // Khởi tạo chế độ ban đầu cho hệ thống, ở đây sau này là lấy các biến lưu trong EEPROM
  night_mode = Nightmode_Off;
  speed = low;
  power = Power_On;
  control_mode = Manual;
  filter_mode = Fresh_Air;
  UV_mode = UVon;
  ION_mode = IONoff;
  ion_val = 0;
  HAL_GPIO_WritePin(UV_GPIO_Port, UV_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Ion_GPIO_Port, Ion_Pin, GPIO_PIN_RESET);
  //fan_12_run(lowspeed);
  xiaomi_fan_run(LOW_SPEED);
  //Subcribes_MQTT();

  // subcribeMQTT();


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensor_Task, osPriorityAboveNormal, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControl_Task, osPriorityNormal, 0, 128);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of DisplayTask */
  osThreadDef(DisplayTask, StartDisplay_task, osPriorityBelowNormal, 0, 128);
  DisplayTaskHandle = osThreadCreate(osThread(DisplayTask), NULL);

  /* definition and creation of ServerTask */
  osThreadDef(ServerTask, StartServer_task, osPriorityAboveNormal, 0, 128);
  ServerTaskHandle = osThreadCreate(osThread(ServerTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  MQTTYield(&client, 500);
	  }
	  printf("test mqtt disconnect\r\n");
	  MQTTDisconnect(&client);
	  NetworkDisconnect(&network);
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UV_Pin|Ion_Pin|Damperin3_Pin|Damperin4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Damperout1_Pin|Damperout2_Pin|Damperout3_Pin|Damperout4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Damperin1_Pin|Damperin2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : UV_Pin Ion_Pin Damperin3_Pin Damperin4_Pin */
  GPIO_InitStruct.Pin = UV_Pin|Ion_Pin|Damperin3_Pin|Damperin4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Damperout1_Pin Damperout2_Pin Damperout3_Pin Damperout4_Pin */
  GPIO_InitStruct.Pin = Damperout1_Pin|Damperout2_Pin|Damperout3_Pin|Damperout4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Damperin1_Pin Damperin2_Pin */
  GPIO_InitStruct.Pin = Damperin1_Pin|Damperin2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : recive_IR_Pin */
  GPIO_InitStruct.Pin = recive_IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(recive_IR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t value;
	if (htim->Instance==TIM2)
	{
		i++;
		if(i==1)
		{
//			value1 = __HAL_TIM_GetCounter(&htim2);    //read TIM2 channel 1 capture value
			__HAL_TIM_SetCounter(&htim2, 0);    //reset counter after input capture interrupt occurs
		}
		if(i==2)
		{
			i = 0;
			value = __HAL_TIM_GetCounter(&htim2);    //read TIM2 channel 1 capture value
			__HAL_TIM_SetCounter(&htim2, 0);    //reset counter after input capture interrupt occurs
			speed = value*60/15;
			HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensor_Task */
/**
  * @brief  Function implementing the SensorTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartSensor_Task */
void StartSensor_Task(void const * argument)
{
    
    
    
    
    
    

  /* USER CODE BEGIN 5 */
	//svm30 SVM30;
	//getSVM30(&SVM30);
	//tvoc_ppb = SVM30.tvoc_ppb;
	//co2_ppm  = SVM30.co2_eq_ppm;
	//temperature = SVM30.temperature;
	//humidity = SVM30.humidity;
	//startToRevSM04L();
	//PM25 = amphenol1.PM2p5_Standard;
	//Convertvalue();
	//sprintf(AQI,"%d.%d-%d.%d-%d-%d-%d",tmpIntte1,tmpIntte2,tmpInthu1,tmpInthu2,PM25,tvoc_ppb,co2_ppm);
	//checkState();

  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(500);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartControl_Task */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControl_Task */
void StartControl_Task(void const * argument)
{
  /* USER CODE BEGIN StartControl_Task */
	char trans_str[96] = {0,};
	 static char *decode_str[] = {"UNUSED", "UNKNOWN", "RC5", "RC6", "NEC", "SONY", "PANASONIC", "JVC", "SAMSUNG", "WHYNTER", "AIWA_RC_T501", "LG", "SANYO", "MITSUBISHI", "DISH", "SHARP", "DENON", "PRONTO"};
	// if (Vanst == 1){
	//	 Vanst = 3;
	//	 directionOfRotation(false, 220);
	//}
	//else if(Vanst == 0){
	//	Vanst = 3;
	//	directionOfRotation(true, 220);
	//}
  /* Infinite loop */
  for(;;)
  {
	  if(my_decode(&results))
		{

			snprintf(trans_str, 96, "Cod: %p | Type: %s | Bits: %d\n", (void*)results.value, decode_str[results.decode_type + 1], results.bits);
			HAL_UART_Transmit(&huart2, (uint8_t*)&trans_str, sizeof(trans_str), 100);
			//printf("%s",trans_str);
			if ((void*)results.value == 0xffb04f ){
				SwitchIR++;
				flagSW = 1;
				if (SwitchIR == 3){
					SwitchIR = 1;
				}
			}
			if ((void*)results.value == 0xff22dd ){
				SpeedIR++;
				flagSP = 1;
				if (SpeedIR == 4){
					SpeedIR = 1;
				}
			}
			if ((void*)results.value == 0xff7887 ){
				ModeIR++;
				flagMode = 1;
				if (ModeIR == 4){
					ModeIR = 1;
				}
			}
			if ((void*)results.value == 0xff8877  ){
				IonIR++;
				flagIon = 1;
				if (IonIR == 3){
					IonIR = 1;
				}
			}
			Control_IR();

			my_resume();
		}

    osDelay(500);
  }
  /* USER CODE END StartControl_Task */
}

/* USER CODE BEGIN Header_StartDisplay_task */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplay_task */
void StartDisplay_task(void const * argument)
{
  /* USER CODE BEGIN StartDisplay_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
  }
  /* USER CODE END StartDisplay_task */
}

/* USER CODE BEGIN Header_StartServer_task */
/**
* @brief Function implementing the ServerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServer_task */
void StartServer_task(void const * argument)
{
  /* USER CODE BEGIN StartServer_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
  }
  /* USER CODE END StartServer_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	user_TIM_PeriodElapsedCallback(htim);
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
