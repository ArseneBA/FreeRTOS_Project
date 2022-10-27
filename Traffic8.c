/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"


/* USER CODE BEGIN Includes */
#include <stdio.h>                    /* standard I/O .h-file                */
#include <ctype.h>                    /* character functions                 */
#include <string.h>                   /* string and memory functions         */
#include <stdlib.h>
#include <stdbool.h>
#include "tim.h"
#include "stm32f1xx_hal_tim.h"
#include "LCD.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ESC				0x1B							/* Caract?re Escape */
#define M_MAJ			0x4d							/* Caract?re M*/
#define M_MIN			0x6d							/* Caract?re m*/
#define N_MAJ			0x4e							/* Caract?re N*/
#define N_MIN			0x6e							/* Caract?re n*/
#define CTRL_Q     0x11                             // Control+Q character code
#define CTRL_S     0x13                             // Control+S character code
#define DEL        0x7F
#define BACKSPACE  0x08
#define CR         0x0D
#define LF         0x0A


const char menu[] = 
 
   "\n"
   "+**************         CONTROLLEUR DE FEUX            ********+\n"
   "|                                                              |\n"
   "+ command -+ syntax -----+ function ---------------------------+\n"
   "| ESC      |             | entree commandes                    |\n"
   "|          |             |                                     |\n"
   "| Horloge  | H hh:mm:ss  | mise a jour horloge                 |\n"
   "| Debut    | D hh:mm:ss  | mise a jour debut controle feux     |\n"
   "| Fin      | F hh:mm:ss  | mise a jour fin controle feux       |\n"
	 "|          |             |                                     |\n"
	 "| M	       |             | test des feux en mode manuel        |\n"
   "+----------+-------------+-------------------------------------+\n";
const char mode_manuel[] = 
	   
	 "\n"
   "+**************             MODE MANUEL                ********+\n"
   "|                                                              |\n"
   "+ command -+ syntax -----+ function ---------------------------+\n"
   "| N        |             | prochaine phase                     |\n"
   "+----------+-------------+-------------------------------------+\n";


struct temps {
	volatile unsigned int			 heure;
	volatile unsigned int			 min;
	volatile unsigned int			 sec;
	
};
const char * chaine1 = " Horloge: %02d: %02d: %02d ";
const char * chaine2 = " Debut: %02d:%02d:%02d  / ";
const char * chaine3 = " Fin: %02d:%02d:%02d  Control FEUX \r";

struct print_H {
	const char * chaine;
	unsigned int			 heure;
	unsigned int			 min;
	unsigned int			 sec;
};
	
struct print_H aff_H;

 struct temps debut = 	{ 6, 0, 0 };
 struct temps fin   = 	{ 18, 0, 0 };
 struct temps horloge = 	{ 12, 0, 0 };
struct temps v_temps;

volatile bool DPV1;
volatile bool DPV2;
volatile bool detect1;
volatile bool	detect2;
volatile bool train1 = 0;
volatile bool train2 = 0;
volatile bool train = 0;
volatile bool mess_train_nuit = 0;
volatile int cpt_top;
char ph;
char task;
bool escape;
bool capteur;
bool flag_manuel = false;
unsigned char c;
/* USER CODE END Includes */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void controleur( void *pvParameters );
void command  (void *pvParameters);
void lecture_BP (void *pvParameters);
void barriere (void *pvParameters);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void sequenceur (bool);
bool generation_temps ( void) ;
bool lect_H (char  * );
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
QueueHandle_t xRxQueue;
QueueHandle_t xTxQueue; //Permet de communiquer le temps entre la fonction g?n?ration temps et la fonction commande, la queue est de taille 1 fois le message de temps complet la donn?e temps donc on ne doit pas g?rer la queue m?me si on ne lit pas la donn?e
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	lcd_init();
	lcd_clear();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
		xRxQueue = xQueueCreate( 16, sizeof( char ) );
		xTxQueue = xQueueCreate( 3, sizeof( struct print_H) );
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
	HAL_UART_Receive_IT( &huart1, &c, 1);
	

  /* USER CODE BEGIN 2 */
	MX_TIM2_Init();
	
	cpt_top = 0;
	
  xTaskCreate(      controleur,       /* Function that implements the task. */
                    "CONTROLEUR",          /* Text name for the task. */
                    128,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+2,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */
										
	xTaskCreate(      command,       /* Function that implements the task. */
                    "COMMAND",          /* Text name for the task. */
                    128,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */				

	xTaskCreate(      lecture_BP,       /* Function that implements the task. */
                    "LECT_BP",          /* Text name for the task. */
                    128,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+3,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */
										
	xTaskCreate(      barriere,       /* Function that implements the task. */
                    "BARRIERE",          /* Text name for the task. */
                    128,      /* Stack size in words, not bytes. */
                    NULL,    /* Parameter passed into the task. */
                    tskIDLE_PRIORITY+1,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3|TIM_CHANNEL_4);  //Active le PWM pour le moteur
	
	
  /* USER CODE END 2 */

  
 

  /* Start scheduler */
   	vTaskStartScheduler();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
/****************************************************************************/
/*       		 TACHE  MEMORISATION  BOUTON POUSSOIR PIETON			    */
/****************************************************************************/
void lecture_BP  (void *pvParameters) {  

	TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
  while ( 1 ) {
		
  	if ( HAL_GPIO_ReadPin( GPIOB, DPV1_Pin ))  DPV1 = 1;
																												// memorisation appui BP1 	
		if ( HAL_GPIO_ReadPin( GPIOB, DPV2_Pin ) )	DPV2 = 1;
						 					                           // Scrutation des touches toutes les 50ms
		train1 = HAL_GPIO_ReadPin( GPIOC, train1_Pin );
		train2 = HAL_GPIO_ReadPin( GPIOC, train2_Pin );
		train = train1 || train2;
		vTaskDelayUntil( &xLastWakeTime, 50/ portTICK_PERIOD_MS);

  }
}

/****************************************************************************/
/*       				FONCTION 	SEQUENCEUR								*/
/****************************************************************************/
void sequenceur (bool valid_seq) { 
  
static char cpt;


	if ( valid_seq ) {

		switch  ( ph ) {

		case 1: 
		{		
				HAL_GPIO_WritePin(GPIOB,R1_Pin|O1_Pin|R2_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, V1_Pin|S1_Pin|V2_Pin|O2_Pin|S2_Pin, 0);
				cpt= 0;
				ph = 2;
				DPV1 = DPV2 = detect1 = detect2 = 0;	
		}
		break; 

		case 2:
		{
				 HAL_GPIO_WritePin(GPIOB,V1_Pin|S2_Pin|R2_Pin, 1);
				 HAL_GPIO_WritePin(GPIOB,R1_Pin|O1_Pin|V2_Pin|O2_Pin|S1_Pin, 0);
				if (( ++cpt > 8 ) || DPV1 || ( detect2&&!detect1&&!DPV2 ) || flag_manuel || train) ph = 3;
				//S'il y a un train, on va au feu rouge pour les voitures passant par le train
				//Si l'on est en mode manuelle, on ne fait plus attention au timer et on change de phase seulement quand on appuie sur n
		}
		break;

		case 3:
		{		
				HAL_GPIO_WritePin(GPIOB, O1_Pin|R2_Pin, 1);
				HAL_GPIO_WritePin (GPIOB, R1_Pin|V1_Pin|S1_Pin|V2_Pin|O2_Pin|S2_Pin, 0);
				ph = 4;			
		}
		break;

		case 4:
		{		
				HAL_GPIO_WritePin (GPIOB, R1_Pin|R2_Pin|O2_Pin, 1);
				HAL_GPIO_WritePin (GPIOB, V1_Pin|O1_Pin|S1_Pin|V2_Pin|S2_Pin, 0);
				cpt = 0;
				ph = 5;	
				DPV1 = DPV2 = detect1 = detect2 = 0;
			  if (train) HAL_GPIO_WritePin (GPIOB, S1_Pin, 1);
		}
		break;

		case 5:
		{
				HAL_GPIO_WritePin (GPIOB, R1_Pin|V2_Pin|S1_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, V1_Pin|O1_Pin|S2_Pin|R2_Pin|O2_Pin, 0);
				if (!train)
				{	
					if (( ++cpt > 8 ) || DPV2 || ( detect1&&!detect2&&!DPV1 ) || flag_manuel) ph = 6;
					//Si l'on est en mode manuelle, on ne fait plus attention au timer et on change de phase seulement quand on appuie sur n
				}
				else
				{
					if (DPV2) ph = 6;
				}
		}
		break;

		case 6:
		{
				HAL_GPIO_WritePin (GPIOB, R1_Pin|O2_Pin, 1);
				HAL_GPIO_WritePin (GPIOB, V1_Pin|O1_Pin|S1_Pin|R2_Pin|V2_Pin|S2_Pin, 0);
				if (train)
				{					
					ph = 7;
					HAL_GPIO_WritePin (GPIOB, S1_Pin, 1);
				}
				else ph = 1;
		}
		break;
		
		case 7:
		{
				HAL_GPIO_WritePin (GPIOB, R1_Pin|R2_Pin|S1_Pin|S2_Pin, 1);
				HAL_GPIO_WritePin (GPIOB, V1_Pin|O1_Pin|O2_Pin|V2_Pin, 0);
				if (( ++cpt > 8 ) || (detect2&&!DPV2) || flag_manuel) ph = 4;
				//Les pi?tons ont 8 sec pour passer
				//S'il y a des voitures mais pas de pi?tons on remet le feu des voitures au vert
				//Si on est en manuel on passe d'une phase ? l'autre manuellement
		}
		break;
		}
	}

	else {
		HAL_GPIO_WritePin (GPIOB, R1_Pin|V1_Pin|S1_Pin|R2_Pin|V2_Pin|S2_Pin, 0);
    HAL_GPIO_TogglePin(GPIOB,O1_Pin);
		HAL_GPIO_TogglePin(GPIOB, O2_Pin);
	}
}

/****************************************************************************/
/*               	TACHE 	BARRIERE							*/
/****************************************************************************/
void barriere( void *pvParameters )
{
	int etat_barriere = 0;  //0 levee, 1 en mouvement, 2 baiss?e
	int flag_attente = 0;
	unsigned int TIMCounter, TIMtest;
	
	TickType_t xLastWakeTime;
	// Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		//printf("Compteur %d", TIMtest);
		if (train && etat_barriere == 0 && flag_attente == 1)
		{
			cpt_top = 0;
			__HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_4, 15000 ); //Clock apb1 36MHZ -> ARR = 35999 -> 1KHz
			__HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_3, 0 );
			etat_barriere = 1;
			flag_attente = 0;
		}
		else if (!train && etat_barriere == 2)
		{
			cpt_top = 0;
			__HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_4, 15000 );
			__HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_3, 0 );
			etat_barriere = 1;
		}
		else if(etat_barriere == 1)
		{
			if (cpt_top >= 600)
			{		
				__HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_3 | TIM_CHANNEL_4, 0 );
				if (train) etat_barriere = 2;
				else etat_barriere = 0;
			}
		}
		if (train && etat_barriere == 0 && flag_attente == 0)
		{
			flag_attente = 1;
			vTaskDelayUntil(&xLastWakeTime, 5000/ portTICK_PERIOD_MS);
		}
	}
}
/****************************************************************************/
/*               	FONCTION 	GENERATION TEMPS							*/
/****************************************************************************/
bool generation_temps ( void) { 
/* Fonction g?n?rant le temps
*/
	bool valid_seq;
	
	
	// Indentation du temps
    					                /* clock is an endless loop           */
    if (++horloge.sec == 60)  {         /* calculate the second               */
      horloge.sec = 0;
      if (++horloge.min == 60)  {       /* calculate the minute               */
        horloge.min = 0;
        if (++horloge.heure == 24)  {    /* calculate the hour                 */
          horloge.heure = 0;
        }
      }
    }
  
	aff_H.chaine = chaine1;
  aff_H.heure = horloge.heure;
	aff_H.min = horloge.min;
	aff_H.sec = horloge.sec;		
  if (!xQueueSend( xTxQueue, ( void * ) &aff_H, 100 / portTICK_PERIOD_MS ))  xQueueReset( xTxQueue );
	aff_H.chaine = chaine2;
  aff_H.heure = debut.heure;
	aff_H.min = debut.min;
	aff_H.sec = debut.sec;		
  if ( !xQueueSend( xTxQueue, ( void * ) &aff_H, 100 / portTICK_PERIOD_MS )) xQueueReset( xTxQueue );
	aff_H.chaine = chaine3;
  aff_H.heure = fin.heure;
	aff_H.min = fin.min;
	aff_H.sec = fin.sec;		
  if (!xQueueSend( xTxQueue, ( void * ) &aff_H, 100 / portTICK_PERIOD_MS )) xQueueReset( xTxQueue );

           
   
  	if (memcmp (&debut, &fin, sizeof ( struct temps)) < 0)  {
    	if (memcmp (&debut, &horloge, sizeof ( struct temps)) < 0  &&
        	memcmp (&horloge, &fin,   sizeof ( struct temps)) < 0) {
						valid_seq = 1;
						if (mess_train_nuit) {
							lcd_clear();
							mess_train_nuit = 0;
						}
					}	
			else {
				valid_seq = 0;
				lcd_clear();
				set_cursor(0,0);
				lcd_print("Train non");
				set_cursor(0,1);
				lcd_print("autorise");
				mess_train_nuit = 1;
			}        
		}		
  	else  {
    	if (memcmp (&fin,   &horloge, sizeof (debut)) > 0  &&
        	memcmp (&horloge, &debut, sizeof (debut)) > 0) {
						valid_seq = 1;
						if (mess_train_nuit) {
							lcd_clear();
							mess_train_nuit = 0;
						}
					}
		else {
			valid_seq = 0;
			lcd_clear();
			set_cursor(0,0);
			lcd_print("Train non");
			set_cursor(0,1);
			lcd_print("autoris?");
			mess_train_nuit = 1;
			}
  	}

	return ( valid_seq );
} 

           
/****************************************************************************/
/*     FONCTION lecture commande et convertion en structure temps  			*/
/****************************************************************************/
bool lect_H (char  *buffer)  {
  signed char args;                          	     	/* number of arguments       */

  v_temps.sec = 0;                               	/* preset second             */
  args = sscanf (buffer, "%d:%d:%d",        		/* scan input line for       */
                 &v_temps.heure,                  	/* hour, minute and second   */
                 &v_temps.min,
                 &v_temps.sec);
  
  if (v_temps.heure > 23  ||  v_temps.min > 59  ||  /* check for valid inputs    */
      v_temps.sec > 59   ||  args < 2        ||  args == EOF)  {
	  
    printf ("\n*** ERROR: INVALID TIME FORMAT\n");
	 
    return (0);
  }
  return (1);
}

/****************************************************************************/
/*       				TACHE	CONTROLEUR									*/
/****************************************************************************/
void controleur  (void *pvParameters) {
	//Permet de lancer le sequencer et la g?n?ration du temps
  
	TickType_t xLastWakeTime;
	
	
 	ph = 1;
	// Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

 	while (1) {
		if (!flag_manuel)
			sequenceur(generation_temps());
		else
			generation_temps();
		vTaskDelayUntil( &xLastWakeTime, 1000/ portTICK_PERIOD_MS);
 	}
}

/****************************************************************************/
/*       				TACHE	GESTION  DES COMMANDES						*/
/****************************************************************************/
void command  (void *pvParameters) {                  
  
	char cmde[16];						// en RAM interne pour acc?s rapide 
	char	c;						// PQ avoir deux variables qui ont le m?me non
	char cnt,i = 0;
  struct print_H aff;
	
		printf ( menu);
  	while (1)  {   
		 
		if 	( xQueueReceive( xTxQueue, &aff, 100 / portTICK_PERIOD_MS )) {
			
			printf( aff.chaine, aff.heure, aff.min, aff.sec );
		}
			
		if ( xQueueReceive( xRxQueue, &c, 100 / portTICK_PERIOD_MS )) {
		//Si qq chose ? ?t? envoy? sur la queue (dans HAL_UART_RxCpltCallback) on rentre dans la fonction (xQueueReceive return pdTrue s'il y a qq chose dans la queue
			if ( c == ESC ) {  		
				printf ( "\n\rCommandes :  ");	
				cnt= 0;
				do {
					// On lit au moins une fois la commande qui sera envoy?e sur l'UART
					xQueueReceive( xRxQueue, &c, portMAX_DELAY );
					cmde[cnt++] = c;
				} while ( c != CR );
					cmde[--cnt]= 0; // marquage fin de chaine
				
				task = 0xff; 	
			
				for ( i=0; cmde[i] !=0; i++ )	{		// convertion en Majuscule 
					cmde[i] = toupper(cmde[i]);
				}

				for ( i=0; cmde[i] == ' ';i++ );			// suppression des espaces 


			switch ( cmde [i] )	{

	 
				case 'H':   
																							// Set Time Command          
							if (lect_H (&cmde[i+1]))  {        			// read time input and       
									horloge.heure = v_temps.heure;            // store in 'ctime'          
									horloge.min  = v_temps.min;
									horloge.sec  = v_temps.sec;
					}
				break;

					case 'F':                                		// Set End Time Command     
							if ( lect_H (&cmde[i+1]))  {        		// read time input and       
									fin.heure = v_temps.heure;               // store in 'end'           
									fin.min  = v_temps.min;
									fin.sec  = v_temps.sec;
							 }
						break;

						case 'D':                                // Set Start Time Command    
							if ( lect_H (&cmde[i+1]))  {        // read time input and       
									debut.heure = v_temps.heure;             // store in 'start'       
									debut.min   = v_temps.min;
									debut.sec   = v_temps.sec;
						}
						break;
				}   
			}

			if( c == M_MAJ  || c == M_MIN)
			{
				/*On arr?te l'incr?mentation automatique des ?tats dans le controlleur, on fait cela ? l'aide d'un flag.
				Si notre flag est activ?, on rentre en mode manuel, et on ne rentre plus 
				*/
				printf(mode_manuel);
				flag_manuel = true;
				do {
					// On lit au moins une fois la commande qui sera envoy?e sur l'UART
					xQueueReceive( xRxQueue, &c, portMAX_DELAY );
					if (c == N_MAJ || c == N_MIN)
					{
						sequenceur(true);
						c = 0x00;
					}
				} while ( c != M_MAJ  && c != M_MIN);
				flag_manuel = false;
				
			}
		}
	}
}


/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}
  /* USER CODE BEGIN Callback 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
  if ( GPIO_Pin == Voit1_Pin ) detect1 = 1;		   // Capteur pr?sence v?hicule sur voie 1 ON
	if ( GPIO_Pin == Voit2_Pin ) 
		detect2 = 1;							// Capteur pr?sence v?hicule sur voie 2 ON
	if ( GPIO_Pin == top_codeur_Pin ) 
		cpt_top++; 
  
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*On utilise pas de tache permettant de g?rer l'interruption et on envoie directement sur la queue le buffeur de notre UART
	Puis on r?arme l'UART.
	On utilise pas de Handler Task parce qu'on ne met qu'une variable ? jour, hors ici il y a tr?s peu ? faire
	*/
	BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR( xRxQueue, &c, &xHigherPriorityTaskWoken );
		HAL_UART_Receive_IT( &huart1, &c, 1);
}
  /* USER CODE END Callback 1 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
