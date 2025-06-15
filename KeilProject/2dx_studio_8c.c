
//Final Project Code Submission - Muhammad Haseeb Aslam - 400449291

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"


//---------- Preprocessor Directives & Global variables ----------

#define I2C_MCS_ACK 0x00000008 // Data Acknowledge Enable
#define I2C_MCS_DATACK 0x00000008 // Acknowledge Data
#define I2C_MCS_ADRACK 0x00000004 // Acknowledge Address
#define I2C_MCS_STOP 0x00000004 // Generate STOP
#define I2C_MCS_START 0x00000002 // Generate START
#define I2C_MCS_ERROR 0x00000002 // Error
#define I2C_MCS_RUN 0x00000001 // I2C Master Enable
#define I2C_MCS_BUSY 0x00000001 // I2C Busy
#define I2C_MCR_MFE 0x00000010 // I2C Master Function Enable

#define MAXRETRIES 5 // number of sttempts

#define STEPS 32 // amount of steps per rotation. 
#define ROTATIONS 3 // amount of rotations

uint16_t dev = 0x29; 
uint8_t ToFMode = 1;
uint8_t dataReady; 
int status = 0;

uint16_t values[STEPS * ROTATIONS]; 
char float_values[STEPS * ROTATIONS]; 

int motorSteps[] = {
    0b00000011,
    0b00000110,
    0b00001100,
    0b00001001
};
int rev_motorSteps[] = {
    0b00001001,
    0b00001100,
    0b00000110,
    0b00000011
};
void I2C_Init(void) {
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0; // activate I2C0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // activate port B
    while ((SYSCTL_PRGPIO_R & 0x0002) == 0) {}; // ready?

    GPIO_PORTB_AFSEL_R |= 0x0C; // 3) enable alt funct on PB2,3       
    GPIO_PORTB_ODR_R |= 0x08; // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C; // 5) enable digital I/O on PB2,3
   
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200; //TED
    I2C0_MCR_R = I2C_MCR_MFE; // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011; 
    

}

void init_portM() {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; //enable clock and wait for it to load
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0) {

    };

    //ditigal enable, set to output
    GPIO_PORTM_DIR_R = 0b00000001; // Enable PM0 as output.
    GPIO_PORTM_DEN_R = 0b00000001; // Enable PM0 as digital pin.
    return;
}

void PortJ_Init(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8; // Activate clock for Port J
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0)
    {
    };                         // Allow time for clock to stabilize
    GPIO_PORTJ_DIR_R &= ~0x02; // Make PJ1 input
    GPIO_PORTJ_DEN_R |= 0x02;  // Enable digital I/O on PJ1

    GPIO_PORTJ_PCTL_R &= ~0x000000F0; //  Configure PJ1 as GPIO
    GPIO_PORTJ_AMSEL_R &= ~0x02;      //  Disable analog functionality on PJ1
    GPIO_PORTJ_PUR_R |= 0x02;         //	Enable weak pull up resistor on PJ1
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void) {
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6; // activate clock for Port N
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0) {}; // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00; // make PG0 in (HiZ)
    GPIO_PORTG_AFSEL_R &= ~0x01; // disable alt funct on PG0
    GPIO_PORTG_DEN_R |= 0x01; // enable digital I/O on PG0
    // configure PG0 as GPIO
    //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
    GPIO_PORTG_AMSEL_R &= ~0x01; // disable analog functionality on PN0

    return;
}

//Initialize motor port
void init_portH() {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; //enable clock and wait for it to load
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0) {

    };

    //ditigal enable, set to output
    GPIO_PORTH_DIR_R = 0b00001111; 
    GPIO_PORTH_DEN_R = 0b00001111; 
    return;
}

void VL53L1X_XSHUT(void) {
    GPIO_PORTG_DIR_R |= 0x01; // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110; //PG0 = 0
    FlashLED4(1); 
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01; // make PG0 input (HiZ)

}

// Function to collect data from ToF sensor
uint16_t measure_distance() {
    uint16_t Distance = 0;
    uint8_t dataReady = 0;
		uint8_t RangeStatus = 0;

    while (dataReady == 0) {
        status = VL53L1X_StartRanging(dev);
        status = VL53L1X_CheckForDataReady(dev, & dataReady);

        VL53L1_WaitMs(dev, 2);
        VL53L1X_StopRanging(dev);
        status = VL53L1X_ClearInterrupt(dev);

    } 
    dataReady = 0;

		status = VL53L1X_GetRangeStatus(dev, &RangeStatus); // make sure status ok
    status = VL53L1X_GetDistance(dev, & Distance); // Get distance measurement 

    status = VL53L1X_ClearInterrupt(dev);

    return Distance;

}

// Function to spin motor
void rotate_m(uint16_t * values) { 
	  int counter = 0; 
    int direction = 0;

    for (int i = 0; i < ROTATIONS; i++) { // how many rotations to spin for
        for (int x = 0; x < STEPS; x++) { //how many steps per rotation

            for (int j = 0; j < (2048 / (STEPS * 4)); j++) {
                for (int k = 0; k < 4; k++) {

                    if (direction) { //forward rotation
                        GPIO_PORTH_DATA_R = rev_motorSteps[k];
                    } else { //backwards rotation
                        GPIO_PORTH_DATA_R = motorSteps[k];
                    }

                    SysTick_Wait10ms(1);
                }
            }

            values[counter] = measure_distance(); 
            FlashLED2(1); //flash PN0 for every measurement taken
            SysTick_Wait10ms(1);
            counter++;

        } 

		//flash led 3 to incicate user to take a step
				for(int i=0;i<5;i++){
					FlashLED3(1); //PF4
					SysTick_Wait10ms(50);
					direction = !direction;
				}
        
    }
}

//function to verify bus speed during demo
void measure_CLKspeed(){
    while(1){
        GPIO_PORTM_DATA_R ^= 1;
        SysTick_Wait10ms(1);
    }
}

int main(void) {

    uint8_t dataReady;
    uint8_t sensorState = 0;

    int start = 0; //boolean to start measurements
    int rotate = 0;

    //initialize functions
    PLL_Init();
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
    PortJ_Init();
    init_portH();
		init_portM();

    //This function only measures bus speed. Uncomment only for that purpose!
	  //measure_CLKSpeed();

    // Wait for ToF Sensor to turn on
    while (sensorState == 0) {
        status = VL53L1X_BootState(dev, & sensorState);
        SysTick_Wait10ms(10);
    }
    UART_printf("ToF Initialized.\r\n");

    status = VL53L1X_ClearInterrupt(dev);

    //Initialize sensor with predefined mode
    status = VL53L1X_SensorInit(dev);
    Status_Check("SensorInit", status);

    //Start ranging of ToF sensor
    status = VL53L1X_StartRanging(dev);
    Status_Check("StartRanging", status);

    //Check if letter 'e' was recieved
    while (1) {
        start = UART_InChar();
			
        if (start == 101) {
            FlashLED1(1);//PN1 flashed 
            start = 0;
            break; 
        }
    }
    while(1) {
        rotate = GPIO_PORTJ_DATA_R;
        if(!rotate){
            break;
        }
    }

    rotate_m(values); 

    for (int i = 0; i < (int)(ROTATIONS * (STEPS)); i++) {
        sprintf(float_values, "%u\r\n", values[i]);
        UART_printf(float_values);
    }
    UART_printf("x\r\n");
    VL53L1X_StopRanging(dev);

}