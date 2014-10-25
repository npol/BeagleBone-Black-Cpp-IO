/**************************************
 * Mobot Tutorial 2014
 * BeagleBone Black I/O Starter Code
 * Nishant Pol (npol)
 *
 * Included here are functions to
 * write to pins as digital I/O
 * and pwm.
 * To control a motor using a motor
 * driver, you use two signal lines,
 * one for direction (using digital I/O),
 * and one for speed (using pwm).
 *
 * These functions use the terminal interface
 * to communicate with I/O drivers via files.
 * For more information:
 * https://www.youtube.com/watch?v=s9tnTcQlTDY
 * https://groups.google.com/forum/#!msg/beagleboard/wjbOVE6ItNg/Dym4H4HuI8gJ
 *
 * Important Note about pins:
 * The BeagleBone Black has two header blocks
 * P8 and P9, as well as onboard LEDs, which
 * you can control with these functions.
 * A pin has two references, one for the functions
 * below, and one for the header.  You should
 * Use the tables to map header designation
 * to pin number.  For example, the pin with
 * header designation P9_12 is pin 60.
 * Tables are at:
 * https://github.com/derekmolloy/boneDeviceTree/tree/master/docs
 *
 * To Compile (~/Desktop may be different):
 * root@beaglebone:~/Desktop# g++ mobotIO.cpp -o io
 * To run
 * root@beaglebone:~/Desktop# ./io
 *************************************/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
using namespace std;

/** Function Prototypes
 ** Keep these at the top of your file
 **/

/* Set pin to digital input or output
 * bank:
 * pin:
 * mode: 0 is input, 1 is output
 */
void pinMode(unsigned char pin,signed char mode);

/* Write high/low to pin
 * Output will change only if pinMode(,,1)
 * has been called prior
 */
void digitalWrite(unsigned char pin,unsigned char value);


void pwmInit(unsigned char bank, unsigned char pin);

/* Write PWM value to pin
 * Output will change only if pwmInit()
 * has been called prior
 */
void pwmWrite(unsigned char bank, unsigned char pin, unsigned long int period, unsigned long int duty);
/** Main function
 ** This is where all you magic happens.
 ** The sample code will turn on the LED for 4sec,
 ** then turn off the LED for 4sec.  It then frees the pin
 ** Then, the LED flashes on and off using PWM for 50%, 20%, and 90%
 **/

int main(){
    pinMode(50,1);      //Set P9_14 as output
    digitalWrite(50,1); //Set P9_14 high
    sleep(4);           //Wait for 1sec
    digitalWrite(50,0); //Set P9_14 low
    sleep(4);           //Wait for 1sec
    pinMode(50,-1);     //Free P4_12
    sleep(4);

    pwmInit(9,14);                          //Initialize pin for PWM
    /*Note: In current implementation, once pin has been assigned
     * to PWM, it cannot be changed back to I/O.  A workaround is
     * to set duty = 0 for high and duty = period for low.
     * Also, pwmInit cannot be called more than once after bootup.
     */
    pwmWrite(9,14,1000000000,500000000);    //50% on
    sleep(4);
    pwmWrite(9,14,1000000000,800000000);    //20% on
    sleep(4);
    pwmWrite(9,14,1000000000,100000000);    //90% on
    return 0;
}

/** Function Implementations
 ** Keep these at the bottom of your file,
 ** or in another file.  You shouldn't have
 ** to modify these unless there is a bug
 **/

void pinMode(unsigned char pin,signed char mode){
    char buffer[128];
    if(mode < 0){//If we wanted to free pin
        //Set pin as input to avoid errors
        snprintf(buffer,128,"echo \"in\" > /sys/class/gpio/gpio%d/direction",pin);
        system(buffer);
        //Free pin
        snprintf(buffer,128,"echo %d > /sys/class/gpio/unexport",pin);
        system(buffer);
        return;
    }
    //Make the pin available to us
    snprintf(buffer,128,"echo %d > /sys/class/gpio/export",pin);
    system(buffer);
    //Set direction of pin
    if(mode == 0){//Set input
        snprintf(buffer,128,"echo \"in\" > /sys/class/gpio/gpio%d/direction",pin);
        system(buffer);
    } else {//Set output
        snprintf(buffer,128,"echo \"out\" > /sys/class/gpio/gpio%d/direction",pin);
        system(buffer);
    }
    return;
}

void digitalWrite(unsigned char pin,unsigned char value){
    char buffer[128];
    if(value){
        snprintf(buffer,128,"echo 1 > /sys/class/gpio/gpio%d/value",pin);
        system(buffer);
    }
    else{
        snprintf(buffer,128,"echo 0 > /sys/class/gpio/gpio%d/value",pin);
        system(buffer);
    }
    return;
}

void pwmInit(unsigned char bank, unsigned char pin){
    char buffer[128];
    printf("A\n");
    snprintf(buffer,128,"echo am33xx_pwm > /sys/devices/bone_capemgr.8/slots");
    system(buffer);
    printf("B\n");
    snprintf(buffer,128,"echo bone_pwm_P%d_%d > /sys/devices/bone_capemgr.8/slots",bank,pin);
    system(buffer);
    printf("C\n");
    return;
}

void pwmWrite(unsigned char bank, unsigned char pin, unsigned long int period, unsigned long int duty){
    char buffer[128];
    snprintf(buffer,128,"echo %d > /sys/devices/ocp.2/pwm_test_P%d_%d.%d/period",period,bank,pin,pin);
    system(buffer);
    snprintf(buffer,128,"echo %d > /sys/devices/ocp.2/pwm_test_P%d_%d.%d/duty",duty,bank,pin);
    system(buffer);
    return;
}




