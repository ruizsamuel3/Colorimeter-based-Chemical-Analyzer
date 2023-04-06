// Jason Losh
//Combination of several of his lab code
//Edited for his Embedded Systems by Sam Ruiz
//Combination of Embedded System Labs
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "clock.h"
#include "uart0.h"
#include "rgb_led.h"
#include "wait.h"
#include "tm4c123gh6pm.h"
#include "adc0.h"
// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

//Step Motor Wires
#define BLACK_MOTOR (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))//PE4
#define WHITE_MOTOR (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))//PE5
#define YELLOW_MOTOR (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))//PE3
#define GREEN_MOTOR (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))//PE2
//PortB Masks
#define AIN11_MASK 32 //at PB5 = 2^5
//PortD Masks for IR
#define FREQ_IN_MASK 1
//PortE Masks for Step Motor
#define BLACK_MOTOR_MASK 16
#define WHITE_MOTOR_MASK 32
#define YELLOW_MOTOR_MASK 8
#define GREEN_MOTOR_MASK 4
// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

//Motor Globals
uint8_t phase;
uint8_t position;
//Test Tube Step Motor Locations
uint8_t tube0 = 194;
uint8_t tube1 = 27;
uint8_t tube2 = 60;
uint8_t tube3 = 94;
uint8_t tube4 = 127;
uint8_t tube5 = 161;
//RGB Globals
uint16_t pwm_r;
uint16_t pwm_g;
uint16_t pwm_b;
uint16_t raw_r;
uint16_t raw_g;
uint16_t raw_b;
//IR Global
bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;
uint32_t Time[50];
uint8_t count = 0;
uint8_t code;
char strCode[32];
char keyName[10];
uint32_t bitCode[32];
bool valid;
//Hardcoded Measure Raw Values  R     G     B     PH
double rawReference[5][4] =    {{2970.0/3029.0, 2760.0/3100.0, 1206.0/3052.0, 6.8}, //R1
                                {2870.0/3029.0, 1734.0/3100.0, 1379.0/3052.0, 7.2}, //R2
                                {3092.0/3029.0, 1386.0/3100.0, 951.0/3052.0, 7.5}, //R3
                                {2635.0/3029.0, 1043.0/3100.0, 1024.0/3052.0, 7.8}, //R4
                                {2889.0/3029.0, 746.0/3100.0, 1262.0/3052.0, 8.2}}; //R5
//IR Sensor Timer
void enableTimerMode()
{
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_NEG;           // measure time from positive edge to positive edge => CHANGED to negative edge
    WTIMER2_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER2_TAV_R = 0;                               // zero counter for first period
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER2A-16-96);         // turn-on interrupt 112 (WTIMER2A)
}
//IR Get Key Name
void getKeyName(){
    switch(code){
    case  4: strcpy(keyName, "JUMP3"); break;
    case  5: strcpy(keyName, "JUMP7"); break;
    case  6: strcpy(keyName, "FADE3"); break;
    case  7: strcpy(keyName, "FADE7"); break;
    case  8: strcpy(keyName, "DIY4"); break;
    case  9: strcpy(keyName, "DIY5"); break;
    case  10: strcpy(keyName, "DIY6"); break;
    case  11: strcpy(keyName, "FLASH"); break;
    case  12: strcpy(keyName, "DIY1"); break;
    case  13: strcpy(keyName, "DIY2"); break;
    case  14: strcpy(keyName, "DIY3"); break;
    case  15: strcpy(keyName, "AUTO"); break;
    case  16: strcpy(keyName, "RDOWN"); break;
    case  17: strcpy(keyName, "GDOWN"); break;
    case  18: strcpy(keyName, "BDOWN"); break;
    case  19: strcpy(keyName, "SLOW"); break;
    case  20: strcpy(keyName, "RUP"); break;
    case  21: strcpy(keyName, "GUP"); break;
    case  22: strcpy(keyName, "BUP"); break;
    case  23: strcpy(keyName, "QUICK"); break;
    case  24: strcpy(keyName, "R5"); break;
    case  25: strcpy(keyName, "G5"); break;
    case  26: strcpy(keyName, "B5"); break;
    case  27: strcpy(keyName, "W5"); break;
    case  28: strcpy(keyName, "R4"); break;
    case  29: strcpy(keyName, "G4"); break;
    case  30: strcpy(keyName, "B4"); break;
    case  31: strcpy(keyName, "W4"); break;
    case  64: strcpy(keyName, "POWER"); break;
    case  65: strcpy(keyName, "SKIP"); break;
    case  68: strcpy(keyName, "W1"); break;
    case  69: strcpy(keyName, "B1"); break;
    case  72: strcpy(keyName, "W2"); break;
    case  73: strcpy(keyName, "B2"); break;
    case  76: strcpy(keyName, "W3"); break;
    case  77: strcpy(keyName, "B3"); break;
    case  80: strcpy(keyName, "R3"); break;
    case  81: strcpy(keyName, "G3"); break;
    case  84: strcpy(keyName, "R2"); break;
    case  85: strcpy(keyName, "G2"); break;
    case  88: strcpy(keyName, "R1"); break;
    case  89: strcpy(keyName, "G1"); break;
    case  92: strcpy(keyName, "BRIGHTUP"); break;
    case  93: strcpy(keyName, "BRIGHTDOWN"); break;
    default : strcpy(keyName, "UNKNOWN"); break;
    }
}
bool isValid(uint32_t count){
    uint8_t i;
    for(i = 1; i < count; i++){
        uint32_t t = (Time[i + 1] - Time[i]);
        if((t >= 33750 && t <= 56250))
        {
            bitCode[i-1]= 0;
            strCode[i-1] = '0'; //creating string to ate bitcode in UART, offset by -1 because weird offset in TIME
        }
        else if((t >= 78750 && t <= 101250))
        {
            bitCode[i-1] = 1;
            strCode[i-1] = '1';
        }
        else{
            if(i != 33){ //all data done and now on stop bit
                return false;
            }
        }
    }
   for(i = 0; i < 8; i++){ //verify address bits
        if(bitCode[i] == bitCode[i+8]){
            return false;
        }
    }
   code = 0;
    for(i = 23; i > 15; i--){
            if(bitCode[i] == bitCode[i+8]){ //verify data bits
                return false;
            }
            code += bitCode[i];
            if(i != 16){
                code = code << 1;
            }
        }
    getKeyName();
    return true;
}

void wideTimer2Isr()
{

    if(WTIMER2_TAV_R > 4000000){
        count = 0;
    }
    if(count == 0){
        WTIMER2_TAV_R  = 0;
        Time[count] = (WTIMER2_TAV_R);
        count++;
    }
    else if(count == 1){
        Time[count] = (WTIMER2_TAV_R);
        if((Time[1] - Time[0]) >= 520000 &&(Time[1] - Time[0]) <= 560000){
            count++;
        }
        else{
            count  = 0;
        }
    }
    else if(count > 1){

        Time[count] = (WTIMER2_TAV_R);
        uint32_t timeData = Time[count] - Time[count -1];
        if( (timeData >= 33750 && timeData <= 56250) || (timeData >= 78750 && timeData <= 101250)){
            count++;
        }
        else{
            count = 0;
        }
    }
    if(count ==34){
                valid = isValid(count);
                count =0;
                WTIMER2_ICR_R = TIMER_ICR_CAECINT;
                uint32_t i;
                for(i = 0; i < 50;i++)
                {
                    Time[i] = 0;
                }

                }
    WTIMER2_ICR_R = TIMER_ICR_CAECINT;

}


//-----------------------------------------------------------------------------
//IR Stuff
// Period timer service publishing latest time measurements every negative edge



//-----------------------------------------------------------------------------
//Motor Stuff
void applyPhase(phaseInput){
    switch(phaseInput){
    case 0: BLACK_MOTOR = 1; WHITE_MOTOR = 0; YELLOW_MOTOR = 0; GREEN_MOTOR = 0; break;
    case 1: BLACK_MOTOR = 0; WHITE_MOTOR = 0; YELLOW_MOTOR = 1; GREEN_MOTOR = 0; break;
    case 2: BLACK_MOTOR = 0; WHITE_MOTOR = 1; YELLOW_MOTOR = 0; GREEN_MOTOR = 0; break;
    case 3: BLACK_MOTOR = 0; WHITE_MOTOR = 0; YELLOW_MOTOR = 0; GREEN_MOTOR = 1; break;
    }
    phase = phaseInput;
}

void stepCcw(){
    position--;
    phase = ((phase - 1) % 4);
    if(phase > 4){
            phase = 3;
        }
    applyPhase(phase);
    waitMicrosecond(12000);
}

void stepCw(){
    position++;
    phase = ((phase + 1) % 4);
    applyPhase(phase);
    waitMicrosecond(12000);
}

void setPosition(uint8_t positionFixed){
    RED_LED = 1;
    GREEN_LED = 0;
    if(positionFixed > position){
        while(positionFixed != position){
            stepCw();
        }
    }
    else if(positionFixed < position){
        while(positionFixed != position){
            stepCcw();
        }
    }
    RED_LED = 0;
    GREEN_LED = 1;
}

void home(){
    RED_LED = 1;
    GREEN_LED = 0;
    putsUart0("\nCalibrating Motor...");
    position = 0;
    while(position != 199){
        stepCw();
    }
    waitMicrosecond(500000);
    setPosition(tube0);
    RED_LED = 0;
    GREEN_LED = 1;
    putsUart0("Complete\n");
}

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//Command String Stuff
char* getFieldString(USER_DATA* data, uint8_t fieldNumber){
    return &(data->buffer[data->fieldPosition[fieldNumber]]);
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber){
    return atoi(&(data->buffer[data->fieldPosition[fieldNumber]]));
}

bool isCommand(USER_DATA* data, const char strCommand[],uint8_t minArguments){
    if(strcmp(data->buffer,strCommand) == 0)
    {
        if(data->fieldCount-1 == minArguments){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

void getsUart0(USER_DATA *data){
    uint8_t count= 0;
    while(true)
    {
        char c = getcUart0();               //Gets next char
        if(c == 8 || c == 127){             // If c is backspace
            if(count > 0){
                count--;
            }
        }
        else if(c == 13){                   //if c is enter
            data->buffer[count] = '\0';
            break;
        }
        else if(c >= 32){                   //anything else gets added to the buffer
            data->buffer[count] = c;
            count++;
            if(count == MAX_CHARS){
                data->buffer[count] = '\0';
                break;
            }
        }
    }

    return;
}

void parseFields(USER_DATA *data){
    int8_t count = 0;
    int8_t fieldCount = 0;
    char type;
    bool isCurrentDelimiter = false;
    bool isPastDelimiter = true;
    while(count != MAX_CHARS || fieldCount <= MAX_FIELDS){

        if(data->buffer[count] == '\0')
        {
            break;
        }
        else if((data->buffer[count]) >= 65 && (data->buffer[count])<= 90) //A-Z
        {
            type = 'a';
            isCurrentDelimiter = false;
        }
        else if((data->buffer[count]) >= 97 && (data->buffer[count])<= 122) //a - z
        {
            type = 'a';
            isCurrentDelimiter = false;
        }
        else if((data->buffer[count]) >= 45 && (data->buffer[count]) <=57) //0-9-.
        {
            type = 'n';
            isCurrentDelimiter = false;
        }
        else{
            isCurrentDelimiter = true;
        }

        if(isCurrentDelimiter){
            isPastDelimiter = true;
        }
        else if(isCurrentDelimiter == false && isPastDelimiter == true){ //checks if current is not delimiter
            data->fieldPosition[fieldCount] = count;                     //and if past is, saves
            data->fieldType[fieldCount] = type;
            fieldCount++;
            data->fieldCount = fieldCount;
            isPastDelimiter = false;
            data->buffer[count-1] = NULL;
        }
        count++;
    }
}

//-----------------------------------------------------------------------------
//Analog RBG Sensor Stuff
void calibrate(){

    RED_LED = 1;
    GREEN_LED = 0;
    setPosition(tube0);
    waitMicrosecond(500000);
    uint16_t pwm = 0;
    uint16_t r = 0;
    setRgbColor(0,0,0);
    putsUart0("\nCalibrating RGB\n");
    //-----------------------------
    //Calibrate for Red
    putsUart0("Calibrating Red.....");
    while(pwm < 1023 && r < 3072){
        setRgbColor(pwm, 0 , 0);
        waitMicrosecond(3000);
        r = readAdc0Ss3();
        pwm++;
    }
    pwm_r = pwm;
    raw_r = r;
    putsUart0("Complete");
    pwm = 0;
    r = 0;
    //-----------------------------
    //Calibrate for Green
    putsUart0("\nCalibrating Green...");
    while(pwm < 1023 && r < 3072){
        setRgbColor(0, pwm, 0);
        waitMicrosecond(3000);
        r = readAdc0Ss3();
        pwm++;
    }
    pwm_g = pwm;
    raw_g = r;
    putsUart0("Complete");
    pwm = 0;
    r = 0;
    //-----------------------------
    //Calibrate for Blue
    putsUart0("\nCalibrating Blue....");
    while(pwm < 1023 && r < 3072){
        setRgbColor(0, 0, pwm);
        waitMicrosecond(3000);
        r = readAdc0Ss3();
        pwm++;
    }
    pwm_b = pwm;
    raw_b = r;
    putsUart0("Complete\n");
    pwm = 0;
    r = 0;
    //-----------------------------
    setRgbColor(0, 0, 0);
    char str[100];
    sprintf(str, "(R,G,B): (%4u,%4u,%4u)\n", pwm_r,pwm_g,pwm_b);
    putsUart0(str);
    RED_LED = 0;
    GREEN_LED = 1;
}

void measure(uint8_t tube, uint16_t *r, uint16_t *g, uint16_t *b){
    RED_LED = 1;
    GREEN_LED = 0;
    uint16_t i;

    setPosition(tube);
    waitMicrosecond(500000);

    //Measure Red
    for(i = 0; i <= pwm_r; i++){
        setRgbColor(i, 0, 0);
        waitMicrosecond(3000);
        *r = readAdc0Ss3();
    }
    //Measure Green
    for(i = 0; i <= pwm_g; i++){
        setRgbColor(0, i, 0);
        waitMicrosecond(3000);
        *g = readAdc0Ss3();
    }

    //Measure Blue
    for(i = 0; i <= pwm_b; i++){
        setRgbColor(0, 0, i);
        waitMicrosecond(3000);
        *b = readAdc0Ss3();
    }
    setRgbColor(0, 0, 0);
    RED_LED = 0;
    GREEN_LED = 1;
}
//Measure PH
void measurePh(uint8_t tube, double *ph){
    uint16_t re;
    uint16_t gre;
    uint16_t blu;
    measure(tube, &re, &gre, &blu);
    RED_LED = 1;
    GREEN_LED = 0;
    uint8_t i;
    double disRef1, disRef2, phRef1, phRef2, phNew;
    double red_ref = (double)re / (double)raw_r;
    double green_ref = (double)gre / (double)raw_g;
    double blue_ref = (double)blu / (double)raw_b;
    //Calculate Distance for current
    for(i = 0; i < 5; i++){
        double red   = pow((red_ref - (rawReference[i][0])),2);
        double green = pow((green_ref - (rawReference[i][1])),2);
        double blue  = pow((blue_ref - (rawReference[i][2])),2);
        double d = red + green + blue;

        if( i == 0){
            disRef1 = d;
            phRef1 = rawReference[i][3];
        }
        else if(i == 1){
            if(d < disRef1){
                disRef2 = disRef1;
                phRef2 = phRef1;
                disRef1 = d;
                phRef1 = rawReference[i][3];
            }
            else{
                disRef2 = d;
                phRef2 = rawReference[i][3];
            }
        }
        else{
            if(d < disRef1){
                disRef2 = disRef1;
                phRef2 = phRef1;
                disRef1 = d;
                phRef1 = rawReference[i][3];
            }
            else if(d < disRef2){
                disRef2 = d;
                phRef2 = rawReference[i][3];
            }
        }
    }
    phNew = disRef1 / (disRef1 + disRef2);
    phNew *= fabs(phRef2 - phRef1);
    *ph = phNew + phRef1;
    RED_LED = 0;
    GREEN_LED = 1;
}
// Initialize  Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    //Timer for IR
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2;
    //GPIO Registers
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R3 |SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R4; //PORT B, Port E and Port F
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs

    //GPIO PE configurations for  Motor
    GPIO_PORTE_DIR_R |= BLACK_MOTOR_MASK | WHITE_MOTOR_MASK | YELLOW_MOTOR_MASK | GREEN_MOTOR_MASK; // Set to outputs
    GPIO_PORTE_DR2R_R |= BLACK_MOTOR_MASK | WHITE_MOTOR_MASK | YELLOW_MOTOR_MASK | GREEN_MOTOR_MASK;
    GPIO_PORTE_DEN_R |= BLACK_MOTOR_MASK | WHITE_MOTOR_MASK | YELLOW_MOTOR_MASK | GREEN_MOTOR_MASK;

    //Configure AIN11 as an analog input
    GPIO_PORTB_AFSEL_R |= AIN11_MASK;
    GPIO_PORTB_DEN_R &= ~AIN11_MASK;
    GPIO_PORTB_AMSEL_R |= AIN11_MASK;

    //Configure PD0 to read remote data from sensor
    GPIO_PORTD_AFSEL_R |= FREQ_IN_MASK; //breaks here
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD0_M;
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_WT2CCP0;
    GPIO_PORTD_DEN_R |= FREQ_IN_MASK;
    GPIO_PORTD_IM_R = 1;
}
//-----------------------------------------------------------------------------
int main(void)
{
    USER_DATA data;
    // Initialize hardware
    initHw();
    initUart0();
    initRgb();
    initAdc0Ss3();
    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    //Use Ain11 input with N=4 hardware sampling
    setAdc0Ss3Mux(11);
    setAdc0Ss3Log2AverageCount(2);
    //Begin IR Timer Mode
    enableTimerMode();
    //Begin Rest of code, LED red on startup process, green when ready
    phase = 0;
    position = 0;
    home();
    calibrate();
    //Main Loop with commands
    bool instructions = true;
    while(true){

        if(instructions){
            putsUart0("Type in a Command or Press Button on Remote.\n>");
            instructions = false;
        }
        if(valid){
            //Variables used in some functions
            uint16_t r = 0;
            uint16_t g = 0;
            uint16_t b = 0;
            double ph = 0;
            char str[100];

            //Main Switch code
            switch(code){
            //General Commands: Home, Calibrate
            case 64: home(); break; //Power to go to home()
            case 65: calibrate(); break; //Skip to go calibrate()
            //Go to Tube X commands
            case 12: setPosition(tube1);break; //DIY1 to go to tube 1
            case 13: setPosition(tube2);break; //DIY2 to go to tube 2
            case 14: setPosition(tube3);break; //DIY3 to go to tube3
            case 8:  setPosition(tube4);break; //DIY4 to go to tube4
            case 9:  setPosition(tube5);break; //DIY5 to go to tube5
            case 10: setPosition(tube0);break; //DIY6 to go to tube 0/R
            //Measure TubeX Raw Commands
            case 88://R1 for tube 0/R
                measure(tube0, &r, &g, &b);
                sprintf(str, "Raw (R,G,B): (%4u,%4u,%4u)\n", r,g,b);
                putsUart0(str);
                break;
            case 84://R2 for tube 1
                measure(tube1, &r, &g, &b);
                sprintf(str, "Raw (R,G,B): (%4u,%4u,%4u)\n", r,g,b);
                putsUart0(str);
                break;
            case 80://R3 for tube 2
                measure(tube2, &r, &g, &b);
                sprintf(str, "Raw (R,G,B): (%4u,%4u,%4u)\n", r,g,b);
                putsUart0(str);
                break;
            case 28://R4 for tube 3
                measure(tube3, &r, &g, &b);
                sprintf(str, "Raw (R,G,B): (%4u,%4u,%4u)\n", r,g,b);
                putsUart0(str);
                break;
            case 24://R5 for tube 4
                measure(tube4, &r, &g, &b);
                sprintf(str, "Raw (R,G,B): (%4u,%4u,%4u)\n", r,g,b);
                putsUart0(str);
                break;
            case 20://Rup for tube 5
                measure(tube5, &r, &g, &b);
                sprintf(str, "Raw (R,G,B): (%4u,%4u,%4u)\n", r,g,b);
                putsUart0(str);
                break;
            //Measure TubeX Ph commands
            case 69://B1 for Tube 1
                measurePh(tube1,&ph);
                sprintf(str, "PH: %.2f\n", ph);
                putsUart0(str);
                break;
            case 73://B2 for Tube 2
                measurePh(tube2,&ph);
                sprintf(str, "PH: %.2f\n", ph);
                putsUart0(str);
                break;
            case 77://B3 for Tube 3
                measurePh(tube3, &ph);
                sprintf(str, "PH: %.2f\n", ph);
                putsUart0(str);
                break;
            case 30://B4 for Tube 4
                measurePh(tube4,&ph);
                sprintf(str, "PH: %.2f\n", ph);
                putsUart0(str);
                break;
            case 26://B5 for Tube 5
                measurePh(tube5,&ph);
                sprintf(str, "PH: %.2f\n", ph);
                putsUart0(str);
                break;
            }

            valid = false;
            code = 0;
            instructions = true;
        }
        if(kbhitUart0()){
            getsUart0(&data);
            #ifdef DEBUG
            putsUart0(data.buffer);
            putcUart0('\n');
            #endif
            // parse fields
            parseFields(&data);
            #ifdef DEBUG
            uint8_t i;
            for (i = 0; i< data.fieldCount; i++)
            {
                putcUart0(data.fieldType[i]);
                putcUart0('\t');
                putsUart0(&data.buffer[data.fieldPosition[i]]);
                putcUart0('\n');
            }
            #endif
            if (isCommand(&data, "set", 2)){
                valid = true;
                int32_t add = getFieldInteger(&data, 1);
                int32_t data2 = getFieldInteger(&data, 2);
                putcUart0('\n');
                int32_t sub = add-data2;        //test actions
                if(sub== 0){
                    putsUart0("\nResult = 0\n");
                    RED_LED = 0;
                    GREEN_LED = 1;
                }
                else{
                    putsUart0("\nResult != 0\n");
                    GREEN_LED =0;
                    RED_LED = 1;
                }
            }
            else if(isCommand(&data, "alert",1))
            {
                valid = true;
                char* str = getFieldString(&data, 1); //Test to check if its working,
                if(*str == 49){             //section will be rewritten
                    RED_LED = 1;
                }
                else if(*str == 48){
                    RED_LED = 0;
                }
                else
                {
                    putsUart0("In Arguments\n");
                }
            }
            else if(isCommand(&data,"home",0)){
                home();
            }
            else if(isCommand(&data,"calibrate",0)){
                calibrate();
            }
            else if(isCommand(&data,"tube", 1)){
                uint8_t tube = getFieldInteger(&data, 1);
                switch(tube){
                case 0: setPosition(tube0); break;
                case 1: setPosition(tube1); break;
                case 2: setPosition(tube2); break;
                case 3: setPosition(tube3) ; break;
                case 4: setPosition(tube4) ; break;
                case 5: setPosition(tube5)  ; break;
                default: putsUart0("Invalid Command Parameters\n"); break;
                }

            }
            else if(isCommand(&data,"measure", 2)){
                uint16_t r = 0;
                uint16_t g = 0;
                uint16_t b = 0;
                double ph = 0;
                char str[100];
                char *measureType = getFieldString(&data, 2);

                if(strcmp(measureType, "raw") == 0){
                    uint8_t tubeNum = getFieldInteger(&data, 1);
                    char *isR = getFieldString(&data, 1);
                    if(*isR == 'R'){
                       measure(tube0, &r, &g, &b);
                    }
                    else{
                        switch(tubeNum){
                            case 1: measure(tube1, &r, &g, &b); break;
                            case 2: measure(tube2, &r, &g, &b); break;
                            case 3: measure(tube3, &r, &g, &b); break;
                            case 4: measure(tube4, &r, &g, &b); break;
                            case 5: measure(tube5, &r, &g, &b); break;
                            default: putsUart0("Invalid Command Parameters\n"); break;
                        }
                    }
                    sprintf(str, "Raw\nRed: %4u\nGreen: %4u\nBlue %4u\n", r,g,b);
                    putsUart0(str);
                }
                else if(strcmp(measureType, "ph") == 0){

                    uint8_t tubeNum = getFieldInteger(&data, 1);
                        switch(tubeNum){
                            case 1: measurePh(tube1, &ph); break;
                            case 2: measurePh(tube2, &ph); break;
                            case 3: measurePh(tube3, &ph); break;
                            case 4: measurePh(tube4, &ph); break;
                            case 5: measurePh(tube5, &ph); break;
                            default: putsUart0("Invalid Command Parameters\n"); break;
                            }
                            sprintf(str, "PH: %.2f\n",ph);
                            putsUart0(str);
                }
                else{
                    putsUart0("Invalid measurement type\n");
                }
                instructions = true;
            }
            else{
                putsUart0("Invalid Command\n");
                putcUart0('\n');
            }
        }
    }
}
