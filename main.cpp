#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

extern "C" {
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
}

#define NFFT 128            
#define SAMPLING_FREQ 8000  
#define TARGET_BIN 19        
#define MAG_THRESHOLD 150.0f 

#define MOTOR_PORT GPIO_PORTD_BASE
#define IN1_PIN GPIO_PIN_0
#define IN2_PIN GPIO_PIN_1

#define DHT_PORT GPIO_PORTD_BASE
#define DHT_PIN GPIO_PIN_2

struct cmpx { float real; float imag; };
typedef struct cmpx COMPLEX;
COMPLEX in_data[NFFT];
COMPLEX Twiddle[NFFT];

#define LCD_SLAVE_ADDR 0x3C
const int fontData[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 
    0x3E, 0x51, 0x49, 0x45, 0x3E, 
    0x00, 0x42, 0x7F, 0x40, 0x00, 
    0x42, 0x61, 0x51, 0x49, 0x46, 
    0x21, 0x41, 0x45, 0x4B, 0x31, 
    0x18, 0x14, 0x12, 0x7F, 0x10, 
    0x27, 0x45, 0x45, 0x45, 0x39, 
    0x3C, 0x4A, 0x49, 0x49, 0x30, 
    0x01, 0x71, 0x09, 0x05, 0x03, 
    0x36, 0x49, 0x49, 0x49, 0x36, 
    0x06, 0x49, 0x49, 0x29, 0x1E, 
    0x00, 0x36, 0x36, 0x00, 0x00, 
    0x3E, 0x41, 0x41, 0x41, 0x22, 
    0x7F, 0x08, 0x08, 0x08, 0x7F, 
    0x3F, 0x40, 0x40, 0x40, 0x3F  
};

void WriteCmd(int cmd) {
    I2CMasterSlaveAddrSet(I2C2_BASE, LCD_SLAVE_ADDR, false);
    I2CMasterDataPut(I2C2_BASE, 0x00);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_BASE));
    I2CMasterDataPut(I2C2_BASE, (uint8_t)cmd);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C2_BASE));
}

void WriteData(int data) {
    I2CMasterSlaveAddrSet(I2C2_BASE, LCD_SLAVE_ADDR, false);
    I2CMasterDataPut(I2C2_BASE, 0x40);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_BASE));
    I2CMasterDataPut(I2C2_BASE, (uint8_t)data);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C2_BASE));
}

void DisplayString(int x, int y, const char *s) {
    for(; *s; s++) {
        WriteCmd(0xb0 | (y & 0x07));
        int col = 128 - x;
        WriteCmd(0x10 | ((col+2) >> 4));
        WriteCmd(0x0f & (col+2));
        for(int i=4; i>=0; i--) {
            int idx = 0;
            if (*s == ':') idx = 10;
            else if (*s == 'C') idx = 11;
            else if (*s == 'H') idx = 12;
            else if (*s == 'U') idx = 13;
            else if (*s >= '0' && *s <= '9') idx = (*s - '0' + 1);
            WriteData(fontData[idx*5+i]);
        }
        x += 6;
    }
}

bool Read_DHT11(uint8_t *temp, uint8_t *hum) {
    uint8_t data[5] = {0,0,0,0,0};
    
    GPIOPinTypeGPIOOutput(DHT_PORT, DHT_PIN);
    GPIOPinWrite(DHT_PORT, DHT_PIN, 0);
    delay(18); 
    GPIOPinWrite(DHT_PORT, DHT_PIN, DHT_PIN);
    delayMicroseconds(40);
    
    GPIOPinTypeGPIOInput(DHT_PORT, DHT_PIN);
    
    uint32_t timeout = 10000;
    while(!GPIOPinRead(DHT_PORT, DHT_PIN)) { if(--timeout == 0) return false; }
    timeout = 10000;
    while(GPIOPinRead(DHT_PORT, DHT_PIN)) { if(--timeout == 0) return false; }
    
    for (int i = 0; i < 40; i++) {
        while(!GPIOPinRead(DHT_PORT, DHT_PIN)); 
        delayMicroseconds(40); 
        if(GPIOPinRead(DHT_PORT, DHT_PIN)) {
            data[i/8] |= (1 << (7 - (i%8)));
            while(GPIOPinRead(DHT_PORT, DHT_PIN)); 
        }
    }
    
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        *hum = data[0];
        *temp = data[2];
        return true;
    }
    return false;
}

void make_twiddle_array(int fftlen, struct cmpx *Twiddle) {
    for (int n=0 ; n<fftlen; n++) {
        float vn = (float) (3.14159265f * n / fftlen);
        Twiddle[n].real = cos(vn);
        Twiddle[n].imag = -sin(vn);
    }
}

void fft(struct cmpx *Y, int fftlen) {
    COMPLEX temp1, temp2;
    int i, j, k, upper_leg, lower_leg, leg_diff, num_stages=0, index, step;
    make_twiddle_array(fftlen, Twiddle);
    
    i=1;
    do { num_stages += 1; i = i * 2; } while (i != fftlen);
    
    leg_diff = fftlen/2; step = 2;
    for (i=0; i < num_stages; i++) {
        index = 0;
        for (j=0; j < leg_diff; j++) {
            for (upper_leg = j; upper_leg < fftlen; upper_leg += (2*leg_diff)) {
                lower_leg = upper_leg + leg_diff;
                temp1.real = Y[upper_leg].real + Y[lower_leg].real;
                temp1.imag = Y[upper_leg].imag + Y[lower_leg].imag;
                temp2.real = Y[upper_leg].real - Y[lower_leg].real;
                temp2.imag = Y[upper_leg].imag - Y[lower_leg].imag;
                Y[lower_leg].real = temp2.real*Twiddle[index].real - temp2.imag*Twiddle[index].imag;
                Y[lower_leg].imag = temp2.real*Twiddle[index].imag + temp2.imag*Twiddle[index].real;
                Y[upper_leg].real = temp1.real; Y[upper_leg].imag = temp1.imag;
            }
            index += step;
        }
        leg_diff /= 2; step *= 2;
    }
}

void run_motor_sequence() {
    Serial.println("--- 1200 Hz Sound Freq Match Found! Triggering Motor ---");
    DisplayString(10, 6, "RUNNING ");

    GPIOPinWrite(MOTOR_PORT, IN1_PIN | IN2_PIN, IN1_PIN);
    delay(3000); 

    GPIOPinWrite(MOTOR_PORT, IN1_PIN | IN2_PIN, 0);
    delay(1000); 

    GPIOPinWrite(MOTOR_PORT, IN1_PIN | IN2_PIN, IN2_PIN);
    delay(3000); 

    GPIOPinWrite(MOTOR_PORT, IN1_PIN | IN2_PIN, 0);
    DisplayString(10, 6, "READY   ");
}

void setup() {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(MOTOR_PORT, IN1_PIN | IN2_PIN);
    GPIOPinWrite(MOTOR_PORT, IN1_PIN | IN2_PIN, 0); 

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE4_I2C2SCL); GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4); GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceDisable(ADC0_BASE, 3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);

    WriteCmd(0xAE); WriteCmd(0x8D); WriteCmd(0x14); WriteCmd(0xAF);
    DisplayString(10, 6, "READY   ");

    Serial.begin(115200);
}

void loop() {
    uint32_t micSample;
    uint8_t currentTemp = 0, currentHum = 0;
    char outBuf[32];
    
    for (int i = 0; i < NFFT; i++) {
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, &micSample);
        
        in_data[i].real = (float)micSample;
        in_data[i].imag = 0.0f;
        delayMicroseconds(125); 
    }

    fft(in_data, NFFT);

    float mag1200 = sqrt(in_data[TARGET_BIN].real * in_data[TARGET_BIN].real + 
                         in_data[TARGET_BIN].imag * in_data[TARGET_BIN].imag);

    if(Read_DHT11(&currentTemp, &currentHum)) {
        sprintf(outBuf, "C  : %d", currentTemp); 
        DisplayString(10, 2, outBuf);
        sprintf(outBuf, "HU : %d", currentHum);  
        DisplayString(10, 4, outBuf);
        
        Serial.print("Temp: "); Serial.print(currentTemp);
        Serial.print("C | Humidity: "); Serial.print(currentHum);
        Serial.println("%");
    }

    if (mag1200 > MAG_THRESHOLD) {
        run_motor_sequence();
    }

    delay(100); 
}
