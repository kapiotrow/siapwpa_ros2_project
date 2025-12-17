#include <EEPROM.h>
//#define _NAMIKI_MOTOR
#include <MotorWheel.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <SONAR.h>

#include "pb_encode.h"
#include "pb_decode.h"
#include "mecanum.pb.h"
#include <stdio.h>

/* 
 * Timeout for waiting for a message from the controller in milliseconds,
 * after which the car will stop. 
 */
#define MESSAGE_TIMEOUT_MS 1000

/*
************************************************************************************
                                     Sonar:0x12
                           ------------------------------
                          |                             |
                        M3|                             |M2
                          |                             |
                          |                             |
                          |                             |
                Sonar:0x13|                             |Sonar:0x11
                          |                             |
                          |                             |Power Switch
                          |                             |
                          |                             |
                          -------------------------------
                          |                             |
                        M4|                             |M1
                          |                             |
                          -------------------------------
                                    Sonar:0x14
************************************************************************************
*/

irqISR(irq1, isr1);
MotorWheel wheel_UL(3, 2, 4, 5, &irq1);

irqISR(irq2, isr2);
MotorWheel wheel_LL(9, 8, 14, 15, &irq2);

irqISR(irq3, isr3);
MotorWheel wheel_LR(10, 12, 16, 17, &irq3);

irqISR(irq4, isr4);
MotorWheel wheel_UR(11, 13, 18, 19, &irq4);

Omni4WD Omni(&wheel_UL, &wheel_LL, &wheel_LR, &wheel_UR);

SONAR sonar11(0x11), sonar12(0x12), sonar13(0x13), sonar14(0x14);
unsigned short distBuf[4];

unsigned char sonarsUpdate()
{
    static unsigned char sonarCurr = 1;
    if (sonarCurr == 4)
    {
        sonarCurr = 1;
    }
    else
    {
        ++sonarCurr;
    }

    if (sonarCurr == 1)
    {
        distBuf[1] = sonar12.getDist();
        sonar12.showDat();
        sonar12.trigger();
    }
    else if (sonarCurr == 2)
    {
        distBuf[2] = sonar13.getDist();
        sonar13.showDat();
        sonar13.trigger();
    }
    else if (sonarCurr == 3)
    {
        distBuf[3] = sonar14.getDist();
        sonar14.showDat();
        sonar14.trigger();
    }
    else
    {
        distBuf[0] = sonar11.getDist();
        sonar11.showDat();
        sonar11.trigger();
    }

    return sonarCurr;
}

#ifdef PRINT_DEBUG_MESSAGES
void print_wheel_speeds(void)
{
    Serial.println("Wheel speeds:");
    Serial.print("UL: ");
    Serial.println(Omni.wheelULGetSpeedMMPS());
    Serial.print("UR: ");
    Serial.println(Omni.wheelURGetSpeedMMPS());
    Serial.print("LL: ");
    Serial.println(Omni.wheelLLGetSpeedMMPS());
    Serial.print("LR: ");
    Serial.println(Omni.wheelLRGetSpeedMMPS());
}
#endif

void setup()
{
    TCCR1B = TCCR1B & 0xf8 | 0x01; // Pin9,Pin10 PWM 31250Hz /Modified Pin
    TCCR2B = TCCR2B & 0xf8 | 0x01; // Pin3,Pin11 PWM 31250Hz

    // Initialize Serial communication
    Serial.begin(19200);
    delay(100);

    SONAR::init(13);

    /* Disabling PID couses all the wheels to stop imidiately */
    Omni.PIDDisable();
    delay(100);

    Omni.PIDEnable(0.35, 0.02, 0, 10);
}

// Message received flag
bool messageReceived = false;
static unsigned long lastMessageTime = 0;

void loop()
{
    // Buffer to store incoming data with default values
    uint8_t receivedBuffer[256] = {0};
    size_t bytesRead = 0;

    Serial.print("Available bytes: ");
    Serial.println(Serial.available());

    // Read data from the serial port into the buffer
    while (Serial.available() > 0)
    {
        // Read one byte at a time
        receivedBuffer[bytesRead] = Serial.read();

#ifdef PRINT_DEBUG_MESSAGES
        Serial.print((char)receivedBuffer[bytesRead]);
#endif

        // Increment the bytes read index
        bytesRead++;

        // Check if the buffer is full, you may adjust the buffer size as needed
        if (bytesRead >= sizeof(receivedBuffer))
        {
#ifdef PRINT_DEBUG_MESSAGES
            Serial.println("Error: Received data exceeds buffer size");
#endif
            return;
        }
        
        // Small delay to allow next byte to arrive
        if (Serial.available() > 0) {
            delay(1);
        }
    }

    ControlRequest receivedControlRequest;
    // Check if a complete ControlRequest message is received
    if (bytesRead > 0)
    {
        // Create a stream that reads from the buffer
        pb_istream_t istream = pb_istream_from_buffer(receivedBuffer, bytesRead);
        receivedControlRequest = ControlRequest_init_zero;

        if (pb_decode(&istream, &ControlRequest_msg, &receivedControlRequest))
        {
            // Successfully decoded a ControlRequest message
#ifdef PRINT_DEBUG_MESSAGES
            Serial.println("Received ControlRequest message:");
            Serial.print("Speed: ");
            Serial.println(receivedControlRequest.speed_mmps);
            Serial.print("Radius: ");
            Serial.println(receivedControlRequest.rad);
            Serial.print("Angular Velocity: ");
            Serial.println(receivedControlRequest.omega);
#endif

            // Set the message received flag
            messageReceived = true;

            // Reset the lastMessageTime
            lastMessageTime = millis();
        }
#ifdef PRINT_DEBUG_MESSAGES
        else
        {
            Serial.println("Error decoding ControlRequest message");
        }
#endif

        // Reset bytesRead after processing the message
        bytesRead = 0;
    }

    if (messageReceived)
    {
        // Set the car
#ifdef PRINT_DEBUG_MESSAGES
        Serial.println("Set the car");
#endif
        Omni.setCarMove(receivedControlRequest.speed_mmps,
                        receivedControlRequest.rad,
                        receivedControlRequest.omega);
        
        // Calculate PID and run PWM
        Omni.delayMS(200);

        // Set the LED on
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else if (millis() - lastMessageTime > MESSAGE_TIMEOUT_MS)
    {
        // Stop the car
#ifdef PRINT_DEBUG_MESSAGES
        Serial.println("Stop the car");
#endif
        Omni.setCarMove(0, 0, 0);

        // Calculate PID and run PWM
        Omni.delayMS(200);

        // Set the LED off
        digitalWrite(LED_BUILTIN, LOW);
    }
    messageReceived = false;
}
