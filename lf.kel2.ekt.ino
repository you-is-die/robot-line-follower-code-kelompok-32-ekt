; lf_kel3_ekt.asm
;
; Created: 6/14/2024 2:42:42 AM
; Author : yusdi
;


#include <avr/io.h>
#include <util/delay.h>

// Define sensor pins
#define sensor1 PC5
#define sensor2 PC4
#define sensor3 PC3
#define sensor4 PC2
#define sensor5 PC1

// Define motor control pins
#define motorA1 PD7
#define motorA2 PD6
#define motorB1 PD5
#define motorB2 PD4

// PID constants
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.0;

float previous_error = 0.0;
float integral = 0.0;

void setup() {
    // Initialize sensor pins as input
    DDRC &= ~(1 << sensor1);
    DDRC &= ~(1 << sensor2);
    DDRC &= ~(1 << sensor3);
    DDRC &= ~(1 << sensor4);
    DDRC &= ~(1 << sensor5);

    // Initialize motor control pins as output
    DDRD |= (1 << motorA1) | (1 << motorA2) | (1 << motorB1) | (1 << motorB2);

    // Start serial communication for debugging
    // Set baud rate to 9600
    UBRR0H = 0;
    UBRR0L = 103;
    // Enable transmitter
    UCSR0B = (1 << TXEN0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void sendSerial(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void sendSerialStr(const char* str) {
    while (*str) {
        sendSerial(*str++);
    }
}

void sendSerialInt(int value) {
    char buffer[4];
    itoa(value, buffer, 10);
    sendSerialStr(buffer);
}

void motorControl(float speedA, float speedB) {
    if (speedA > 0) {
        PORTD |= (1 << motorA1);
        PORTD &= ~(1 << motorA2);
    } else {
        PORTD &= ~(1 << motorA1);
        PORTD |= (1 << motorA2);
        speedA = -speedA;
    }

    if (speedB > 0) {
        PORTD |= (1 << motorB1);
        PORTD &= ~(1 << motorB2);
    } else {
        PORTD &= ~(1 << motorB1);
        PORTD |= (1 << motorB2);
        speedB = -speedB;
    }

    // Assuming you have PWM set up on these pins, you would set the duty cycle here.
    // For example:
    // OCR0A = (uint8_t)(speedA * 255.0);
    // OCR0B = (uint8_t)(speedB * 255.0);
}

float calculatePID(float error) {
    integral += error;
    float derivative = error - previous_error;
    previous_error = error;
    return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

int main(void) {
    setup();
    while (1) {
        // Read sensor values
        int s1 = PINC & (1 << sensor1) ? 1 : 0;
        int s2 = PINC & (1 << sensor2) ? 1 : 0;
        int s3 = PINC & (1 << sensor3) ? 1 : 0;
        int s4 = PINC & (1 << sensor4) ? 1 : 0;
        int s5 = PINC & (1 << sensor5) ? 1 : 0;

        // Debugging
        sendSerialInt(s1); sendSerial(' ');
        sendSerialInt(s2); sendSerial(' ');
        sendSerialInt(s3); sendSerial(' ');
        sendSerialInt(s4); sendSerial(' ');
        sendSerialInt(s5); sendSerial('\n');

        // Calculate error
        float error = s1 * -2 + s2 * -1 + s3 * 0 + s4 * 1 + s5 * 2;

        // Calculate PID output
        float pidOutput = calculatePID(error);

        // Adjust motor speeds based on PID output
        float baseSpeed = 0.5; // Base speed for both motors
        float speedA = baseSpeed + pidOutput;
        float speedB = baseSpeed - pidOutput;

        // Ensure motor speeds are within bounds
        if (speedA > 1.0) speedA = 1.0;
        if (speedA < 0.0) speedA = 0.0;
        if (speedB > 1.0) speedB = 1.0;
        if (speedB < 0.0) speedB = 0.0;

        motorControl(speedA, speedB);

        _delay_ms(100); // Add a small delay for stability
    }
}
