#include "DEV_Config.h"
#include "LCD_Driver.h"
#include "GUI_Paint.h"
#include <SPI.h> // Include SPI for communication

// Global variables
unsigned long lastCommandTime = 0; // Store the time of the last command
bool isWaitingToReset = false;    // Flag to check if a reset to white is pending

void setup() {
    // Initialize the LCD
    Config_Init();
    LCD_Init();          // Initialize the display
    LCD_Clear(WHITE);    // Start with a white screen
    delay(100);          // Give some time for initialization
    LCD_Clear(BLACK);    // Start with a white screen
  
    // Initialize Serial Communication
    Serial.begin(9600);
    Serial.println("Waiting for 'input' to change screen...");
}
  
void loop() {
    // Check if a command was received
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n'); // Read command from Serial Monitor
        command.trim(); // Remove extra spaces or newlines

        if (command == "yes") {
            Serial.println("Turning screen green!");
            LCD_Clear(GREEN); // Turn the screen green
            isWaitingToReset = true;
            lastCommandTime = millis(); // Record the time of the command
        } else if (command == "no") {
            Serial.println("Turning screen red!");
            LCD_Clear(RED); // Turn the screen red
            isWaitingToReset = true;
            lastCommandTime = millis(); // Record the time of the command
        } else if (command == "waiting") {
            Serial.println("Turning screen blue!");
            LCD_Clear(BLUE); // Turn the screen blue
            isWaitingToReset = true;
            lastCommandTime = millis(); // Record the time of the command
        } else {
            Serial.println("Invalid command. Send 'yes', 'no', or 'error' to change the screen color.");
        }
    }

    // Check if it's time to reset the screen to white
    if (isWaitingToReset && (millis() - lastCommandTime >= 2500)) { // 10 seconds = 10000 milliseconds
        Serial.println("Resetting screen to white...");
        LCD_Clear(BLACK); // Reset the screen to white
        isWaitingToReset = false; // Reset the flag
    }
}
