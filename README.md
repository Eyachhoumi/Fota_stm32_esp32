      FOTA (Firmware Over-The-Air) for STM32 and ESP32

      
         📝 Project Overview
      This project demonstrates a Firmware Over-The-Air (FOTA) update system for STM32 microcontrollers using an ESP32 module for WiFi connectivity and MQTT for communication. The main objective is to enable seamless, wireless firmware updates for the STM32F407 microcontroller, with the ESP32 module handling the firmware download process.

         🛠️ Features
      Wireless Firmware Updates: Automatically update the firmware on an STM32F407 microcontroller through an MQTT message received by the ESP32 module.
      WiFi Connectivity with ESP32: The ESP32 module connects to a WiFi network and subscribes to MQTT topics for receiving update notifications.
      CRC Validation: Ensures firmware integrity by calculating and verifying CRC values during the update process.
      HTTP Download: Downloads the firmware binary using HTTP links received via MQTT, with the ESP32 managing the file download.
      
         🔄 Project Workflow
      Firmware Upload: The new firmware binary is uploaded to a MinIO server.
      MQTT Notification: Jenkins publishes an MQTT message containing the HTTP download link and the CRC value of the firmware.
      ESP32 Subscription: The ESP32 module subscribes to the MQTT topic and receives the message with the HTTP link and CRC value.
      ESP32 Download: The ESP32 downloads the firmware binary using the received HTTP link.
      Transfer to STM32: Once the download is complete, the ESP32 transfers the binary file to the STM32 via a communication interface (UART/I2C).
      Verification and Execution: The STM32 verifies the firmware integrity using the received CRC value and executes the new firmware if the verification is successful.
      
         📋 Requirements
      STM32F407 Discovery Board
      ESP32 Module
      MinIO Server
      MQTT Broker (Mosquitto)
      CI Server (Jenkins)
      STM32CubeIDE and Arduino IDE
