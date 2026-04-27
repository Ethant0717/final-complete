#include <XboxSeriesXControllerESP32_asukiaaa.hpp> 

 

XboxSeriesXControllerESP32_asukiaaa::Core xboxController; 

 #define RX_PIN 16
#define TX_PIN 17

#define CUSTOM_RX_PIN 16

#define CUSTOM_TX_PIN 17 // esp -> stm 

 const float PITCH_LIMIT = 20.0f;
const float ROLL_LIMIT  = 20.0f;
const float YAW_LIMIT   = 50.0f;

void setup() { 

  Serial.begin(115200); // Debug to PC 

  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); 

 

  Serial.println("Starting Xbox Controller..."); 

  xboxController.begin(); 

} 

 

void loop() { 

  xboxController.onLoop(); 

 

  if (xboxController.isConnected()) { 

    auto &e = xboxController.xboxNotif; 

 

    float throttle_up = map(e.trigLT, 0, 1023, 1100, 1700);  

 

    float throttle_down = map(e.trigRT, 0, 1023, 1100, 1700);  

 

    float pitch    = map(e.joyLVert, -32768, 32767, -PITCH_LIMIT, PITCH_LIMIT);  

 

    float roll     = map(e.joyLHori, -32768, 32767, -ROLL_LIMIT,  ROLL_LIMIT);  

 

    float yaw      = map(e.joyRHori, -32768, 32767, -YAW_LIMIT,   YAW_LIMIT);  

    float throttle = throttle_up - throttle_down;

         if (throttle < 1000) throttle = 1000;
        if (throttle > 1700) throttle = 1700;

    char dataPacket[80];  

     

    sprintf(dataPacket,
        "U%.2f, D%.2f, P%.2f, R%.2f, Y%.2f\n",
        throttle_up,
        throttle_down,
        pitch,
        roll,
        yaw);


    Serial2.print(dataPacket); 

 

    Serial.print("Sent to STM32: "); 

    Serial.print(dataPacket); 

 

    delay(100); 

     

  } else { 

    if (xboxController.getCountFailedConnection() > 10) { 

       ESP.restart(); 

    } 

    delay(1000); 

  } 

} 

 