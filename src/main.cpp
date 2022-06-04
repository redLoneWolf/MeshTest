#include <Arduino.h>
#include <painlessMesh.h>
#include <Preferences.h>

#include "sensors.h"

#include "BluetoothSerial.h"
#include <Parser.h>

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

#define BAUDRATE 115200

// #include <parameters.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

Preferences preferences;
void led();

void callBack(Command command, uint8_t data[64])
{
  Serial.printf("Command : %d Data : %d \n", command, data[0]);

  switch (command)
  {
  case LED:
    led();
    break;
  case GPS:
    preferences.putBool("gps", true);
    initGPS(data[0]);
    break;

  case BMP:
    preferences.putBool("bmp", true);
    initBMP();
    break;

  case MAG:
    preferences.putBool("mag", true);
    initCompass();
    break;

  case MESSAGE:
  {
    int sender = 1123;
    String txt = "Default";
    Serial.printf("Sending %s to  : %d ", txt.c_str(), sender);
    mesh.sendSingle(sender,txt);
    mesh.getNodeList
  }
  break;

  default:
    Serial.printf("Command %d to  data[0] :  %d ", command, data[0]);
    break;
  }
}

Parser<BluetoothSerial> parser(&callBack);

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;

// User stub
void sendMessage(); // Prototype so PlatformIO doesn't complain

Task taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, &sendMessage);

TaskHandle_t Task1;
TaskHandle_t Task2;

// Needed for painless library
void receivedCallback(uint32_t from, String &msg)
{

  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  //  updateAll();
}

void newConnectionCallback(uint32_t nodeId)
{
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback()
{
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset)
{
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void Task1code(void *pvParameters)
{

  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  
  for (;;)
  {
    updateAll();
    parser.update();
  }
}

// Task2code: blinks an LED every 700 ms
void Task2code(void *pvParameters)
{
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  Serial.printf(" Hai");
  // mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes(ERROR | STARTUP); // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

  for (;;)
  {

    mesh.update();
  }
}



void sendMessage()
{
  // updateAll();
  String msg = getJsonString();
  // Serial.printf("Message Sent %s",msg.c_str());
  // msg += mesh.getNodeId();
  mesh.sendBroadcast(msg);

 

  // taskSendMessage.setInterval(random(TASK_SECOND * 1, TASK_SECOND * 5));
  Serial.println("Sent  : ");
  Serial.print(msg);
  Serial.println();
}

void setup()
{
  Serial.begin(BAUDRATE);
  pinMode(LED_BUILTIN, OUTPUT);
  parser.begin("ESP BLE");
  preferences.begin("my-app", false);

  Serial.printf(" Hi");
  // initAll();
  initSensors(preferences);

  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1",   /* Name of the task */
      10000,     /* Stack size in words */
      NULL,      /* Task input parameter */
      0,         /* Priority of the task */
      &Task1,    /* Task handle. */
      0);        /* Core where the task should run */

  xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      1);
}
bool connected = false;
void led()
{
  // int8_t state = parser.read_i8();

  //  write_i8(state);
  if (connected == true)
  {
    digitalWrite(LED_BUILTIN, LOW);
    connected = false;
  }
  else
  {
    connected = true;
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // parser.sendPacket(4, LED, 1);
  // write_i8(102);
  // write_i8(103);
  // write_i8(104);
}

void checkConnected()
{
  if (connected)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void loop()
{
  // it will run the user scheduler as well
  checkConnected();
}