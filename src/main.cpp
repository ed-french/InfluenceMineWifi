#include <Arduino.h>


#include <WiFi.h>
#include <HTTPClient.h> // To fetch the time and alarm time as required

#include "credentials.h"

#include <ArduinoJson.h>

#include <UrlEncode.h>


// #include <AsyncTCP.h>

#include "esp_sleep.h"
#include "driver/gpio.h"

#define INTERRUPT_PIN 2

#define PIN_LED 10

#include <LSM303.h>
#include <Wire.h>

LSM303 compass;
char report[2000];

uint32_t next_calibration_due=millis();

#define CALIBRATION_INTERVAL_MS 600000

uint16_t g_threshold=1;



bool led_state=false;


// #define ELEGANTOTA_USE_ASYNC_WEBSERVER 1

#define WIFI_AT_BOOT_TIMEOUT_MS 45000

#define OLD_READING_BUFFER_SIZE 16 

// #define USE_FAKE_SLEEP


const char * hostname="Influence";
const char * influence_mine_server_url="http://192.168.1.125/wifi_influence?message=";



typedef struct Reading
{
  int16_t x;
  int16_t y;
  int16_t z;
} reading_t;


reading_t g_old_readings[OLD_READING_BUFFER_SIZE];
uint8_t g_next_old_reading=0;

reading_t mag_as_reading(LSM303::vector<int16_t> mag)
{
  reading_t res=Reading();
  res.x=mag.x;
  res.y=mag.y;
  res.z=mag.z;

  return res;

}


typedef struct Message
{
    char type[20];// BOOT, CALLIBRATION, TRIGGER
    uint32_t running_ms;
    reading_t calibration;
    reading_t latest;
    uint32_t variance;
} message_t;


StaticJsonDocument<1023> doc;

void get_json_message(char * buff, message_t message)
{
  // Result will be like:
  /*
  {"running_ms":12332,"calibration":[123,12,332],"latest":[122,24,300],"variance":50}
  */
  doc.clear();
  doc["type"]=message.type;
  doc["running_ms"]=message.running_ms;

  JsonArray calibration=doc.createNestedArray("calibration");
  calibration.add(message.calibration.x);
  calibration.add(message.calibration.y);
  calibration.add(message.calibration.z);

  JsonArray latest=doc.createNestedArray("latest");
  latest.add(message.latest.x);
  latest.add(message.latest.y);
  latest.add(message.latest.z);

  doc["variance"]=message.variance;

  serializeJson(doc,buff,1023);


}


void get_sendable_message(char * buff,message_t)
{
  //todo
}


reading_t g_mean_reading;

Reading get_mean_reading(uint16_t samples)
{
  // does calibration...
  Serial.println("recalibrating.....");
  int32_t xacc=0;
  int32_t yacc=0;
  int32_t zacc=0;

  for (uint16_t i=0;i<samples;i++)
  {
    compass.read();
    xacc+=compass.m.x;
    yacc+=compass.m.y;
    zacc+=compass.m.z;


    delay(140); // Wait for new data
  }
  Reading res;
  res.x=int(xacc/samples);
  res.y=int(yacc/samples);
  res.z=int(zacc/samples);

  // Reset the global old reading buffer
  g_next_old_reading=0;
  for (uint8_t i=0;i<OLD_READING_BUFFER_SIZE;i++)
  {
    g_old_readings[i]=res;
  }

  return res;


}


reading_t get_old_reading_average()
{
  int32_t x=0;
  int32_t y=0;
  int32_t z=0;
  for (uint8_t i=0;i<OLD_READING_BUFFER_SIZE;i++)
  {
    x+=g_old_readings[i].x;
    y+=g_old_readings[i].y;
    z+=g_old_readings[i].z;
  }
  reading_t res=Reading();
  res.x=x/OLD_READING_BUFFER_SIZE;
  res.y=y/OLD_READING_BUFFER_SIZE;
  res.z=z/OLD_READING_BUFFER_SIZE;
  return res;
}

uint32_t calc_variance(LSM303::vector<int16_t> measured,reading_t baseline)
{
  uint32_t vx=abs(measured.x-baseline.x);
  uint32_t vy=abs(measured.y-baseline.y);
  uint32_t vz=abs(measured.z-baseline.z);
  return vx+vy+vz;
}


uint32_t calc_variance(reading_t measured, reading_t baseline)
{
  uint32_t vx=abs(measured.x-baseline.x);
  uint32_t vy=abs(measured.y-baseline.y);
  uint32_t vz=abs(measured.z-baseline.z);
  return vx+vy+vz;
}




void fake_sleep()
{
    // Just waits for 4 seconds or until pin changes
    pinMode(INTERRUPT_PIN,INPUT_PULLUP);
    digitalWrite(PIN_LED,false);// LED OFF FOR DURATION
    uint16_t tries_left=2000;
    while (true)
    {
      if (tries_left==0)
      {
        Serial.println("Timed out waiting for 'interrupt'");
        return;
      }
      if (digitalRead(INTERRUPT_PIN)==0)
      {
        Serial.println("interrupt rx'd");
        return;
      }

    }
}

void sleep_now()
{

  Serial.println("Going to sleep now");

  // pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
  // esp_deep_sleep_enable_gpio_wakeup(1 << INTERRUPT_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  esp_deep_sleep_enable_gpio_wakeup(1 << INTERRUPT_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);

  gpio_set_direction((gpio_num_t)INTERRUPT_PIN, GPIO_MODE_INPUT); 
  esp_sleep_enable_timer_wakeup(10*1000000);
  esp_light_sleep_start();
  
  //esp_deep_sleep_start();

  Serial.println("This will never be printed");
}


uint16_t signal_to_server(const char * url)
{

  // Check the wifi is still up
  if (!WiFi.status()== WL_CONNECTED)
  {
    esp_restart(); // Actually maybe just sleep????
    return 0; // never happens but gotta keep the compiler happy
  }

  HTTPClient http;


  // char url[400];
  // strcpy(url,influence_mine_server_url);
  // strcat(url,wakeup_reason);
  Serial.println(url);
      
  // Your Domain name with URL path or IP address with path
  http.begin(url);
    
  int httpResponseCode = http.GET();

  String payload=String("0");
      
  if (httpResponseCode>0)
  {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        payload = http.getString();
        Serial.println(payload);
  } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return atoi(payload.c_str());


}



void connect_wifi()
{


  
  WiFi.setHostname(hostname);

  /*                    

  ################################################################                                      
  
      JUST FOR THE 1.0.0 ESP32C3 Mini Lolin boards due to RF error
  
  
  */
  //WiFi.setTxPower(WIFI_POWER_8_5dBm); // May be required to connect

   /*                    

  ################################################################                                      
 
  
  */



  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  delay(1000);
   /*                    

  ################################################################                                      
  
      JUST FOR THE 1.0.0 ESP32C3 Mini Lolin boards due to RF error
  
  
  */
  // WiFi.setTxPower(WIFI_POWER_8_5dBm); // May be required to connect

   /*                    

  ################################################################                                      
 
  
  */
  WiFi.begin(ssid, password);
  delay(1000);
  Serial.printf("Connecting to: %s\n",ssid);
  Serial.println("");

  uint32_t wifi_timeout=millis()+WIFI_AT_BOOT_TIMEOUT_MS;

  // Wait for connection
  while (true)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nWifi now connected");
      break;
    }
    if (millis()>wifi_timeout)
    {
      Serial.println("Pointless message saying we are restarting to have another go at connecting");
      esp_restart();
    }
    
    delay(200);
    Serial.print(".");
  }

  
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}

uint16_t send_message_get_threshold(const char * type,reading_t calib, reading_t latest, uint32_t variance)
{
      message_t mess=Message();
      mess.running_ms=millis();
      mess.variance=variance;
      mess.latest=latest;
      mess.calibration=calib;
      strcpy(mess.type,type);
   
      get_json_message(report,mess);
      Serial.println(report);

      String message_url=String(influence_mine_server_url)+urlEncode(report);
      uint16_t res=signal_to_server(message_url.c_str());
      Serial.printf("Fetched new threshold: %d\n",res);
      return res;
}


void get_wakeup_reason(char * reason)
{
   esp_sleep_wakeup_cause_t wakeup_reason;

   wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
      case ESP_SLEEP_WAKEUP_GPIO:    // 05 - this is used by ESP32-C3
        strcpy(reason,"Wakeup%20by%20GPIO");
        break;
      case ESP_SLEEP_WAKEUP_EXT0:
        strcpy(reason,"Wakeup%20caused%20by%20external%20signal%20using%20RTC_IO");
        break;
      case ESP_SLEEP_WAKEUP_EXT1:
        strcpy(reason,"Wakeup%20caused%20by%20external%20signal%20using%20RTC_CNTL");
        break;
      case ESP_SLEEP_WAKEUP_TIMER:
        strcpy(reason,"Wakeup%20caused%20by%20timer");
        break;
      case ESP_SLEEP_WAKEUP_TOUCHPAD:
        strcpy(reason,"Wakeup%20caused%20by%20touchpad");
        break;
      case ESP_SLEEP_WAKEUP_ULP:
        strcpy(reason,"Wakeup%20caused%20by%20ULP%20program");
        break;
      default:
        strcpy(reason,"Unknown%20reason");
        
        break;
     }
}

void startup_countdown(uint8_t seconds)
{
  
  for (uint8_t i=seconds;i>0;i--)
  {
    digitalWrite(PIN_LED,led_state);
    led_state=!led_state;
    Serial.println(i);
    delay(1000);
  }
}

void wifi_off()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}


void setup()
{
  Serial.begin(115200);
  
  pinMode(PIN_LED,OUTPUT);
  pinMode(INTERRUPT_PIN,INPUT);
  char reason[200];
  startup_countdown(10);
  // get_wakeup_reason(reason);
  // connect_wifi();
  // startup_countdown(2);
  // 

  //signal_to_server(reason);
  // startup_countdown(10);

  //wifi_off();
  
  // sleep_now();
  Wire.begin();
  compass.init();
  compass.enableDefault();

  //mean_reading=get_mean_reading(100);

  //printf("mean reading: %d,%d,%d\n",mean_reading.x,mean_reading.y,mean_reading.z);


  
  
 



}







void loop()
{
  uint16_t cycle_count=0;
  while (true)
  {
    cycle_count++;
    // Consider recalibration (also repeated after each trigger)
    if (random(0,7*60*60)<1 || millis()>next_calibration_due)
    {
      g_mean_reading=get_mean_reading(100);
      next_calibration_due=millis()+CALIBRATION_INTERVAL_MS;
      connect_wifi();
      g_threshold=send_message_get_threshold("CALIBRATION_TIMED",g_mean_reading,g_mean_reading,0);
      wifi_off();

    }
    
    compass.readMag();

    reading_t latest=mag_as_reading(compass.m);

    if ((cycle_count & 0x0F)==0)
    {
      // Compare the old reading buffer with the last calibration, do we need to recalibrate?
      reading_t old_reading_ave=get_old_reading_average();
      uint32_t variance=calc_variance(old_reading_ave,g_mean_reading);
      Serial.printf("Recent readings c.f. to calibration variance: %d\n",variance);

      // put this reading in the buffer
      g_old_readings[g_next_old_reading]=latest;
      g_next_old_reading++;
      if (g_next_old_reading==OLD_READING_BUFFER_SIZE)
      {
        g_next_old_reading=0;
      }

      if (variance>10)
      {
        g_mean_reading=get_mean_reading(100);//Recalibrate
        connect_wifi();
        g_threshold=send_message_get_threshold("CALIBRATION_DRIFTED",g_mean_reading,g_mean_reading,0);
        
        wifi_off();
      }


    }


      

      uint32_t var=calc_variance(compass.m,g_mean_reading);

      if (var>g_threshold)
      {

        connect_wifi();
        g_threshold=send_message_get_threshold("TRIGGER", \
                                          g_mean_reading,\
                                          mag_as_reading(compass.m), \
                                          var);
        wifi_off();
        // Recalibrate after each send
        g_mean_reading=get_mean_reading(100);

      }




      //snprintf(report, sizeof(report), "%d,%d,%d,,%d",
      //compass.m.x, compass.m.y, compass.m.z,var);
      //Serial.println(report);



      // Serial.println(report);
      // 
      // signal_to_server(report);
      // 

      

      #ifdef USE_FAKE_SLEEP
        fake_sleep();
      #else
        sleep_now();
      #endif
      //

      
  }

  
}



  //signal_to_server();

