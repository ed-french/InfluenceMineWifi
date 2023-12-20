/*


      ESP32C3 Wifi Influence Mine
      ===========================

      A SeedStudio Xiao ESP32C3 plus an LM303 magnetometer provide a
      battery powered wifi vehicle sensor with average current draw of around 1.5mA
      Hence a 32700 6000mA hour battery should last around 4000 hours, or c. 150 days

      Periodically recalibrates itself

      Requires a server endpoint that accepts its json message, and responds with updated parameters for its operation


      Typical server response:

      {
        "trigger_threshold":30,
        "recalibration_interval_ms":600000,
        "http_retries":3,
        "drift_recalibration_threshold":10,
        "wifi_connect_timeout_ms":45000,
        "max_sleep_interval_ms":10000,
        "reboot_now_flag":0,
        "use_fake_sleep":1,
        "stay_connected":0
      }

      To do:

          


        




        Wiring
        ======

                         =====================
                        =| G20           G21 |=
                        =| G8             G7 |= LM303 SCL
                        =| G9             G6 |= LM303 SDA
                        =| G10            G5 |=
        LiFePo4 + LM303 =| 3v3            G4 |=
        LiFePo4 + LM303 =| Gnd  |=====|   G3 |=
                        =| 5V   | USB |   G2 |= LM303 DataReady
                         =====================



*/






#include <Arduino.h>


#include <WiFi.h>
#include <HTTPClient.h> // To fetch the time and alarm time as required


#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>


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




bool led_state=false;


// #define ELEGANTOTA_USE_ASYNC_WEBSERVER 1

#define WIFI_AT_BOOT_TIMEOUT_MS 45000

#define OLD_READING_BUFFER_SIZE 16 


// Settings from server (retrieved after first calibration)
uint16_t g_s_trigger_threshold=30;  // was g_threshold
uint32_t g_s_recalibration_interval_ms=600000; // was CALIBRATION_INTERVAL_MS
uint8_t g_s_http_retries=3; 
uint8_t g_s_drift_recalibration_threshold=10;
uint32_t g_s_max_sleep_interval_ms=10000;
uint32_t g_s_wifi_connect_timeout_ms=45000; // was WIFI_AT_BOOT_TIMEOUT
uint32_t g_s_recalibration_reading_count=100;
bool g_s_reboot_flag=false;
bool g_s_use_fake_sleep=true; // Was 
bool g_s_stay_connected=true;



const char * hostname="Influence";
const char * influence_mine_server_url="http://192.168.1.125/wifi_influence?message=";

bool g_freshly_booted=true; // Will be reset after first successful send of a report
char g_last_boot_reason[200];
uint8_t g_last_tries_required=0;
int8_t g_current_rssi=0;


AsyncWebServer  server(80);


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
    char last_boot_reason[200];
    bool freshly_booted;
    uint8_t last_tries_required;
    int8_t rssi;


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

  doc["last_boot_reason"]=message.last_boot_reason;
  doc["freshly_booted"]=message.freshly_booted;
  doc["last_tries_required"]=message.last_tries_required;
  doc["rssi"]=message.rssi;

  serializeJson(doc,buff,1023);


}





reading_t g_mean_reading;

Reading get_mean_reading()
{
  // does calibration...
  Serial.println("recalibrating.....");
  int32_t xacc=0;
  int32_t yacc=0;
  int32_t zacc=0;

  for (uint16_t i=0;i<g_s_recalibration_reading_count;i++)
  {
    compass.read();
    xacc+=compass.m.x;
    yacc+=compass.m.y;
    zacc+=compass.m.z;
    Serial.print("?");

    delay(140); // Wait for new data
  }
  printf("\nRaw mag accum over %d readings  :  %d,%d,%d\n",g_s_recalibration_reading_count,xacc,yacc,zacc);
  Reading res;
  res.x=int(xacc/(int32_t)g_s_recalibration_reading_count);
  res.y=int(yacc/(int32_t)g_s_recalibration_reading_count);
  res.z=int(zacc/(int32_t)g_s_recalibration_reading_count);
  printf("Norm'd mag accum over             : %d,%d,%d\n",res.x,res.y,res.z);
  
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
    Serial.print("Fake sleeping ......zzzzzz...");
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
  esp_sleep_enable_timer_wakeup(g_s_max_sleep_interval_ms);
  esp_light_sleep_start();
  
  //esp_deep_sleep_start();

  Serial.println("This will never be printed");
}


void signal_to_server_get_settings(const char * url)
{

  // Check the wifi is still up
  if (!WiFi.status()== WL_CONNECTED)
  {

    esp_restart(); // Actually maybe just sleep????
    return; // never happens but gotta keep the compiler happy
  }

  HTTPClient http;


  // char url[400];
  // strcpy(url,influence_mine_server_url);
  // strcat(url,wakeup_reason);

  uint8_t tries_left=g_s_http_retries;

  while (true)
  {
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

          // Parse payload into g_s_ parameters
          DynamicJsonDocument doc(1024);
          deserializeJson(doc, payload);
          uint16_t g_s_trigger_threshold=30;  // was g_threshold
          uint32_t g_s_recalibration_interval_ms=doc["recalibration_interval_ms"];
          uint8_t g_s_http_retries=doc["http_retries"];
          uint8_t g_s_drift_recalibration_threshold=doc["drift_recalibration_threshold"];
          uint32_t g_s_max_sleep_interval_ms=doc["max_sleep_interval_ms"];
          uint32_t g_s_wifi_connect_timeout_ms=doc["wifi_connect_timeout_ms"]; // was WIFI_AT_BOOT_TIMEOUT
          uint32_t g_s_recalibration_reading_count=doc["recalibration_reading_count"];
          bool g_s_reboot_flag=(bool)doc["reboot_flag"];
          bool g_s_use_fake_sleep=(bool)doc["use_fake_sleep"]; 
          bool g_s_stay_connected_new=(bool)doc["stay_connected"];
          if (g_s_stay_connected && !g_s_stay_connected_new)
          {
            // going offline, so shutdown the server stuff

            //ElegantOTA.end(); // No such function, hopefully it doesn't mind!
            server.end();
            Serial.println("Server shut down now!");
          }


          Serial.println("New settings:");
          Serial.printf("\ttrigger_threshold=%d\n",g_s_trigger_threshold);
          Serial.printf("\trecalibration_interval_ms=%d\n",g_s_recalibration_interval_ms);
          Serial.printf("\thttp_retries=%d\n",g_s_http_retries);
          Serial.printf("\tdrift_recalibration_threshold=%d\n",g_s_drift_recalibration_threshold);
          Serial.printf("\tmax_sleep_interval_ms=%d\n",g_s_max_sleep_interval_ms);
          Serial.printf("\twifi_connect_timeout_ms=%d\n",g_s_wifi_connect_timeout_ms);
          Serial.printf("\trecalibration_reading_count=%d",g_s_recalibration_reading_count);
          Serial.printf("\treboot_flag=%d\n",g_s_reboot_flag);
          Serial.printf("\tuse_fake_sleep=%d\n",g_s_use_fake_sleep);
          Serial.printf("\tstay_connected=%d\n",g_s_stay_connected);

          if (g_s_reboot_flag)
          {
            for (uint8_t i=20;i>0;i--)
            {
              Serial.printf("Going to reboot in %d seconds/n",i);
              delay(1000);
            }
            esp_restart();
          }
          http.end();
          g_freshly_booted=false; // reset the flag now we've send successfully to server once

          g_last_tries_required=g_s_http_retries-tries_left+1;
          g_current_rssi=WiFi.RSSI();
          return;





    } else {
          Serial.print("Error code: ");
          Serial.println(httpResponseCode);
          tries_left--;
          if (tries_left==0)
          {
            http.end();
            Serial.println("Given up on the http request");
            return;
          }
          g_last_tries_required=99; // Signal the previous message failed
    }
  }
  
  


}



void setup_server(void)
{
  if (!g_s_stay_connected)
  {
    Serial.println("Not running server as not permanently connected");
  }

  server.on("/",HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "This is the dawn light controller.\n"
                        "\t* Use /get_mag to return the current magnetic field strength\n"
                        "\tUse /update to install new firmware remotely (creds wificlassic)\n");
  });

  server.on("/get_mag",HTTP_GET, [](AsyncWebServerRequest *request) {
        char response[300];
        compass.readMag();
        reading_t latest=mag_as_reading(compass.m);
        sprintf(response,"{\"x\":%d,\"y\":%d,\"z\":%d}",latest.x,latest.y,latest.z);
        request->send(200, "text/plain", response);
        Serial.println(response);
  });

  ElegantOTA.begin(&server);    // Start AsyncElegantOTA
  ElegantOTA.setAuth(ota_username,ota_password);
  server.begin();
  Serial.println("HTTP server started");

}




void connect_wifi()
{
  if (WiFi.status()== WL_CONNECTED)
  {
    Serial.println("Non need to connect wifi, already connected.");
    return;
  }
  
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
  delay(200);
   /*                    

  ################################################################                                      
  
      JUST FOR THE 1.0.0 ESP32C3 Mini Lolin boards due to RF error
  
  
  */
  // WiFi.setTxPower(WIFI_POWER_8_5dBm); // May be required to connect

   /*                    

  ################################################################                                      
 
  
  */
  WiFi.begin(ssid, password);
  delay(200);
  Serial.printf("Connecting to: %s\n",ssid);
  Serial.println("");

  uint32_t wifi_timeout=millis()+g_s_wifi_connect_timeout_ms;

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

  if (g_s_stay_connected)
  {
    setup_server();
  }

  g_current_rssi=WiFi.RSSI();
  printf("Current RSSI %d\n",g_current_rssi);
    

}

void send_message_get_settings(const char * type,reading_t calib, reading_t latest, uint32_t variance)
{
      message_t mess=Message();
      mess.running_ms=millis();
      mess.variance=variance;
      mess.latest=latest;
      mess.calibration=calib;
      strcpy(mess.last_boot_reason,g_last_boot_reason);
      mess.freshly_booted=g_freshly_booted;
      mess.last_tries_required=g_last_tries_required;
      mess.rssi=g_current_rssi;
      strcpy(mess.type,type);
   
      get_json_message(report,mess);
      Serial.println(report);

      String message_url=String(influence_mine_server_url)+urlEncode(report);
      signal_to_server_get_settings(message_url.c_str());



      Serial.println("Fetched new settings");
      
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
  if (g_s_stay_connected)
  {
    Serial.println("Not switching off wifi as flag set to stay connected.");
    return;
  }
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
  get_wakeup_reason(g_last_boot_reason);
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
      g_mean_reading=get_mean_reading();
      next_calibration_due=millis()+g_s_recalibration_interval_ms;
      connect_wifi();



      send_message_get_settings("CALIBRATION_TIMED",g_mean_reading,g_mean_reading,0);
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

      if (variance>g_s_drift_recalibration_threshold)
      {
        g_mean_reading=get_mean_reading();//Recalibrate
        connect_wifi();
        send_message_get_settings("CALIBRATION_DRIFTED",g_mean_reading,g_mean_reading,0);
        
        wifi_off();
      }


    }


      

      uint32_t var=calc_variance(compass.m,g_mean_reading);

      if (var>g_s_trigger_threshold)
      {
        connect_wifi();
        send_message_get_settings("TRIGGER", \
                                          g_mean_reading,\
                                          mag_as_reading(compass.m), \
                                          var);
        wifi_off();
        // Recalibrate after each send
        g_mean_reading=get_mean_reading();

      }




      //snprintf(report, sizeof(report), "%d,%d,%d,,%d",
      //compass.m.x, compass.m.y, compass.m.z,var);
      //Serial.println(report);



      // Serial.println(report);
      // 
      // signal_to_server(report);
      // 

      

      if (g_s_use_fake_sleep)
      {
        fake_sleep();
      } else {
        sleep_now();
      }
      //

      
  }

  
}



  //signal_to_server();

