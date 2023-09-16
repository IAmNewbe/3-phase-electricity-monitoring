#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ThingsBoard.h>
#include <PZEM004Tv30.h>

#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#endif

#if !defined(PZEM_SERIAL)
#define PZEM_SERIAL Serial2
#endif

#define NUM_PZEMS 3


PZEM004Tv30 pzems[NUM_PZEMS];

/* ***************************************************************
 * Uncomment USE_SOFTWARE_SERIAL in order to enable Softare serial
 *
 * Does not work for ESP32
 *****************************************************************/
//#define USE_SOFTWARE_SERIAL



#if defined(USE_SOFTWARE_SERIAL) && defined(ESP32)
    #error "Can not use SoftwareSerial with ESP32"
#elif defined(USE_SOFTWARE_SERIAL)

#include <SoftwareSerial.h>

SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
#endif

//#define WIFI_AP "E5573-MIFI-381476" surabaya
//#define WIFI_PASSWORD "12345678"
// #define WIFI_AP "MTN-MobileWiFi-E5573" //gresik
// #define WIFI_PASSWORD "3JAF6A3J"
#define WIFI_AP "crustea" //gresik
#define WIFI_PASSWORD "12345678"

#define TOKEN "SG1tAxWUe3kLYLKXF0LG"
//#define TOKEN "uTHDviq2md1SZOKaCoX0"

#define GPIO23 23 //relay Pin 
#define GPIO2 2

#define GPIO23_PIN 14
//#define GPIO2_PIN 5

PZEM004Tv30 pzem1(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN  , 0x10);
PZEM004Tv30 pzem2(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN  , 0x11);
PZEM004Tv30 pzem3(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN  , 0x12);

char thingsboardServer[] = "18.140.254.213";
//char thingsboardServer[] = "thingsboard.cloud";

WiFiClient wifiClient;

PubSubClient client(wifiClient);

int status = WL_IDLE_STATUS;

ThingsBoard tb(wifiClient);

unsigned long lastSend;

// We assume that all GPIOs are LOW
boolean gpioState[] = {false, false};

float voltage, current, power, energy, frequency, pf, va, var;
float voltage_2, current_2, power_2, energy_2, frequency_2, pf_2, va_2, var_2;
float voltage_3, current_3, power_3, energy_3, frequency_3, pf_3, va_3, var_3;
float voltage3ph, current3ph, power3ph, energy3ph, frequency3ph, pf3ph, va3ph, var3ph;

void Read3PhasePzem(){
  /**
   * Fungsi untuk mendebug(menampilakan pada serial monitor) nilai-nilai yang didapatkan dari vaiabel yang diolah pada fungsi pzemRead.
   */
  
  voltage = pzem1.voltage();
  current = pzem1.current();
  power = pzem1.power();
  energy = pzem1.frequency();
  frequency = pzem1.frequency();
  pf = pzem1.pf();

  // voltage = 230;
  // current = 0.2;
  // power = 34;
  // energy = 50;
  // frequency = 50;
  // pf = 0.68;

  if (pf == 0 || isnan(pf)) {
  va = 0;
  } else {
    va = power / pf;
  }
  if (pf == 0 || isnan(pf)) {
    var = 0;
  } else {
    var = power / pf * sqrt(1-sq(pf));
  }

  voltage_2 = pzem2.voltage();
  current_2 = pzem2.current();
  power_2 = pzem2.power();
  energy_2 = pzem2.frequency();
  frequency_2 = pzem2.frequency();
  pf_2 = pzem2.pf();

  if (pf_2 == 0 || isnan(pf_2)) {
  va_2 = 0;
  } else {
    va_2 = power_2 / pf_2;
  }
  if (pf_2 == 0 || isnan(pf_2)) {
    var_2 = 0;
  } else {
    var_2 = power_2 / pf_2 * sqrt(1-sq(pf_2));
  }

  voltage_3 = pzem3.voltage();
  current_3 = pzem3.current();
  power_3 = pzem3.power();
  energy_3 = pzem3.frequency();
  frequency_3 = pzem3.frequency();
  pf_3 = pzem3.pf();

  if (pf_3 == 0 || isnan(pf_3)) {
  va_3 = 0;
  } else {
    va_3 = power_3 / pf_3;
  }
  if (pf_3 == 0 || isnan(pf_3)) {
    var_3 = 0;
  } else {
    var_3 = power_3 / pf_3 * sqrt(1-sq(pf_3));
  }

  voltage3ph = sqrt(sq(voltage) + sq(voltage_2) + sq(voltage_3));
 
  if ((current > 0) && (current_2 > 0) && (current_3 > 0)) {
  current3ph = 1/3 * (current+current_2+current_3);
  } else {
      if ((current == 0 || isnan(current)) && (current_2 > 0) && (current_3 > 0)) {
      current3ph = 1/2 * (current_2+current_3);
     }
      if ((current > 0) && (current_2 == 0 || isnan(current_2)) && (current_3 > 0)) {
      current3ph = 1/2 * (current+current_3);
     }
      if ((current > 0) && (current_2 > 0) && (current_3 == 0 || isnan(current_3))) {
      current3ph = 1/2 * (current+current_2);
     }
      if ((current > 0) && (current_2 == 0 || isnan(current_2)) && (current_3 == 0 || isnan(current_3))) {
      current3ph = current;
     }
     if ((current == 0 || isnan(current)) && (current_2 > 0) && (current_3 == 0 || isnan(current_3))) {
      current3ph = current_2;
     }
     if ((current == 0 || isnan(current)) && (current_2 == 0 || isnan(current_2)) && (current_3 > 0)) {
      current3ph = current_3;
     }
     if ((current == 0 || isnan(current)) && (current_2 == 0 || isnan(current_2)) && (current_3 == 0 || isnan(current_3))) {
      current3ph = 0;
     }
  }

  power3ph = (power + power_2 + power_3);
  energy3ph = (energy + energy_2 + energy_3);
  va3ph = (va + va_2 + va_3);
  var3ph = (var + var_2 + var_3);

  if ((frequency > 0) && (frequency_2 > 0) && (frequency_3 > 0)) {
  frequency3ph = 1/3 * (frequency + frequency_2 + frequency_3);
  } else {
    if((frequency > 0) && (frequency_2 > 0) && (frequency_3 == 0 || isnan(frequency_3))) {
    frequency3ph = 1/2 * (frequency+frequency_2);
    }
    if((frequency > 0) && (frequency_2 == 0 || isnan(frequency_2)) && (frequency_3 > 0)) {
    frequency3ph = 1/2 * (frequency+frequency_3);
    }
    if((frequency == 0 || isnan(frequency)) && (frequency_2 > 0) && (frequency_3 > 0)) {
    frequency3ph = 1/2 * (frequency_2+frequency_3);
    }
    if((frequency > 0) && (frequency_2 == 0 || isnan(frequency_2)) && (frequency_3 == 0 || isnan(frequency_3))) {
    frequency3ph = frequency;
    }
    if((frequency == 0 || isnan(frequency)) && (frequency_2 > 0) && (frequency_3 == 0 || isnan(frequency_3))) {
    frequency3ph = frequency_2;
    }
    if((frequency == 0 || isnan(frequency)) && (frequency_2 == 0 || isnan(frequency_2)) && (frequency_3 > 0)) {
    frequency3ph = frequency_3;
    }
    if((frequency == 0 || isnan(frequency)) && (frequency_2 == 0 || isnan(frequency_2)) && (frequency_3 == 0 || isnan(frequency_3))) {
    frequency3ph = 0;
    }
  }

  if ((pf > 0) && (pf_2 > 0) && (pf_3 > 0)) {
  pf3ph = 1/3 * (pf+pf_2+pf_3);
  } else {
    if((pf > 0) && (pf_2 > 0) && (pf_3 == 0 || isnan(pf_3))) {
    pf3ph = 1/2 * (pf+pf_2);
    }
    if((pf > 0) && (pf_2 == 0 || isnan(pf_2)) && (pf_3 > 0)) {
    pf3ph = 1/2 * (pf+pf_3);
    }
    if((pf == 0 || isnan(pf)) && (pf_2 > 0) && (pf_3 > 0)) {
    pf3ph = 1/2 * (pf_2+pf_3);
    }
    if((pf > 0) && (pf_2 == 0 || isnan(pf_2)) && (pf_3 == 0 || isnan(pf_3))) {
    pf3ph = pf;
    }
    if((pf == 0 || isnan(pf)) && (pf_2 > 0) && (pf_3 == 0 || isnan(pf_3))) {
    pf3ph = pf_2;
    }
    if((pf == 0 || isnan(pf)) && (pf_2 == 0 || isnan(pf_2)) && (pf_3 > 0)) {
    pf3ph = pf_3;
    }
    if((pf == 0 || isnan(pf)) && (pf_2 == 0 || isnan(pf_2)) && (pf_3 == 0 || isnan(pf_3))) {
    pf3ph = 0;
    }
  }
  Serial.println("Sending data to ThingsBoard:");
  // Serial.println("Voltage:", voltage);
  Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
  Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
  Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
  Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
  Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
  Serial.print("PF: ");           Serial.println(pf);
 
  tb.sendTelemetryFloat("Voltage_R", voltage);
  tb.sendTelemetryFloat("Power_R", power);
  tb.sendTelemetryFloat("Pf_R", pf);
  tb.sendTelemetryFloat("Current_R", current);
  tb.sendTelemetryFloat("Frekuensi_R", frequency);
  tb.sendTelemetryFloat("Energy_R", energy);
  tb.sendTelemetryFloat("ApparentPower_R", va);
  tb.sendTelemetryFloat("ReactivePower_R", var);

  tb.sendTelemetryFloat("Voltage_S", voltage_2);
  tb.sendTelemetryFloat("Power_S", power_2);
  tb.sendTelemetryFloat("Pf_S", pf_2);
  tb.sendTelemetryFloat("Current_S", current_2);
  tb.sendTelemetryFloat("Frekuensi_S", frequency_2);
  tb.sendTelemetryFloat("Energy_S", energy_2);
  tb.sendTelemetryFloat("ApparentPower_S", va_2);
  tb.sendTelemetryFloat("ReactivePower_S", var_2);

  tb.sendTelemetryFloat("Voltage_T", voltage_3);
  tb.sendTelemetryFloat("Power_T", power_3);
  tb.sendTelemetryFloat("Pf_T", pf_3);
  tb.sendTelemetryFloat("Current_T", current_3);
  tb.sendTelemetryFloat("Frekuensi_T", frequency_3);
  tb.sendTelemetryFloat("Energy_T", energy_3);
  tb.sendTelemetryFloat("ApparentPower_T", va_3);
  tb.sendTelemetryFloat("ReactivePower_T", var_3);

  tb.sendTelemetryFloat("Voltage_3ph", voltage3ph);
  tb.sendTelemetryFloat("Power_3ph", power3ph);
  tb.sendTelemetryFloat("Pf_3ph", pf3ph);
  tb.sendTelemetryFloat("Current_3ph", current3ph);
  tb.sendTelemetryFloat("Frekuensi_3ph", frequency3ph);
  tb.sendTelemetryFloat("Energy_3ph", energy3ph);
  tb.sendTelemetryFloat("ApparentPower_3ph", va3ph);
  tb.sendTelemetryFloat("ReactivePower_3ph", var3ph);
}

void pzemMonitor(){

  /**
   * Fungsi untuk mendebug(menampilakan pada serial monitor) nilai-nilai yang didapatkan dari vaiabel yang diolah pada fungsi pzemRead.
   */
   
      for(int i = 10; i <= 12; i++){
        // Print the Address of the PZEM
        Serial.print("PZEM ");
        Serial.print(i);
        Serial.print(" - Address:");
        Serial.println(pzems[i].getAddress(), HEX);
        Serial.println("===================");
        
        if ( i == 10){
          voltage = pzems[i].voltage();
          current = pzems[i].current();
          power = pzems[i].power();
          energy = pzems[i].energy();
          frequency = pzems[i].frequency();
          pf = pzems[i].pf();

          if (pf == 0) {
          va = 0;
          } else {
            va = power / pf;
          }
          if (pf == 0) {
            var = 0;
          } else {
            var = power / pf * sqrt(1-sq(pf));
          }

        }else if ( i == 11){
          voltage_2 = pzems[i].voltage();
          current_2 = pzems[i].current();
          power_2 = pzems[i].power();
          energy_2 = pzems[i].energy();
          frequency_2 = pzems[i].frequency();
          pf_2 = pzems[i].pf();

          if (pf_2 == 0) {
          va_2 = 0;
          } else {
            va_2 = power_2 / pf_2;
          }
          if (pf_2 == 0) {
            var_2 = 0;
          } else {
            var_2 = power_2 / pf_2 * sqrt(1-sq(pf_2));
          }
          
        }else if ( i == 12) {
          voltage_3 = pzems[i].voltage();
          current_3 = pzems[i].current();
          power_3 = pzems[i].power();
          energy_3 = pzems[i].energy();
          frequency_3 = pzems[i].frequency();
          pf_3 = pzems[i].pf();

          if (pf_3 == 0) {
          va_3 = 0;
          } else {
            va_3 = power_3 / pf_3;
          }
          if (pf_3 == 0) {
            var_3 = 0;
          } else {
            var_3 = power_3 / pf_3 * sqrt(1-sq(pf_3));
          }
        }

        // Read the data from the sensor

        // Check if the data is valid
        if(isnan(voltage)){
            Serial.println("Error reading voltage");
        } else if (isnan(current)) {
            Serial.println("Error reading current");
        } else if (isnan(power)) {
            Serial.println("Error reading power");
        } else if (isnan(energy)) {
            Serial.println("Error reading energy");
        } else if (isnan(frequency)) {
            Serial.println("Error reading frequency");
        } else if (isnan(pf)) {
            Serial.println("Error reading power factor");
        } else {
            // Print the values to the Serial console
            Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
            Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
            Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
            Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
            Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
            Serial.print("PF: ");           Serial.println(pf);

        }
        Serial.println("-------------------");
        Serial.println();
    }

    Serial.println();
    delay(2000);
}

void powerSystem(int relayMode){

  /**
   * Fungsi untuk menghidupkan dan mematikan relay 
   * Digunakan untuk nyala mati aerator 
   * Panggil fungsi dengan parameter 0 untuk menyalakan relay dan 1 untuk mematikan relay
   */
   
  if(relayMode == 0){
    // Turn the relay switch ON 
    digitalWrite(GPIO23, LOW);// set relay pin to low 
    Serial.println("Relay ON ");
  }else if(relayMode == 1){
    // Turn the relay switch OFF 
    digitalWrite(GPIO23, HIGH);// set relay pin to HIGH
    Serial.println("Relay OFF ");
  }

}

void getAndSendData(){
  voltage3ph = sqrt(sq(voltage) + sq(voltage_2) + sq(voltage_3));
 
  if ((current > 0) && (current_2 > 0) && (current_3 > 0)) {
  current3ph = 1/3 * (current+current_2+current_3);
  } else {
      if ((current == 0) && (current_2 > 0) && (current_3 > 0)) {
      current3ph = 1/2 * (current_2+current_3);
     }
      if ((current > 0) && (current_2 == 0) && (current_3 > 0)) {
      current3ph = 1/2 * (current+current_3);
     }
      if ((current > 0) && (current_2 > 0) && (current_3 == 0)) {
      current3ph = 1/2 * (current+current_2);
     }
      if ((current > 0) && (current_2 == 0) && (current_3 == 0)) {
      current3ph = current;
     }
     if ((current == 0) && (current_2 > 0) && (current_3 == 0)) {
      current3ph = current_2;
     }
     if ((current == 0) && (current_2 == 0) && (current_3 > 0)) {
      current3ph = current_3;
     }
     if ((current == 0) && (current_2 == 0) && (current_3 == 0)) {
      current3ph = 0;
     }
  }

  power3ph = (power + power_2 + power_3);
  energy3ph = (energy + energy_2 + energy_3);
  va3ph = (va + va_2 + va_3);
  var3ph = (var + var_2 + var_3);

  if ((frequency > 0) && (frequency_2 > 0) && (frequency_3 > 0)) {
  frequency3ph = 1/3 * (frequency + frequency_2 + frequency_3);
  } else {
    if((frequency > 0) && (frequency_2 > 0) && (frequency_3 == 0)) {
    frequency3ph = 1/2 * (frequency+frequency_2);
    }
    if((frequency > 0) && (frequency_2 == 0) && (frequency_3 > 0)) {
    frequency3ph = 1/2 * (frequency+frequency_3);
    }
    if((frequency == 0) && (frequency_2 > 0) && (frequency_3 > 0)) {
    frequency3ph = 1/2 * (frequency_2+frequency_3);
    }
    if((frequency > 0) && (frequency_2 == 0) && (frequency_3 == 0)) {
    frequency3ph = frequency;
    }
    if((frequency == 0) && (frequency_2 > 0) && (frequency_3 == 0)) {
    frequency3ph = frequency_2;
    }
    if((frequency == 0) && (frequency_2 == 0) && (frequency_3 > 0)) {
    frequency3ph = frequency_3;
    }
    if((frequency == 0) && (frequency_2 == 0) && (frequency_3 == 0)) {
    frequency3ph = 0;
    }
  }

  if ((pf > 0) && (pf_2 > 0) && (pf_3 > 0)) {
  pf3ph = 1/3 * (pf+pf_2+pf_3);
  } else {
    if((pf > 0) && (pf_2 > 0) && (pf_3 == 0)) {
    pf3ph = 1/2 * (pf+pf_2);
    }
    if((pf > 0) && (pf_2 == 0) && (pf_3 > 0)) {
    pf3ph = 1/2 * (pf+pf_3);
    }
    if((pf == 0) && (pf_2 > 0) && (pf_3 > 0)) {
    pf3ph = 1/2 * (pf_2+pf_3);
    }
    if((pf > 0) && (pf_2 == 0) && (pf_3 == 0)) {
    pf3ph = pf;
    }
    if((pf == 0) && (pf_2 > 0) && (pf_3 == 0)) {
    pf3ph = pf_2;
    }
    if((pf == 0) && (pf_2 == 0) && (pf_3 > 0)) {
    pf3ph = pf_3;
    }
    if((pf == 0) && (pf_2 == 0) && (pf_3 == 0)) {
    pf3ph = 0;
    }
  }
  Serial.println("Sending data to ThingsBoard:");
 
  tb.sendTelemetryFloat("Voltage_R", voltage);
  tb.sendTelemetryFloat("Power_R", power);
  tb.sendTelemetryFloat("Pf_R", pf);
  tb.sendTelemetryFloat("Current_R", current);
  tb.sendTelemetryFloat("Frekuensi_R", frequency);
  tb.sendTelemetryFloat("Energy_R", energy);
  tb.sendTelemetryFloat("ApparentPower_R", va);
  tb.sendTelemetryFloat("ReactivePower_R", var);

  tb.sendTelemetryFloat("Voltage_S", voltage_2);
  tb.sendTelemetryFloat("Power_S", power_2);
  tb.sendTelemetryFloat("Pf_S", pf_2);
  tb.sendTelemetryFloat("Current_S", current_2);
  tb.sendTelemetryFloat("Frekuensi_S", frequency_2);
  tb.sendTelemetryFloat("Energy_S", energy_2);
  tb.sendTelemetryFloat("ApparentPower_S", va_2);
  tb.sendTelemetryFloat("ReactivePower_S", var_2);

  tb.sendTelemetryFloat("Voltage_T", voltage_3);
  tb.sendTelemetryFloat("Power_T", power_3);
  tb.sendTelemetryFloat("Pf_T", pf_3);
  tb.sendTelemetryFloat("Current_T", current_3);
  tb.sendTelemetryFloat("Frekuensi_T", frequency_3);
  tb.sendTelemetryFloat("Energy_T", energy_3);
  tb.sendTelemetryFloat("ApparentPower_T", va_3);
  tb.sendTelemetryFloat("ReactivePower_T", var_3);

  tb.sendTelemetryFloat("Voltage_3ph", voltage3ph);
  tb.sendTelemetryFloat("Power_3ph", power3ph);
  tb.sendTelemetryFloat("Pf_3ph", pf3ph);
  tb.sendTelemetryFloat("Current_3ph", current3ph);
  tb.sendTelemetryFloat("Frekuensi_3ph", frequency3ph);
  tb.sendTelemetryFloat("Energy_3ph", energy3ph);
  tb.sendTelemetryFloat("ApparentPower_3ph", va3ph);
  tb.sendTelemetryFloat("ReactivePower_3ph", var3ph);

}

void setup() {
  Serial.begin(115200);
  pinMode(GPIO23, OUTPUT);
  pinMode(GPIO2, OUTPUT);
  delay(10);
  InitWiFi();
  client.setServer( thingsboardServer, 1883 );
  client.setCallback(on_message);

  lastSend = 0;
}

void loop() {
  if ( !client.connected()  ) {
    reconnect();
  }

  if ( !tb.connected() ) {
    reconnect();
  }

  if ( millis() - lastSend > 5000 ) { // Update and send only after 1 seconds
    // getAndSendData();
    Read3PhasePzem();
//    tb.loop();
    lastSend = millis();
  }

  client.loop();
  // pzemMonitor();
}

// The callback for when a PUBLISH message is received from the server.
void on_message(const char* topic, byte* payload, unsigned int length) {

  Serial.println("On message");

  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(json);

  // Decode JSON request
  StaticJsonDocument<200> jsonBuffer;
  DeserializationError data = deserializeJson(jsonBuffer, (char*)json);

  if (data)
  {
    Serial.println("parseObject() failed");
    return;
  }

  // Check request method
  String methodName = String((const char*)jsonBuffer["method"]);

  if (methodName.equals("getGpioStatus")) {
    // Reply with GPIO status
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
  } else if (methodName.equals("setGpioStatus")) {
    // Update GPIO status and reply
    set_gpio_status(jsonBuffer["params"]["pin"], jsonBuffer["params"]["enabled"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
  }
}

String get_gpio_status() {
  // Prepare gpios JSON payload string
  StaticJsonDocument<200> jsonBuffer;
  JsonObject data = jsonBuffer.to<JsonObject>();
  data[String(GPIO23_PIN)] = gpioState[0] ? true : false;
//  data[String(GPIO2_PIN)] = gpioState[1] ? true : false;
  char payload[256];
  serializeJson(jsonBuffer, payload);
  String strPayload = String(payload);
  Serial.print("Get gpio status: ");
  Serial.println(strPayload);
  return strPayload;
}

void set_gpio_status(int pin, boolean enabled) {
  if (pin == GPIO23_PIN) {
    // Output GPIOs state
    digitalWrite(GPIO23, enabled ? HIGH : LOW);
    digitalWrite(GPIO2, enabled ? HIGH : LOW);
    // Update GPIOs state
    gpioState[0] = enabled;
  } 
//  else if (pin == GPIO2_PIN) {
//    // Output GPIOs state
//    digitalWrite(GPIO2, enabled ? HIGH : LOW);
//    // Update GPIOs state
//    gpioState[1] = enabled;
//  }
}

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("ESP8266 Device", TOKEN, NULL) ) {
      Serial.println( "[DONE]" );
      // Subscribing to receive RPC requests
      client.subscribe("v1/devices/me/rpc/request/+");
      // Sending current GPIO status
      Serial.println("Sending current GPIO status ...");
      client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }

  while (!tb.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[DONE]" );
      // Subscribing to receive RPC requests
      client.subscribe("v1/devices/me/rpc/request/+");
      // Sending current GPIO status
      Serial.println("Sending current GPIO status ...");
      client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
    } else {
      Serial.print( "[FAILED]" );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}
