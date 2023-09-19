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

#define WIFI_AP "crustea" //gresik
#define WIFI_PASSWORD "12345678"

// #define TOKEN "SG1tAxWUe3kLYLKXF0LG"
#define TOKEN "TIGA_FASA_RISET" //testing device

#define GPIO23 23 //relay Pin 
#define GPIO2 2

#define GPIO23_PIN 14
//#define GPIO2_PIN 5

PZEM004Tv30 pzem1(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN  , 0x10);
PZEM004Tv30 pzem2(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN  , 0x11);
PZEM004Tv30 pzem3(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN  , 0x12);

// char thingsboardServer[] = "18.140.254.213";
char thingsboardServer[] = "thingsboard.cloud";

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

void getAndSendData(){

  if ((voltage > 0) && (voltage_2 > 0) && (voltage_3 > 0)) {
    voltage3ph = sqrt(sq(voltage) + sq(voltage_2) + sq(voltage_3));
  } else {
      if ((voltage == 0 || isnan(voltage)) && (voltage_2 > 0) && (voltage_3 > 0)) {
        voltage = 0;
        voltage3ph = sqrt(sq(voltage) + sq(voltage_2) + sq(voltage_3));
     }
      if ((voltage > 0) && (voltage_2 == 0 || isnan(voltage_2)) && (voltage_3 > 0)) {
        voltage_2 = 0;
        voltage3ph = sqrt(sq(voltage) + sq(voltage_2) + sq(voltage_3));
     }
      if ((voltage > 0) && (voltage_2 > 0) && (voltage_3 == 0 || isnan(voltage_3))) {
        voltage_3 = 0;
        voltage3ph = sqrt(sq(voltage) + sq(voltage_2) + sq(voltage_3));
     }
      if ((voltage > 0) && (voltage_2 == 0 || isnan(voltage_2)) && (voltage_3 == 0 || isnan(voltage_3))) {
        voltage3ph = voltage;
     }
     if ((voltage == 0 || isnan(voltage)) && (voltage_2 > 0) && (voltage_3 == 0 || isnan(voltage_3))) {
        voltage3ph = voltage_2;
     }
     if ((voltage == 0 || isnan(voltage)) && (voltage_2 == 0 || isnan(voltage_2)) && (voltage_3 > 0)) {
        voltage3ph = voltage_3;
     }
     if ((voltage == 0 || isnan(voltage)) && (voltage_2 == 0 || isnan(voltage_2)) && (voltage_3 == 0 || isnan(voltage_3))) {
        voltage3ph = 0;
     }
  }

  // voltage3ph = sqrt(sq(voltage) + sq(voltage_2) + sq(voltage_3));
 
  if ((current > 0) && (current_2 > 0) && (current_3 > 0)) {
  current3ph = (current+current_2+current_3) / 3;
  } else {
      if ((current == 0 || isnan(current)) && (current_2 > 0) && (current_3 > 0)) {
      current3ph = (current_2+current_3) / 2;
     }
      if ((current > 0) && (current_2 == 0 || isnan(current_2)) && (current_3 > 0)) {
      current3ph = (current+current_3) / 2;
     }
      if ((current > 0) && (current_2 > 0) && (current_3 == 0 || isnan(current_3))) {
      current3ph = (current+current_2) / 2;
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

  if ((power > 0) && (power_2 > 0) && (power_3 > 0)) {
    power3ph = (power + power_2 + power_3);
  } else {
      if ((power == 0 || isnan(power)) && (power_2 > 0) && (power_3 > 0)) {
        power = 0;
        power3ph = (power + power_2 + power_3);
     }
      if ((power > 0) && (power_2 == 0 || isnan(power_2)) && (power_3 > 0)) {
        power_2 = 0;
        power3ph = (power + power_2 + power_3);
     }
      if ((power > 0) && (power_2 > 0) && (power_3 == 0 || isnan(power_3))) {
        power_3 = 0;
        power3ph = (power + power_2 + power_3);
     }
      if ((power > 0) && (power_2 == 0 || isnan(power_2)) && (power_3 == 0 || isnan(power_3))) {
        power3ph = power;
     }
     if ((power == 0 || isnan(power)) && (power_2 > 0) && (power_3 == 0 || isnan(power_3))) {
        power3ph = power_2;
     }
     if ((power == 0 || isnan(power)) && (power_2 == 0 || isnan(power_2)) && (power_3 > 0)) {
        power3ph = power_3;
     }
     if ((power == 0 || isnan(power)) && (power_2 == 0 || isnan(power_2)) && (power_3 == 0 || isnan(power_3))) {
        power3ph = 0;
     }
  }

  // power3ph = (power + power_2 + power_3);
  if ((energy > 0) && (energy_2 > 0) && (energy_3 > 0)) {
    energy3ph = (energy + energy_2 + energy_3);
  } else {
      if ((energy == 0 || isnan(energy)) && (energy_2 > 0) && (energy_3 > 0)) {
        energy = 0;
        energy3ph = (energy + energy_2 + energy_3);
     }
      if ((energy > 0) && (energy_2 == 0 || isnan(energy_2)) && (energy_3 > 0)) {
        energy_2 = 0;
        energy3ph = (energy + energy_2 + energy_3);
     }
      if ((energy > 0) && (energy_2 > 0) && (energy_3 == 0 || isnan(energy_3))) {
        energy_3 = 0;
        energy3ph = (energy + energy_2 + energy_3);
     }
      if ((energy > 0) && (energy_2 == 0 || isnan(energy_2)) && (energy_3 == 0 || isnan(energy_3))) {
        energy3ph = energy;
     }
     if ((energy == 0 || isnan(energy)) && (energy_2 > 0) && (energy_3 == 0 || isnan(energy_3))) {
        energy3ph = energy_2;
     }
     if ((energy == 0 || isnan(energy)) && (energy_2 == 0 || isnan(energy_2)) && (energy_3 > 0)) {
        energy3ph = energy_3;
     }
     if ((energy == 0 || isnan(energy)) && (energy_2 == 0 || isnan(energy_2)) && (energy_3 == 0 || isnan(energy_3))) {
        energy3ph = 0;
     }
  }
  // energy3ph = (energy + energy_2 + energy_3);
  if ((va > 0) && (va_2 > 0) && (va_3 > 0)) {
    va3ph = (va + va_2 + va_3);
  } else {
      if ((va == 0 || isnan(va)) && (va_2 > 0) && (va_3 > 0)) {
        va = 0;
        va3ph = (va + va_2 + va_3);
     }
      if ((va > 0) && (va_2 == 0 || isnan(va_2)) && (va_3 > 0)) {
        va_2 = 0;
        va3ph = (va + va_2 + va_3);
     }
      if ((va > 0) && (va_2 > 0) && (va_3 == 0 || isnan(va_3))) {
        va_3 = 0;
        va3ph = (va + va_2 + va_3);
     }
      if ((va > 0) && (va_2 == 0 || isnan(va_2)) && (va_3 == 0 || isnan(va_3))) {
        va3ph = va;
     }
     if ((va == 0 || isnan(va)) && (va_2 > 0) && (va_3 == 0 || isnan(va_3))) {
        va3ph = va_2;
     }
     if ((va == 0 || isnan(va)) && (va_2 == 0 || isnan(va_2)) && (va_3 > 0)) {
        va3ph = va_3;
     }
     if ((va == 0 || isnan(va)) && (va_2 == 0 || isnan(va_2)) && (va_3 == 0 || isnan(va_3))) {
        va3ph = 0;
     }
  }
  // va3ph = (va + va_2 + va_3);
  if ((var > 0) && (var_2 > 0) && (var_3 > 0)) {
    var3ph = (var + var_2 + var_3);
  } else {
      if ((var == 0 || isnan(var)) && (var_2 > 0) && (var_3 > 0)) {
        var = 0;
        var3ph = (var + var_2 + var_3);
     }
      if ((var > 0) && (var_2 == 0 || isnan(var_2)) && (var_3 > 0)) {
        var_2 = 0;
        var3ph = (var + var_2 + var_3);
     }
      if ((var > 0) && (var_2 > 0) && (var_3 == 0 || isnan(var_3))) {
        var_3 = 0;
        var3ph = (var + var_2 + var_3);
     }
      if ((var > 0) && (var_2 == 0 || isnan(var_2)) && (var_3 == 0 || isnan(var_3))) {
        var3ph = var;
     }
     if ((var == 0 || isnan(var)) && (var_2 > 0) && (var_3 == 0 || isnan(var_3))) {
        var3ph = var_2;
     }
     if ((var == 0 || isnan(var)) && (var_2 == 0 || isnan(var_2)) && (var_3 > 0)) {
        var3ph = var_3;
     }
     if ((var == 0 || isnan(var)) && (var_2 == 0 || isnan(var_2)) && (var_3 == 0 || isnan(var_3))) {
        var3ph = 0;
     }
  }
  // var3ph = (var + var_2 + var_3);

  if ((frequency > 0) && (frequency_2 > 0) && (frequency_3 > 0)) {
  frequency3ph =  (frequency + frequency_2 + frequency_3) /3;
  } else {
    if((frequency > 0) && (frequency_2 > 0) && (frequency_3 == 0 || isnan(frequency_3))) {
    frequency3ph = (frequency+frequency_2) / 2;
    }
    if((frequency > 0) && (frequency_2 == 0 || isnan(frequency_2)) && (frequency_3 > 0)) {
    frequency3ph = (frequency+frequency_3) / 2;
    }
    if((frequency == 0 || isnan(frequency)) && (frequency_2 > 0) && (frequency_3 > 0)) {
    frequency3ph = (frequency_2+frequency_3) / 2;
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
  pf3ph = (pf+pf_2+pf_3) / 3;
  } else {
    if((pf > 0) && (pf_2 > 0) && (pf_3 == 0 || isnan(pf_3))) {
    pf3ph = (pf+pf_2) / 2;
    }
    if((pf > 0) && (pf_2 == 0 || isnan(pf_2)) && (pf_3 > 0)) {
    pf3ph = (pf+pf_3) / 2;
    }
    if((pf == 0 || isnan(pf)) && (pf_2 > 0) && (pf_3 > 0)) {
    pf3ph = (pf_2+pf_3) / 2;
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

void Read3PhasePzem(){
  /**
   * Fungsi untuk mendebug(menampilakan pada serial monitor) nilai-nilai yang didapatkan dari vaiabel yang diolah pada fungsi pzemRead.
   */
  
  // voltage = pzem1.voltage();
  // current = pzem1.current();
  // power = pzem1.power();
  // energy = pzem1.energy();
  // frequency = pzem1.frequency();
  // pf = pzem1.pf();

  voltage = 230;
  current = 0.2;
  power = 34;
  energy = 50;
  frequency = 50;
  pf = 0.68;

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

  // voltage_2 = pzem2.voltage();
  // current_2 = pzem2.current();
  // power_2 = pzem2.power();
  // energy_2 = pzem2.energy();
  // frequency_2 = pzem2.frequency();
  // pf_2 = pzem2.pf();
  
  voltage_2 = 232;
  current_2 = 0.23;
  power_2 = 44;
  energy_2 = 60;
  frequency_2 = 50;
  pf_2 = 0.78;

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

  // voltage_3 = pzem3.voltage();
  // current_3 = pzem3.current();
  // power_3 = pzem3.power();
  // energy_3 = pzem3.energy();
  // frequency_3 = pzem3.frequency();
  // pf_3 = pzem3.pf();

  voltage_3 = 222;
  current_3 = 0.13;
  power_3 = 54;
  energy_3 = 60;
  frequency_3 = 50;
  pf_3 = 0.88;

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

}

void setup() {
  Serial.begin(115200);
  pinMode(GPIO23, OUTPUT);
  pinMode(GPIO2, OUTPUT);
  delay(10);
  InitWiFi();
  client.setServer( thingsboardServer, 1883 );
  // client.setCallback(on_message);

  lastSend = 0;
}

void loop() {
  if ( !client.connected()  ) {
    reconnect();
  }

  if ( !tb.connected() ) {
    reconnect();
  }

  if ( millis() - lastSend > 1000 ) { // Update and send only after 1 seconds
    getAndSendData();
//    tb.loop();
    lastSend = millis();
  }

  client.loop();
  Read3PhasePzem();
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
      // client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
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
      // client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
    } else {
      Serial.print( "[FAILED]" );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}
