// BMP085 sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
// DHT sensor
#include "DHT.h"

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

#define DHTPIN 2     
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// interval of measurements from sensors
const int SECONDS_BETWEEN_MEASUREMENTS = 60; 
// intervals for stored measurements for the calculation of trends
const int SECONDS_BETWEEN_STORED_MEASUREMENTS = 300;
// number of measurements to use for trends
const int MAX_NUMBER_OF_STORED_MEASUREMENTS = 12;

float temperature = 0;
float humidity;
float pressure;
float measurements[MAX_NUMBER_OF_STORED_MEASUREMENTS];
int measurementsTaken = -1;
int measurementsStored = 0;

unsigned long previousMillis = 0;

/* 
calculate the moving average by dividing the measurements into two groups:
compare then the average of the earlier measures (first half of measurements)
against the average of the later ones and return the diff
*/
float getPressureTrend(){
  // if we have no enough measurements stored yet
  if(measurementsStored < 2){
    return 0.0;
  }

  float avg1 = 0.0;
  float avg2 = 0.0;
  float diff = 0.0; // holds the diff between avg2 and avg1
  float total1 = 0.0;
  float total2 = 0.0;
  int numElems1 = 0;
  int numElems2 = 0;

  int indexFirstMeasurement = MAX_NUMBER_OF_STORED_MEASUREMENTS - measurementsStored;
  
  // Start at the first element with actual data
  for(int i=indexFirstMeasurement; i<MAX_NUMBER_OF_STORED_MEASUREMENTS; i++){
    // separate measurements into two groups, 
    // done earlier (total1) and later (total2)
    if(i < (indexFirstMeasurement + (measurementsStored/2))){
      numElems1++;
      // * converting to int so we don't need to do float math
      total1 = total1 + (measurements[i] * 100);
    } else {
      numElems2++;
      total2 = total2 + (measurements[i] * 100);
    }
  }
  // get the average and compute variance
  avg1 = total1 / numElems1;
  avg2 = total2 / numElems2;
  diff = avg2 - avg1;

  return (float) (diff / 100);
}

// moves the stored measurements one place to the "left" 
// and inserts new measurement at the end
// a kind of simple FIFO queue
void addPressureMeasurement(float newMeasurement){
  for(int i=0; i<MAX_NUMBER_OF_STORED_MEASUREMENTS; i++){      
      if(i != (MAX_NUMBER_OF_STORED_MEASUREMENTS-1)){    
        measurements[i] = measurements[i+1];
      } else{      
        measurements[i] = newMeasurement;
        if(measurementsStored < 12){
          measurementsStored++;
        }
      }
  }
}

// calculate the change per hour recorded
float getPressureChangePerHour(float trend){
  float hours = (float)(measurementsStored * SECONDS_BETWEEN_MEASUREMENTS) / 3600;

  return trend / hours;
}

void setup(void) {
  Serial.begin(9600);
  
  /* Initialise the BMP085 sensor */
  if(!bmp.begin()){
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  // initialise the DHT sensor
  dht.begin();
}

void loop(void) {
  unsigned long currentMillis = millis();
  unsigned long interval = (SECONDS_BETWEEN_MEASUREMENTS * 1000);

  if ((currentMillis - previousMillis >= interval) || (previousMillis == 0)) { 
    previousMillis = currentMillis;
  
    /* Get a new sensor event for BMP085 */ 
    sensors_event_t event;
    bmp.getEvent(&event);
    // get values
    pressure = event.pressure;
    // corrected pressure (sensor assumes we are at sea level, we are at 86m)
    // there is 1 extra hPa per 8.5m
    // So we need to add some 10hPa
    pressure = pressure + 10.0;

    if(measurementsTaken == -1) { // special case when this is run for first time
        addPressureMeasurement(pressure);
        measurementsTaken = 0;
    }
    measurementsTaken++;

    if(measurementsTaken == (SECONDS_BETWEEN_STORED_MEASUREMENTS / SECONDS_BETWEEN_MEASUREMENTS)){
      addPressureMeasurement(pressure);
      measurementsTaken = 0;
    }

    bmp.getTemperature(&temperature);
    humidity = dht.readHumidity();

    // get trend of pressure
    float trend = getPressureTrend();
    float hourChange = getPressureChangePerHour(trend);

    if (!event.pressure){
        Serial.println("BMP Sensor error");
    }
    if(!humidity){
        Serial.println("DHT sensor error");
    }

    /* Display values */
    Serial.print("Pressure:    ");
    Serial.print(pressure);
    Serial.println(" hPa");
        
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.println("");
    
    
    Serial.print("Trend of pressure: ");
    if(trend > 0) { // negative sign is always output
      Serial.print("+");
    }
    char buffer[8];
    dtostrf( trend, 6, 2, buffer );
    Serial.print(buffer);

    // change per hour
    dtostrf(hourChange, 5, 2, buffer);
    Serial.print(". Change/hr:  ");
    if(hourChange > 0) { // negative sign is always output
      Serial.print("+");
    }
    Serial.print(buffer);
    Serial.print(" hPa/hr");
    Serial.println("");

    Serial.println("Stored measurements:");
    for(int i=0; i<MAX_NUMBER_OF_STORED_MEASUREMENTS; i++){ 
      Serial.print(measurements[i]);
      Serial.print(" | ");
    }
    Serial.println("");
    Serial.println("------------------------");
    Serial.println("");
  }
}