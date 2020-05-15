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
// we use these in long calculations so need to be long
const unsigned long SECONDS_BETWEEN_MEASUREMENTS = 60; 
// intervals for stored measurements for the calculation of trends
const unsigned long SECONDS_BETWEEN_STORED_MEASUREMENTS = 300;
// number of measurements to use for trends
const int MAX_NUMBER_OF_STORED_MEASUREMENTS = 12;
// stores the measurements of pressure at intervals given by SECONDS_BETWEEN_STORED_MEASUREMENTS
float pressureMeasurements[MAX_NUMBER_OF_STORED_MEASUREMENTS];
// same for temperature
float temperatureMeasurements[MAX_NUMBER_OF_STORED_MEASUREMENTS];
// and humidity
float humidityMeasurements[MAX_NUMBER_OF_STORED_MEASUREMENTS];
// keeps track of all measurements taken, NOT only stored ones
int measurementsTaken = -1;
// keeps track of measurements stored, up to the MAX_NUMBER_OF_STORED_MEASUREMENTS
int measurementsStored = 0;
// for working out delays
unsigned long previousMillis = 0;

/*!
Calculate the diff between the moving average.
1. divide the measurements into two groups: the earlier measurements (first half), and later ones. 
2. Calculate avg in both groups
3. Subract avg of second group (later) from first one (earlier measurements)
@return: float with the difference between moving avg
*/
float getTrend(float *storage){
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
      total1 = total1 + storage[i];
    } else {
      numElems2++;
      total2 = total2 + storage[i];
    }
  }
  // get the averages and diff
  avg1 = total1 / numElems1;
  avg2 = total2 / numElems2;
  diff = avg2 - avg1;

  return diff;
}

/*! 
  Calculate the change in pressure per hour recorded.
  @param trend, float with the calculated trend (difference in moving avg)
  @return float with the change in hPa per hour 
*/
float getChangePerHour(float trend){
  float hours = (float)(measurementsStored * SECONDS_BETWEEN_STORED_MEASUREMENTS) / 3600;
  return (trend / hours);
}

/*! 
  Moves the stored measurements one place to the "left" 
  and inserts new measurement at the end
  (a kind of simple FIFO queue).
  @param newMeasurement: the measurement taken to be stored
  @param storage: pointer to the array containing the measurements (pressure, temperature or humidity)
*/
void storeMeasurement(float newMeasurement, float *storage){
  for(int i=0; i<MAX_NUMBER_OF_STORED_MEASUREMENTS; i++){      
      if(i != (MAX_NUMBER_OF_STORED_MEASUREMENTS-1)){    
          storage[i] = storage[i+1];
      } else{      
          storage[i] = newMeasurement;
      }
  }
}

void storeAllMeasurements(float pressure, float temperature, float humidity){
  storeMeasurement(pressure, pressureMeasurements);
  storeMeasurement(temperature, temperatureMeasurements);
  storeMeasurement(humidity, humidityMeasurements);
  if(measurementsStored < 12){
    measurementsStored++;
  }
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
  // this one returns void so we cannot error-check as above
  dht.begin();
}

void loop(void) {
  unsigned long currentMillis = millis();
  unsigned long interval = (SECONDS_BETWEEN_MEASUREMENTS * 1000);
  float dewPoint;
  float trend;
  float pressureHourChange;
  float temperatureHourChange;
  float humidityHourChange;
  float heatIndex;
  float temperature;
  float humidity;
  float pressure;
  // holds temporary string to output after dtostrf()
  char buffer[8];

  // if the interval has lapsed or we are just running for fisrt time
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


    // temperature from BMP sensor
    bmp.getTemperature(&temperature);
    // humidity from dht sensor
    humidity = dht.readHumidity();
    // the dht library has a handy function to calculate Heat Index
    heatIndex = dht.computeHeatIndex(temperature, humidity, false);;

    if (!event.pressure){
    Serial.println("BMP Sensor error");
    }
    if(!humidity){
      Serial.println("DHT sensor error");
    }

    // store measurements taken
    if(measurementsTaken == -1) { // special case when this is run for first time
      storeAllMeasurements(pressure, temperature, humidity);
      measurementsTaken = 0;
    } else {
      measurementsTaken++;
    }

    // store a measurement if we have reached the interval, and reset count
    if(measurementsTaken == (SECONDS_BETWEEN_STORED_MEASUREMENTS / SECONDS_BETWEEN_MEASUREMENTS)){
      storeAllMeasurements(pressure, temperature, humidity);
      measurementsTaken = 0;
    }

    // get trend of pressure
    trend = getTrend(pressureMeasurements);
    pressureHourChange = getChangePerHour(trend);
    // temperature
    trend = getTrend(temperatureMeasurements);
    temperatureHourChange = getChangePerHour(trend);
    // humidity
    trend = getTrend(humidityMeasurements);
    humidityHourChange = getChangePerHour(trend);


    /* Display values */
    Serial.print("Pressure:    ");
    Serial.print(pressure);
    Serial.println(" hPa");
        
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    Serial.print("Heat Index: ");
    dtostrf(heatIndex, 5, 1, buffer);
    Serial.print(buffer);
    Serial.println(" C");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    // dew point (condensation point)
    // formula is an approximation that works well with humidity values > 50%
    // source: https://journals.ametsoc.org/doi/pdf/10.1175/BAMS-86-2-225
    // good intro: https://inspectapedia.com/Energy/Dew_Point_Calculation.php  
    dewPoint = temperature - ((100 - humidity) / 5);
    dtostrf(dewPoint, 5, 2, buffer);
    Serial.print("Dew point: ");
    Serial.print(buffer);
    Serial.println(" C");
    
    // change of pressure per hour
    dtostrf(pressureHourChange, 5, 2, buffer);
    Serial.print("Change of pressure per hour:  ");
    if(pressureHourChange > 0) { // negative sign is always output
      Serial.print("+");
    }
    Serial.print(buffer);
    Serial.print(" hPa/hr");
    Serial.println("");

    // change of temperature per hour
    dtostrf(temperatureHourChange, 5, 2, buffer);
    Serial.print("Change of temperature per hour:  ");
    if(temperatureHourChange > 0) { // negative sign is always output
      Serial.print("+");
    }
    Serial.print(buffer);
    Serial.print(" C/hr");
    Serial.println("");

      // change of humidity per hour
    dtostrf(humidityHourChange, 5, 2, buffer);
    Serial.print("Change of humidity per hour:  ");
    if(humidityHourChange > 0) { // negative sign is always output
      Serial.print("+");
    }
    Serial.print(buffer);
    Serial.print(" %/hr");
    Serial.println("");


    /*
      For debugging, uncomment lines below to print out the stored measurements.
      They are output in CSV-compatible format so you can copy and paste into a spreadsheet 
    */
    Serial.println("Debugging, stored measurements: ");
    Serial.println("Pressure, Temperature, Humidity");
    for(int i=0; i<MAX_NUMBER_OF_STORED_MEASUREMENTS; i++){ 
      Serial.print(pressureMeasurements[i]);
      Serial.print(",");
      Serial.print(temperatureMeasurements[i]);
      Serial.print(",");
      Serial.print(humidityMeasurements[i]);
      Serial.println("");
    }
    /* end of debugging code */

    // separator between measurements taken
    Serial.println("");
    Serial.println("------------------------");
    Serial.println("");
  }
}