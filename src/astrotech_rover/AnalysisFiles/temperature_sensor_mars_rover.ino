const int sensorPin = A0;    // Analog input pin
const float R1 = 10000.0;    // 10K resistor
const float BETA = 3950.0;   // Beta value of thermistor
const float T0 = 298.15;     // 25°C in Kelvin
const float R0 = 10000.0;    // Resistance at 25°C

void setup() {
  Serial.begin(9600);
}

void loop() {
  int raw = analogRead(sensorPin);
  float voltage = raw * 5.0 / 1023.0;
  
  // Calculate thermistor resistance
  float R = R1 * (voltage / (5.0 - voltage));
  

  // Steinhart-Hart equation simplified
  float tempK = 1.0 / ( (1.0 / T0) + (1.0 / BETA) * log(R / R0) );
  float tempC = tempK - 273.15;

  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.println(" °C");

  delay(500);
}