// Define the analog pin connected to the potentiometer wiper
const int potPin = A0;  // Analog pin A0

void setup() {
  // Start serial communication to monitor the feedback
  Serial.begin(9600);
}

void loop() {
  // Read the value from the potentiometer
  int potValue = analogRead(potPin);

  // Convert the value to voltage (optional, depends on reference voltage)
  float voltage = potValue * (5.0 / 1023.0);

  // Print the raw analog value and the corresponding voltage
  Serial.print("Potentiometer Value: ");
  Serial.print(potValue);         // Raw value (0-1023)
  Serial.print(" | Voltage: ");
  Serial.println(voltage);        // Voltage equivalent

  // Delay for stability
  delay(500);
}
