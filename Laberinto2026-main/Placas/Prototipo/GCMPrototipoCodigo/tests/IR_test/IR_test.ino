/*codigo para el sensor infrarojo 
pero hay que ajustar el umbral segun el material que se use en la pista del laberinto
este codigo es para testear y ver los valores del umbral para poder calibrr los sensores*/

#define SENSOR_PIN 9  // Cambiá según tu conexión

int valor = 0;
int umbral = 2000; // después lo ajustamos

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Promedio de lecturas (más estable)
  int suma = 0;
  for (int i = 0; i < 10; i++) {
    suma += analogRead(SENSOR_PIN);
    delay(2);
  }
  valor = suma / 10;

  Serial.print("Valor: ");
  Serial.print(valor);

  // Clasificación
  if (valor > umbral) {
    Serial.println(" → BLANCO");
  } else {
    Serial.println(" → NEGRO");
  }

  delay(200);
}