// Añadir librerías
#include <OneWire.h> // (!!!! DIGITAL SENSOR !!!!)
#include <DallasTemperature.h> // (!!!! DIGITAL SENSOR !!!!)

// Definir variables
#define SensorInput_GPIO 7
#define OutputPWM_GPIO 10  
#define pwmRes 12   
#define pwmMax 4095 

// Variables for Conversion
#define Uunits 100 // Set units for Control Output (u) [MAX value of (u) @ MAX pwmDutyCycle] (e.g. MAX Transistor Collector Current [mA])
//#define scaleVadc 5.0 // Set analog read voltage value 3.3 [V] for ESP32 ADC (!!!! ANALOG SENSOR !!!!)

// Execution Time Control
unsigned long pTime = 0;
unsigned long dTime = 0;
long previousMillis = 0;  // For main loop function
long Ts = 1000; // Sample time in ms
long T = 1;
long previousMillis2 = 0; // For auxiliary functions (squarewaves)
bool up = true;

// Measurement Variables
OneWire oneWire(SensorInput_GPIO); // (!!!! DIGITAL SENSOR !!!!)
DallasTemperature TempSensor(&oneWire); // (!!!! DIGITAL SENSOR !!!!)
//int tempAverages = 20; // # of samples for filtering (!!!! ANALOG SENSOR !!!!)
float tempF = 0.0; 

// Control System Variables
float Ref = 35.0; // System Reference - Temperature [°C]
float U_op = 30.0; // Direct Control Output - FOR OPENLOOP or FEEDFORWARD - Transistor Collector Current [mA]
float U_t = 0.0; // Control Output
unsigned int pwmDuty = 0; // Control Output (Converted to PWM Duty Cycle)

// Advanced Serial Input Variables
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Señal de Pruebas:
int i = 0;
float n = 0.0;
int escalones[] = {5,-5,10,-5,-10,-20,-5,-10,0};
long t_millis = 0;
int longitud = sizeof(escalones) / sizeof(escalones[0]);
long time_signal = 800000;

// Variables de reconstrucción:
float U_control = 0;
float U_pasada = 0;
float error_pasado = 0;

// Lazo Abierto:
bool calentamiento = true;

// Cambio de referencia:
const int cantidad = 300;
float datos_estabilidad[cantidad] = {0};
int indice = 0;



void controlador(void){
  // Measurement, Control, Output Command Signal, Serial Data Communication
  unsigned long currentMillis = millis(); // Update current time from the beginning
  if (calentamiento == false){
    if (currentMillis - previousMillis >= Ts) {
      previousMillis = currentMillis;
      TempSensor.requestTemperatures();  tempF = TempSensor.getTempCByIndex(0); 
      float U_t = U_op + U_control;   
      float U_tl = min(max(U_t, 0), Uunits);
      pwmDuty = int((U_tl/Uunits)*pwmMax);
      analogWriteADJ(OutputPWM_GPIO, pwmDuty);
      // Estabilidad:
      datos_estabilidad[indice] = tempF;
      indice = (indice + 1) % cantidad;
      float promedio_estabilidad = calcularPromedio();
      // Calculo de la señal de entrada u(t):
      float error = Ref - tempF;
      float kp =8;
      float ki =0.2;
      float Numerador = (error*(2*kp + ki*T)+error_pasado*(-2*kp + ki*T)+2*U_pasada);
      float Denominador = 2;
      U_control = Numerador / Denominador;
      error_pasado = error;
      U_pasada = U_control;
      // Print de la variable de temperatura, de U_t y del error:
      Serial.print("DATA,TIME,");
      Serial.print(tempF);
      Serial.print(",");
      Serial.print(U_t);
      Serial.print(",");
      Serial.print(error);
    }
  }
  // Advanced Serial Input Functions
  recvWithStartEndMarkers();  
  if (newData == true) {
    parseData();
    newData = false;
  }
}

float calcularPromedio(){
  /*
  A partir de la lista datos_estabilidad[] se calcula el promedio de los ultimas muestras.
  Retorna un float.
  */
    float suma = 0;
    for (int i = 0; i < cantidad; i++){
        suma += datos_estabilidad[i];
    }
    return suma/cantidad;
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200); // Use MAX possible to reduce serial.print() execution time (9600 19200 38400 57600 74880 115200 230400 250000 500000 1000000 2000000)
  Serial.println("LABEL,hora,lectura");
  // Set up inputs (!!!! ANALOG SENSOR !!!!)
  //  pinMode(SensorInput_GPIO, INPUT);

  // Set up inputs (!!!! DIGITAL SENSOR !!!!)
  TempSensor.begin(); // Sensor start command
  TempSensor.setResolution(12); // Sets the maximum supported resolution 12bit. (DS18B20 Accuracy 0.5°C)
  setupPWMadj();
  analogWriteADJ(OutputPWM_GPIO, pwmDuty);
  //  pinMode(OutputPWM_GPIO, OUTPUT);
  //  analogWrite(OutputPWM_GPIO, pwmDuty);
  delay(5000); // Wait 5 [s] to open Serial Monitor/Plotter
}

void calentar(void){
  /*
  Calentar la planta hasta 33°C.
  Si la variable calentamiento es true, enviara el voltaje maximo a la planta
  hasta que el error sea menor a 2°C.
  */
  if (calentamiento){
    pwmDuty = int(pwmMax);
    analogWriteADJ(OutputPWM_GPIO,pwmDuty);
    TempSensor.requestTemperatures();  tempF = TempSensor.getTempCByIndex(0);
    Serial.print("DATA,TIME,");
    Serial.println(tempF);
    Serial.print(",");
    Serial.println(U_t);
    float error = Ref - tempF;
    if (error < 2){
      calentamiento = false;
    }
  }
}

// the loop function runs over and over again forever
void loop() {
  calentar();
  controlador();
}

// Configure digital pins 9 and 10 as 12-bit PWM outputs (3905 Hz).
void setupPWMadj() {
  DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
      | _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
      | _BV(CS10);                    /* no prescaling */
  ICR1 = 0x0fff;                      /* TOP counter value - SETS RESOLUTION/FREQUENCY */
}

// 12-bit version of analogWrite(). Works only on pins 9 and 10. (MAX VAL=4095)
void analogWriteADJ(uint8_t pin, uint16_t val){
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
    }
}

//============ Advanced Serial Input Functions
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<'; // Serial input must start with this character
    char endMarker = '>'; // Serial input must end with this character
    char rc;
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}


void parseData() {      // split the data into its parts
    Ref = atof(receivedChars);     // convert serial input to a float and update System Reference value with that value
}
