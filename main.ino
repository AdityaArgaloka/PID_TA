#include <MCP41_Simple.h>
MCP41_Simple MyPot;

const uint8_t  CS_PIN      = 53;
const uint8_t  motor = 12;
const uint8_t  rev = 11;
const uint8_t  lowBrake = 10;
const uint8_t  gearOne = 9;
const uint8_t  gearThree = 8;

const byte PulsesPerRevolution = 2;
const unsigned long ZeroTimeout = 100000;
const byte numReadings = 20;

volatile unsigned long LastTimeWeMeasured;
volatile unsigned long PeriodBetweenPulses = ZeroTimeout + 1000;
volatile unsigned long PeriodAverage = ZeroTimeout + 1000;
unsigned long FrequencyRaw;
unsigned long FrequencyReal;
unsigned long RPM;
unsigned int PulseCounter = 1;
unsigned long PeriodSum;
unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;
unsigned long CurrentMicros = micros();
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra;
unsigned long readings[numReadings];
unsigned long readIndex;
unsigned long total;
unsigned long average;


double proportional, derivative, integral, previous, potValue = 0;

double kp = 0.00024923,
       ki = 0.0033105,
       kd = 4.6908e-06;


const int pinADC = A4;
double sensitivitas = 66; //tegantung sensor arus yang digunakan, yang ini 5A
double nilaiadc = 00;
double teganganoffset = 2500; //nilai pembacaan offset saat tidak ada arus yang lewat
double tegangan = 00;
double nilaiarus = 00;

const int analogPin = A2; // pin arduino yang terhubung dengan pin S modul sensor tegangan
double Vmodul = 0.0;
double hasil = 0.0;
double R1 = 987.0;
double R2 = 99.0;
double value = 0;


void Current_Measurement() {
  nilaiadc = analogRead(pinADC);
  tegangan = (nilaiadc / 1023.0) * 5000;
  nilaiarus = ((tegangan - teganganoffset) / sensitivitas);
  Serial.print("\t");
  Serial.print("Arus: ");
  Serial.print(nilaiarus, 3);
}

void Voltage_Measurement() {
  value = analogRead(analogPin);
  Vmodul = (value * 5.0) / 1023.0;
  hasil = Vmodul / (R2 / (R1 + R2));
  Serial.print("\t");
  Serial.print("Tegangan: ");
  Serial.println(hasil, 3);

}

void Pulse_Event()
{
  PeriodBetweenPulses = micros() - LastTimeWeMeasured;
  LastTimeWeMeasured = micros();
  if (PulseCounter >= AmountOfReadings)
  {
    PeriodAverage = PeriodSum / AmountOfReadings;
    PulseCounter = 1;
    PeriodSum = PeriodBetweenPulses;
    int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);
    RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);
    AmountOfReadings = RemapedAmountOfReadings;
  }
  else
  {
    PulseCounter++;
    PeriodSum = PeriodSum + PeriodBetweenPulses;
  }
}
double kalrpm;
double kalman(double U) {
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 1;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P * H / (H * P * H + R);
  U_hat += + K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

unsigned long prevTime = millis();
double rpmCounter() {

  LastTimeCycleMeasure = LastTimeWeMeasured;
  CurrentMicros = micros();
  if (CurrentMicros < LastTimeCycleMeasure)
  {
    LastTimeCycleMeasure = CurrentMicros;
  }
  FrequencyRaw = 10000000000 / PeriodAverage;
  if (PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra)
  {
    FrequencyRaw = 0;
    ZeroDebouncingExtra = 2000;
  }
  else
  {
    ZeroDebouncingExtra = 0;
  }
  FrequencyReal = FrequencyRaw / 10000;
  RPM = FrequencyRaw / PulsesPerRevolution * 60;
  RPM = RPM / 10000;
  total = total - readings[readIndex];
  readings[readIndex] = RPM;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings)
  {
    readIndex = 0;
  }
  average = total / numReadings;
  kalrpm = kalman(RPM);
  //delay(100);
  return kalrpm;

}
void motorOn() {
  digitalWrite(motor, LOW);
  Serial.println("MOTOR OFF");
}
void motorOff() {
  digitalWrite(motor, HIGH);
  Serial.println("MOTOR ON");
}
void setReverse() {
  digitalWrite(rev, LOW);
  Serial.println("Motor siap mundur");
}
void setForward() {
  digitalWrite(rev, HIGH);
  Serial.println("Motor siap maju");
}
void lowBrakeOn() {
  digitalWrite(lowBrake, LOW);
  Serial.println("STOP, LOW Brake On");
}
void lowBrakeOff() {
  digitalWrite(lowBrake, HIGH);
  Serial.println("STOP, Low Brake Off");
}
void gearOneOn() {
  digitalWrite(gearOne, LOW);
  digitalWrite(gearThree, HIGH);
  Serial.println("Gear One On");
}
void gearTwoOn() {
  digitalWrite(gearOne, HIGH);
  digitalWrite(gearThree, HIGH);
  Serial.println("Gear Two On");
}
void gearThreeOn() {
  digitalWrite(gearThree, LOW);
  digitalWrite(gearOne, HIGH);
  Serial.println("Gear Three On");
}

char changeCondition;
double setpoint = 0;
void relay() {
  while (Serial.available()) {
    char  changeCondition = Serial.read();
    if (changeCondition == 'a') {
      setForward();
    }
    else if (changeCondition == 'b') {
      setReverse();
    }
    //Stop motor dikondisi maju/mundur
    else if (changeCondition == 'c') {
      lowBrakeOn();
    }
    else if (changeCondition == 'd') {
      lowBrakeOff();
    }
    else if (changeCondition == 'e') {
      motorOn();
    }
    else if (changeCondition == 'f') {
      motorOff();
    }
    else if (changeCondition == 'g') {
      gearOneOn();
    }
    else if (changeCondition == 'h') {
      gearTwoOn();
    }
    else if (changeCondition == 'i') {
      gearThreeOn();
    }
    else if (changeCondition == 'A') {
      setpoint = 1000;
    }
    else if (changeCondition == 'B') {
      setpoint = 1500;
    } else if (changeCondition == 'C') {
      setpoint = 2000;
    }
    else if (changeCondition == 'D') {
      setpoint = 2500;
    }
    else if (changeCondition == 'E') {
      setpoint = 3000;
    }
    else if (changeCondition == 'F') {
      setpoint = 3500;
    }
    else if (changeCondition == 'G') {
      setpoint = 4000;
    }
    else if (changeCondition == 'H') {
      setpoint = 4500;
    }
    else if (changeCondition == 'I') {
      setpoint = 0;
    }
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(100);
  MyPot.begin(CS_PIN);
  pinMode(rev, OUTPUT);
  pinMode(lowBrake, OUTPUT);
  pinMode(motor, OUTPUT);
  pinMode(gearOne, OUTPUT);
  pinMode(gearThree, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(3), Pulse_Event, RISING);
}

void loop() {
  unsigned long currentTime = millis();
  relay();
  if (currentTime - prevTime >= 300) {
    double input = 0;
    input = rpmCounter();
    double output = pid(input);
    MyPot.setWiper(output);
    Current_Measurement();
    Voltage_Measurement();
    prevTime = currentTime;
  }
}

double pid(double input)
{

  double error = setpoint - input;
  proportional = error;
  derivative = (error - previous) ;
  double p_value = proportional * kp;
  double i_value = integral * ki;
  double d_value = derivative * kd;
  double potValue = p_value + i_value + d_value;
  Serial.print("setpoint: ");
  Serial.print(setpoint);
  Serial.print("\t");
  Serial.print("rpmActual: ");
  Serial.print(rpmCounter());
  integral += error;
  previous = error;

  return potValue;

}
