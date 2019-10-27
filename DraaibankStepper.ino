#include <AccelStepper.h>
#include <MultiStepper.h>
#include <EEPROM.h>

#define BACKSPACE 8

#define STEPPER_STEP_PIN 2
#define STEPPER_DIR_PIN 3

#define ENABLE 8

#define MICROMODE_1 5
#define MICROMODE_2 6
#define MICROMODE_3 7

#define MAXSPEED_ADDR 0               // int
#define ACCELERATION_ADDR 2           // int
#define STEPS_PER_REVOLUTION_ADDR 4   // long

// MACRO : Combines FROM_BYTES into TO_OBJECT of datatype "DATATYPE"
#define combineBytes(FROM_BYTES, DATATYPE, TO_OBJECT) TO_OBJECT = *((DATATYPE *)FROM_BYTES);

// MACRO : Splits FROM_DATA of DATATYPE into equivalent byte array TO_BYTES
#define splitBytes(FROM_DATA, DATATYPE, TO_BYTES) *((DATATYPE *)TO_BYTES) = FROM_DATA;

AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

void setup() {
  // Initialize Serial @ 9600 baud
  Serial.begin(9600);

  pinMode(ENABLE, OUTPUT);

  // Set microstelling mode
  pinMode(MICROMODE_1, OUTPUT);
  pinMode(MICROMODE_2, OUTPUT);
  pinMode(MICROMODE_3, OUTPUT);
  digitalWrite(MICROMODE_1, LOW);
  digitalWrite(MICROMODE_2, LOW);
  digitalWrite(MICROMODE_3, LOW);

  initialize();
  menu();
  while(Serial.available()){
    Serial.read();
  }
}

void loop() {
  while(Serial.available()){
    char user_input = Serial.read();
    Serial.println();
    switch(user_input) {
      case '1': {
        Serial.print("Enter the maxSpeed: ");
        String maxSpeed = readString();
        if(maxSpeed.length() > 0) {
          EEPromWriteInt(MAXSPEED_ADDR, maxSpeed.toInt());
        }
        initialize();
        } break;
      case '2': {
        Serial.print("Enter the acceleration: ");
        String acceleration = readString();
        if(acceleration.length() > 0) {
          EEPromWriteInt(ACCELERATION_ADDR, acceleration.toInt());
        }
        initialize();
      } break;
      case '3': {
        Serial.print("Enter the number of steps per revolution: ");
        String stepsPerRevolution = readString();
        if(stepsPerRevolution.length() > 0) {
          EEPromWriteLong(STEPS_PER_REVOLUTION_ADDR, atol(stepsPerRevolution.c_str()));
        }
      } break;
      case '4': {
        Serial.print("Enter the number of teeth: ");
        String teeth = readString();
        float stepsPerRevolution = (float)EEPromReadLong(STEPS_PER_REVOLUTION_ADDR);
        float stepsPerTooth = stepsPerRevolution / teeth.toFloat();
        Serial.print("Number of steps per tooth: ");
        Serial.println(stepsPerTooth);

        // Fill the array with all tooth positions
        long toothPositions[teeth.toInt()+1];
        for(int i=0; i<teeth.toInt(); i++) {
          toothPositions[i] = stepsPerTooth * i;
        }
        toothPositions[teeth.toInt()] = stepsPerRevolution;
        
        Serial.println("  Press [space] for next position, 'b' for previous position, 'q' to quit");
        stepper.setCurrentPosition(0);
        Serial.println("    Current position: 0");
        int step = 0;
        digitalWrite(ENABLE, LOW); // Enable motor
        while(1) {
          while(Serial.available()) {
            char command = Serial.read();
            switch(command) {
              case ' ': {
                step++;
                long newPosition = toothPositions[step];
                stepper.runToNewPosition(newPosition);
                Serial.print("    Current position: ");
                Serial.print(step);
                Serial.print(", ");
                Serial.println(newPosition);
                if(step == teeth.toInt()) {
                  step = 0;
                  stepper.setCurrentPosition(0);
                }
                } break;
              case 'b': {
                step--;
                if(step == -1) {
                  stepper.setCurrentPosition(stepsPerRevolution);
                  step = teeth.toInt() - 1;
                }
                long newPosition = toothPositions[step];
                stepper.runToNewPosition(newPosition);
                Serial.print("    Current position: ");
                Serial.print(step);
                Serial.print(", ");
                Serial.println(newPosition);
               } break;
              case 'q':
                menu();
                return;
                break;
            }
          }
        }
      } break;
      case '5': {
        Serial.print("Enter the number of degrees per step: ");
        int degreesPerStep = readString().toInt();
        float stepsPerRevolution = (float)EEPromReadLong(STEPS_PER_REVOLUTION_ADDR);
        float stepsPerPosition = stepsPerRevolution / 360.0 * (float)degreesPerStep;
        Serial.print("Number of steps per position change: ");
        Serial.println((long)stepsPerPosition);

        Serial.println("  Press [space] for next position, 'b' for previous position, 'q' to quit");
        int currentAngle = 0;
        stepper.setCurrentPosition(0);
        Serial.println("    Current angle: 0");
        digitalWrite(ENABLE, LOW); // Enable motor
        while(1) {
          while(Serial.available()) {
            char command = Serial.read();
            switch(command) {
              case ' ': {
                stepper.setCurrentPosition(0);
                stepper.runToNewPosition(stepsPerPosition);
                currentAngle += degreesPerStep;
                if(currentAngle >= 360) currentAngle -= 360;
                Serial.print("    Current angle: ");
                Serial.println(currentAngle);
                } break;
              case 'b': {
                stepper.setCurrentPosition(stepsPerPosition);
                stepper.runToNewPosition(0);
                currentAngle -= degreesPerStep;
                if(currentAngle < 0) currentAngle += 360;
                Serial.print("    Current angle: ");
                Serial.println(currentAngle);
               } break;
              case 'q':
                menu();
                return;
                break;
            }
          }
        }
      } break;
    }
    
    menu();
  }
  //stepper.runSpeed();
  //stepper.runToNewPosition(4000);
  //stepper.runToNewPosition(0);  
}

void initialize() {
  Serial.println();
  Serial.println("Initializing...");
  int maxSpeed = EEPromReadInt(MAXSPEED_ADDR);
  Serial.print("   Setting maxSpeed to ");
  Serial.println(maxSpeed);
  stepper.setMaxSpeed(maxSpeed);
  
  int acceleration = EEPromReadInt(ACCELERATION_ADDR);
  Serial.print("   Setting acceleration to ");
  Serial.println(acceleration);
  stepper.setAcceleration(acceleration);
}

void menu() {
  digitalWrite(ENABLE, HIGH); // Disable motor

  int maxSpeed = EEPromReadInt(MAXSPEED_ADDR);
  int acceleration = EEPromReadInt(ACCELERATION_ADDR);
  long stepsPerRevolution = EEPromReadLong(STEPS_PER_REVOLUTION_ADDR);
  Serial.println();
  Serial.println("Menu:");
  Serial.print("  1. Set maximum speed: ");
  Serial.println(maxSpeed);
  Serial.print("  2. Set acceleration: ");
  Serial.println(acceleration);
  Serial.print("  3. Set number of steps per revolution: ");
  Serial.println(stepsPerRevolution);
  Serial.println("  4. Start a number of teeth");
  Serial.println("  5. Start a number of degrees per step");
  Serial.print("Choice: ");
}

String readString() {
  String input;
  while(1) {
    while(Serial.available()) {
      char received = Serial.read();
      switch(received) {
        case '\r':
        case '\n':
          Serial.println();
          return input;
        case BACKSPACE:
          if(input.length() > 0) {
            Serial.write(8);
            Serial.write(' ');
            Serial.write(8);
            input.remove(input.length()-1);
          }
          break;
        default:
          Serial.write(received);
          input += received;
      }
    }
  }
}

// Writes Integer's equivalent bytes to address and address + 1
void EEPromWriteInt(unsigned int address, int data)
{
  const byte sizeOfdataType = sizeof(int);
  byte tempBuffer[sizeOfdataType];
  splitBytes(data, int, tempBuffer);
  for(byte index = 0; index < sizeOfdataType; index++) {
    EEPROM.write(address + index, tempBuffer[index]);
  }
}

// Reads bytes from address and address + 1 and returns equivalent int
int EEPromReadInt(unsigned int address)
{
  const byte sizeOfdataType = sizeof(int);
  byte tempBuffer[sizeOfdataType];
  int result;
  for(byte index = 0; index < sizeOfdataType; index++) {
    tempBuffer[index] = EEPROM.read(address + index);
  }
  combineBytes(tempBuffer, int, result);

  return result;
}

// Writes long's equivalent bytes to address, address + 1, address + 2 and address + 3
void EEPromWriteLong(unsigned int address, long data)
{
  const byte sizeOfdataType = sizeof(long);
  byte tempBuffer[sizeOfdataType];
  splitBytes(data, long, tempBuffer);
  for(byte index = 0; index < sizeOfdataType; index++) {
    EEPROM.write(address + index, tempBuffer[index]);
  }
}

// Reads bytes from address, address + 1, address + 2 and address + 3 and returns equivalent long
long EEPromReadLong(unsigned int address)
{
  const byte sizeOfdataType = sizeof(long);
  byte tempBuffer[sizeOfdataType];
  long result;
  for(byte index = 0; index < sizeOfdataType; index++) {
    tempBuffer[index] = EEPROM.read(address + index);
  }
  combineBytes(tempBuffer, long, result);

  return result;
}

// Writes Double's equivalent bytes to address, address + 1, address + 2 and address + 3
void EEPromWriteDouble(unsigned int address, double data)
{
  const byte sizeOfdataType = sizeof(double);
  byte tempBuffer[sizeOfdataType];
  splitBytes(data, double, tempBuffer);
  for(byte index = 0; index < sizeOfdataType; index++) {
    EEPROM.write(address + index, tempBuffer[index]);
  }
}

// Reads bytes from address, address + 1, address + 2 and address + 3 and returns equivalent double
double EEPromReadDouble(unsigned int address)
{
  const byte sizeOfdataType = sizeof(double);
  byte tempBuffer[sizeOfdataType];
  double result;
  for(byte index = 0; index < sizeOfdataType; index++) {
    tempBuffer[index] = EEPROM.read(address + index);
  }
  combineBytes(tempBuffer, double, result);

  return result;
}
