#include "utils.h"

typedef enum {
  S_RR,
  S_CR,
  S_CM,
  S_FM,
  S_CL,
  S_LL,
  S_MAX,
} sensorLocation;

typedef enum {
  MD_FORWARD,
  MD_BACKWARD,
  MD_COAST,
  MD_BRAKE,
} motorMode;

typedef enum {
  M_LEFT,
  M_RIGHT
} motorSide;

typedef struct {
  uint8_t ctrl0;
  uint8_t ctrl1;
} motorInterface;

typedef struct {
  uint8_t state;
  uint8_t leftDutyCycle;
  uint8_t rightDutyCycle;
  motorMode rightMotor;
  motorMode leftMotor;
} steering_table_t;

#define UNKNOW (uint8_t)0x00
#define ON_LINE (uint8_t)(BIT(S_CM) | BIT(S_FM))
#define DEAD_END (uint8_t)(BIT(S_CM))

#define GOING_RIGHT (uint8_t) BIT(S_CL)
#define GOING_RIGHT_0 (uint8_t)(BIT(S_FM) | BIT(S_CM) | BIT(S_CL))
#define GOING_RIGHT_1 (uint8_t)(BIT(S_CM) | BIT(S_CL))
#define GOING_RIGHT_2 (uint8_t)(BIT(S_CL) | BIT(S_LL))
#define GOING_RIGHT_3 (uint8_t)(BIT(S_LL))

#define GOING_LEFT (uint8_t) BIT(S_CR)
#define GOING_LEFT_0 (uint8_t)(BIT(S_FM) | BIT(S_CM) | BIT(S_CR))
#define GOING_LEFT_1 (uint8_t)(BIT(S_CM) | BIT(S_CR))
#define GOING_LEFT_2 (uint8_t)(BIT(S_CR) | BIT(S_RR))
#define GOING_LEFT_3 (uint8_t)(BIT(S_RR))

#define SHARP_RIGHT_TURN (uint8_t)(BIT(S_CM) | BIT(S_RR))
#define GRADUAL_RIGHT_TURN (uint8_t)(BIT(S_CM) | BIT(S_CR) | BIT(S_RR))
#define GRADUAL_RIGHT_TURN_OP (uint8_t)(BIT(S_CL) | BIT(S_CM) | BIT(S_CR) | BIT(S_RR))

#define SHARP_LEFT_TURN (uint8_t)(BIT(S_CM) | BIT(S_LL))
#define GRADUAL_LEFT_TURN (uint8_t)(BIT(S_CM) | BIT(S_CL) | BIT(S_LL))
#define GRADUAL_LEFT_TURN_OP (uint8_t)(BIT(S_CR) | BIT(S_CM) | BIT(S_CL) | BIT(S_LL))

#define INTERSECTION_DEAD (uint8_t)(BIT(S_LL) | BIT(S_CL) | BIT(S_CM) | BIT(S_CR) | BIT(S_RR))
#define INTERSECTION_FORWARD (uint8_t)(BIT(S_LL) | BIT(S_CL) | BIT(S_FM) | BIT(S_CM) | BIT(S_CR) | BIT(S_RR))

//fix this
const uint8_t sensorGpio[S_MAX] = {
  [S_RR] = 19,
  [S_CR] = 21,
  [S_CM] = 20,
  [S_FM] = 18,
  [S_CL] = 3,
  [S_LL] = 2,
};

//fix this
const motorInterface motor[] = {
  [M_LEFT] = { 7, 5 },
  [M_RIGHT] = { 4, 6 }
};

const steering_table_t straightSteer[] = {
  { .state = ON_LINE, .leftDutyCycle = 150, .rightDutyCycle = 150, .rightMotor = MD_FORWARD, .leftMotor = MD_FORWARD },
  { .state = GOING_RIGHT_0, .leftDutyCycle = 0, .rightDutyCycle = 150, .rightMotor = MD_FORWARD, .leftMotor = MD_COAST },
  { .state = GOING_LEFT_0, .leftDutyCycle = 150, .rightDutyCycle = 0, .rightMotor = MD_COAST, .leftMotor = MD_FORWARD },
  { .state = GOING_RIGHT_1, .leftDutyCycle = 0, .rightDutyCycle = 150, .rightMotor = MD_FORWARD, .leftMotor = MD_BRAKE },
  { .state = GOING_LEFT_1, .leftDutyCycle = 150, .rightDutyCycle = 0, .rightMotor = MD_BRAKE, .leftMotor = MD_FORWARD },
  { .state = GOING_RIGHT, .leftDutyCycle = 0, .rightDutyCycle = 150, .rightMotor = MD_FORWARD, .leftMotor = MD_BRAKE },
  { .state = GOING_LEFT, .leftDutyCycle = 150, .rightDutyCycle = 0, .rightMotor = MD_BRAKE, .leftMotor = MD_FORWARD },
  { .state = DEAD_END,   .leftDutyCycle = 75,  .rightDutyCycle = 75, .rightMotor = MD_FORWARD, .leftMotor = MD_FORWARD },
  { .state = INTERSECTION_DEAD, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD },
  { .state = SHARP_RIGHT_TURN, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD },
  { .state = GRADUAL_RIGHT_TURN, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD },
  { .state = GRADUAL_RIGHT_TURN_OP, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD },
  { .state = SHARP_LEFT_TURN, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD },
  { .state = GRADUAL_LEFT_TURN, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD },
  { .state = GRADUAL_LEFT_TURN_OP, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD },
};

const steering_table_t steerCorner[] = {
  { .state = GRADUAL_RIGHT_TURN, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD },
  { .state = SHARP_RIGHT_TURN, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD },
  { .state = SHARP_LEFT_TURN, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD },
  { .state = GRADUAL_LEFT_TURN, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD },
  { .state = INTERSECTION_DEAD, .leftDutyCycle = 100, .rightDutyCycle = 100, .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD },
  { .state = INTERSECTION_FORWARD, .leftDutyCycle = 150, .rightDutyCycle = 150, .rightMotor = MD_FORWARD, .leftMotor = MD_FORWARD },
};

uint8_t prevBitMap;
uint8_t sensorBitMap;

static void readAllSensors(void) {
  for (uint8_t i = 0; i < S_MAX; i++) {
    (digitalRead(sensorGpio[i]) == HIGH) ? SET_BIT(sensorBitMap, i) : CLEAR_BIT(sensorBitMap, i);
  }
}

static void setMotor(motorInterface motor, motorMode mode, uint8_t dutyCycle) {
        digitalWrite(motor.ctrl1, 0);
      digitalWrite(motor.ctrl0, 0);
      return;
  switch (mode) {
    case MD_FORWARD:
      analogWrite(motor.ctrl0, dutyCycle);
      digitalWrite(motor.ctrl1, 0);
      break;

    case MD_BACKWARD:
      analogWrite(motor.ctrl1, dutyCycle);
      digitalWrite(motor.ctrl0, 0);
      break;

    case MD_COAST:
      digitalWrite(motor.ctrl1, 0);
      digitalWrite(motor.ctrl0, 0);
      break;

    case MD_BRAKE:
      digitalWrite(motor.ctrl1, 1);
      digitalWrite(motor.ctrl0, 1);
    default:
      break;
  }
}

static void stayOnLine(uint8_t state) {
  steering_table_t* steer = FIND_ENTRY(straightSteer, ARRAY_SIZE(straightSteer), state, state);

  if (steer != NULL) {  //FROM GPT
    setMotor(motor[M_LEFT], steer->leftMotor, steer->leftDutyCycle);
    setMotor(motor[M_RIGHT], steer->rightMotor, steer->rightDutyCycle);

    //Serial.print("Steer state: ");
    //Serial.println(steer->state);
  }
}

static void makeTurn(uint8_t state) {
  steering_table_t* steer = FIND_ENTRY(steerCorner, ARRAY_SIZE(steerCorner), state, state);

  if (steer != NULL) {  //FROM GPT
    setMotor(motor[M_LEFT], steer->leftMotor, steer->leftDutyCycle);
    setMotor(motor[M_RIGHT], steer->rightMotor, steer->rightDutyCycle);

    //Serial.print("Steer state: ");
    //Serial.println(steer->state);
  }
}

static bool detectTurn(uint8_t state, uint8_t prevState) {
  if ((prevState == ON_LINE) || (prevState == GOING_RIGHT_0) || (prevState == GOING_LEFT_0)) {
    if ((state == SHARP_RIGHT_TURN) || (state == GRADUAL_RIGHT_TURN) || (state == SHARP_LEFT_TURN) || (state == GRADUAL_LEFT_TURN) || (state == INTERSECTION_DEAD) || (state == DEAD_END)) {
      return true;
    }
  }
  return false;
}

void haltMotors(void) {
  for (uint8_t i = 0; i < ARRAY_SIZE(motor); i++) {
    digitalWrite(motor[i].ctrl0, HIGH);  // turn on pullup resistors
    digitalWrite(motor[i].ctrl1, HIGH);  // turn on pullup resistors
  }
}
static bool performingTurn = false;
void handleSteering(void) {
  noInterrupts();
  readAllSensors();
  //Serial.println(sensorGpio);
  /*if (performingTurn == true) {
    if ((sensorBitMap == ON_LINE) || (sensorBitMap == DEAD_END) || (sensorBitMap == GOING_RIGHT_0) || (sensorBitMap == GOING_RIGHT_1) || (sensorBitMap == GOING_LEFT_0) || (sensorBitMap == GOING_LEFT_1)) {
      performingTurn = false;
    }
  }
  if (!performingTurn) {
    if (detectTurn(sensorBitMap, prevBitMap)) {
      makeTurn(sensorBitMap);
      performingTurn = true;
      digitalWrite(8, HIGH);
    } else {
      stayOnLine(sensorBitMap);
      digitalWrite(8, LOW);
    }
  }

  prevBitMap = sensorBitMap;
  interrupts();*/
  stayOnLine(sensorBitMap);
  interrupts();
}

static void initInputs(void) {
  for (uint8_t i = 0; i < ARRAY_SIZE(sensorGpio); i++) {
    pinMode(sensorGpio[i], INPUT_PULLUP);  // Interrupt source 1
    //attachInterrupt(digitalPinToInterrupt(sensorGpio[i]), handleSteering, CHANGE);
  }
}

static void initOutputs(void) {
  for (uint8_t i = 0; i < ARRAY_SIZE(motor); i++) {
    pinMode(motor[i].ctrl0, OUTPUT);    // set pin to output
    digitalWrite(motor[i].ctrl0, LOW);  // turn on pullup resistors
    pinMode(motor[i].ctrl1, OUTPUT);    // set pin to output
    digitalWrite(motor[i].ctrl1, LOW);  // turn on pullup resistors
  }
}

void setup() {
  initInputs();
  initOutputs();
  pinMode(8, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  handleSteering();  //FROM GPT
    Serial.print(sensorBitMap);
  // put your main code here, to run repeatedly:
}
