#include "utils.h"

#define SPEED(X) (X * 1.9)

#define NO_INDICATION (-1)

typedef bool (*next_state_check)(uint8_t state);

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
  next_state_check nextStateFunc;
  uint32_t delay;
  int8_t indicator;
} steering_table_t;

#define UNKNOWN (uint8_t)0x00
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

#define SHARP_RIGHT_TURN_0 (uint8_t)(BIT(S_CM) | BIT(S_RR))
#define SHARP_RIGHT_TURN_1 (uint8_t)(BIT(S_CM) | BIT(S_RR) | BIT(S_FM))
#define SHARP_RIGHT_TURN_2 (uint8_t)(BIT(S_CM) | BIT(S_RR) | BIT(S_CL))
#define SHARP_RIGHT_TURN_3 (uint8_t)(BIT(S_CM) | BIT(S_RR) | BIT(S_CL) | BIT(S_FM))
#define SHARP_RIGHT_TURN_4 (uint8_t)(BIT(S_CL) | BIT(S_CM) | BIT(S_CR) | BIT(S_RR) | BIT(S_FM))
#define GRADUAL_RIGHT_TURN (uint8_t)(BIT(S_CM) | BIT(S_CR) | BIT(S_RR))
#define GRADUAL_RIGHT_TURN_OP (uint8_t)(BIT(S_CL) | BIT(S_CM) | BIT(S_CR) | BIT(S_RR))

#define SHARP_LEFT_TURN_0 (uint8_t)(BIT(S_CM) | BIT(S_LL))
#define SHARP_LEFT_TURN_1 (uint8_t)(BIT(S_CM) | BIT(S_LL) | BIT(S_FM))
#define SHARP_LEFT_TURN_2 (uint8_t)(BIT(S_CM) | BIT(S_LL) | BIT(S_CR))
#define SHARP_LEFT_TURN_3 (uint8_t)(BIT(S_CM) | BIT(S_LL) | BIT(S_CR) | BIT(S_FM))
#define SHARP_LEFT_TURN_4 (uint8_t)(BIT(S_CR) | BIT(S_CM) | BIT(S_CL) | BIT(S_LL) | BIT(S_FM))
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
  [M_RIGHT] = { 4, 6 },
};

uint8_t prevBitMap;
static uint8_t sensorBitMap = 0;
static next_state_check nextStateCheck;
static unsigned long timeStamp;

bool waitForStraightLineORLeft(uint8_t currentState) {
  bool ret = false;

  if ((currentState == ON_LINE) || ((millis() - timeStamp) > 1000)) {
    ret = true;
  }
  return ret;
}

bool waitForStraightLineORRight(uint8_t currentState) {
  bool ret = false;

  if ((currentState == ON_LINE) || ((millis() - timeStamp) > 1000)) {
    ret = true;
  }
  return ret;
}

bool handleRightOp(uint8_t currentState) {
  bool ret = false;

  if ((currentState == ON_LINE) || ((millis() - timeStamp) > 1000)) {
    ret = true;
  }
  return ret;
}

bool handleLeftOp(uint8_t currentState) {
  bool ret = false;

  if ((currentState == ON_LINE) || ((millis() - timeStamp) > 1000)) {
    ret = true;
  }
  return ret;
}

void setIndicatorLight(int8_t indicator) {
  digitalWrite(8, LOW); 
  digitalWrite(9, LOW); 
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  if (indicator >= 0) {
    digitalWrite(indicator, HIGH);
  }
}

const steering_table_t straightSteer[] = {
  { .state = ON_LINE,               .leftDutyCycle = SPEED(45), .rightDutyCycle = SPEED(45), .rightMotor = MD_FORWARD, .leftMotor = MD_FORWARD,  .nextStateFunc = NULL,                        .delay = 0,   .indicator = NO_INDICATION },   

  { .state = GOING_RIGHT,           .leftDutyCycle = 0,         .rightDutyCycle = SPEED(45), .rightMotor = MD_FORWARD, .leftMotor = MD_COAST,    .nextStateFunc = NULL,                        .delay = 0,   .indicator = 8 },
  { .state = GOING_LEFT,            .leftDutyCycle = SPEED(45), .rightDutyCycle = 0,         .rightMotor = MD_COAST,   .leftMotor = MD_FORWARD,  .nextStateFunc = NULL,                        .delay = 0,   .indicator = 9 },
  
  { .state = GOING_RIGHT_0,         .leftDutyCycle = 0,         .rightDutyCycle = SPEED(45), .rightMotor = MD_FORWARD, .leftMotor = MD_COAST,    .nextStateFunc = NULL,                        .delay = 0,   .indicator = 8 },
  { .state = GOING_RIGHT_1,         .leftDutyCycle = 0,         .rightDutyCycle = SPEED(45), .rightMotor = MD_FORWARD, .leftMotor = MD_BRAKE,    .nextStateFunc = NULL,                        .delay = 0,   .indicator = 8 },
  { .state = GOING_RIGHT_2,         .leftDutyCycle = 0,         .rightDutyCycle = SPEED(45), .rightMotor = MD_FORWARD, .leftMotor = MD_BRAKE,    .nextStateFunc = NULL,                        .delay = 0,   .indicator = 8 },
  { .state = GOING_RIGHT_3,         .leftDutyCycle = 0,         .rightDutyCycle = SPEED(45), .rightMotor = MD_FORWARD, .leftMotor = MD_BRAKE,    .nextStateFunc = waitForStraightLineORLeft,   .delay = 0,   .indicator = 8 },

  { .state = GOING_LEFT_0,          .leftDutyCycle = SPEED(45), .rightDutyCycle = 0,         .rightMotor = MD_COAST,   .leftMotor = MD_FORWARD,  .nextStateFunc = NULL,                        .delay = 0,   .indicator = 9 },
  { .state = GOING_LEFT_1,          .leftDutyCycle = SPEED(45), .rightDutyCycle = 0,         .rightMotor = MD_BRAKE,   .leftMotor = MD_FORWARD,  .nextStateFunc = NULL,                        .delay = 0,   .indicator = 9 },
  { .state = GOING_LEFT_2,          .leftDutyCycle = SPEED(45), .rightDutyCycle = 0,         .rightMotor = MD_BRAKE,   .leftMotor = MD_FORWARD,  .nextStateFunc = NULL,                        .delay = 0,   .indicator = 9 },
  { .state = GOING_LEFT_3,          .leftDutyCycle = SPEED(45), .rightDutyCycle = 0,         .rightMotor = MD_BRAKE,   .leftMotor = MD_FORWARD,  .nextStateFunc = waitForStraightLineORRight,  .delay = 0,   .indicator = 9 },

  { .state = DEAD_END,              .leftDutyCycle = SPEED(38), .rightDutyCycle = SPEED(38), .rightMotor = MD_FORWARD, .leftMotor = MD_FORWARD,  .nextStateFunc = NULL,                        .delay = 900, .indicator = NO_INDICATION },
  { .state = INTERSECTION_DEAD,     .leftDutyCycle = SPEED(0),  .rightDutyCycle = SPEED(35), .rightMotor = MD_BRAKE,   .leftMotor = MD_FORWARD,  .nextStateFunc = waitForStraightLineORLeft,   .delay = 900, .indicator = NO_INDICATION },

  { .state = SHARP_RIGHT_TURN_0,    .leftDutyCycle = SPEED(35), .rightDutyCycle = SPEED(10), .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD, .nextStateFunc = waitForStraightLineORRight,  .delay = 900, .indicator = 10 },
 // { .state = SHARP_RIGHT_TURN_1,    .leftDutyCycle = SPEED(35), .rightDutyCycle = SPEED(10), .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD, .nextStateFunc = waitForStraightLineORRight,  .delay = 900, .indicator = 10 },
  { .state = SHARP_RIGHT_TURN_2,    .leftDutyCycle = SPEED(35), .rightDutyCycle = SPEED(10), .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD, .nextStateFunc = waitForStraightLineORRight,  .delay = 900, .indicator = 10 },
 // { .state = SHARP_RIGHT_TURN_3,    .leftDutyCycle = SPEED(35), .rightDutyCycle = SPEED(10), .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD, .nextStateFunc = waitForStraightLineORRight,  .delay = 900, .indicator = 10 },
 // { .state = SHARP_RIGHT_TURN_4,    .leftDutyCycle = SPEED(35), .rightDutyCycle = SPEED(10), .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD, .nextStateFunc = waitForStraightLineORRight,  .delay = 900, .indicator = 10 },
  { .state = GRADUAL_RIGHT_TURN,    .leftDutyCycle = SPEED(35), .rightDutyCycle = SPEED(10), .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD, .nextStateFunc = waitForStraightLineORRight,  .delay = 900, .indicator = 10 },
  { .state = GRADUAL_RIGHT_TURN_OP, .leftDutyCycle = SPEED(40), .rightDutyCycle = SPEED(10), .rightMotor = MD_BACKWARD, .leftMotor = MD_FORWARD, .nextStateFunc = handleRightOp,               .delay = 995, .indicator = 10 },

  { .state = SHARP_LEFT_TURN_0,     .leftDutyCycle = SPEED(10), .rightDutyCycle = SPEED(35), .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD, .nextStateFunc = waitForStraightLineORLeft,   .delay = 900, .indicator = 11 },
 // { .state = SHARP_LEFT_TURN_1,     .leftDutyCycle = SPEED(10), .rightDutyCycle = SPEED(35), .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD, .nextStateFunc = waitForStraightLineORLeft,   .delay = 900, .indicator = 11 },
  { .state = SHARP_LEFT_TURN_2,     .leftDutyCycle = SPEED(10), .rightDutyCycle = SPEED(35), .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD, .nextStateFunc = waitForStraightLineORLeft,   .delay = 900, .indicator = 11 },
 // { .state = SHARP_LEFT_TURN_3,     .leftDutyCycle = SPEED(10), .rightDutyCycle = SPEED(35), .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD, .nextStateFunc = waitForStraightLineORLeft,   .delay = 900, .indicator = 11 },
 // { .state = SHARP_LEFT_TURN_4,     .leftDutyCycle = SPEED(10), .rightDutyCycle = SPEED(35), .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD, .nextStateFunc = waitForStraightLineORLeft,   .delay = 900, .indicator = 11 },
  { .state = GRADUAL_LEFT_TURN,     .leftDutyCycle = SPEED(10), .rightDutyCycle = SPEED(35), .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD, .nextStateFunc = waitForStraightLineORLeft,   .delay = 900, .indicator = 11 },
  { .state = GRADUAL_LEFT_TURN_OP,  .leftDutyCycle = SPEED(10), .rightDutyCycle = SPEED(40), .rightMotor = MD_FORWARD, .leftMotor = MD_BACKWARD, .nextStateFunc = handleLeftOp,                .delay = 995, .indicator = 11 },
};

static void readAllSensors(void) {
  for (uint8_t i = 0; i < S_MAX; i++) {
    (digitalRead(sensorGpio[i]) == LOW) ? SET_BIT(sensorBitMap, i) : CLEAR_BIT(sensorBitMap, i);
  }
}

static void setMotor(motorInterface motor, motorMode mode, uint8_t dutyCycle) {

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

static bool stayOnLine(uint8_t state) {
  // additional condition for sharp turns
  if (nextStateCheck != NULL) {
    if (!nextStateCheck(state)) {
      return true;
    }
  }

  steering_table_t* steer = FIND_ENTRY(straightSteer, ARRAY_SIZE(straightSteer), state, state);
  
  // make sure addition condition func pointer is null before next check, this is non-null if additional condition check is valid
  steer->nextStateFunc = NULL;

  if (steer != NULL) {  //FROM GPT
    // set motor pwm and direction
    setMotor(motor[M_LEFT], steer->leftMotor, steer->leftDutyCycle);
    setMotor(motor[M_RIGHT], steer->rightMotor, steer->rightDutyCycle);
    
    timeStamp = millis();
    
    nextStateCheck = steer->nextStateFunc;
    setIndicatorLight(steer->indicator);
    
    if (steer->delay != 0) {
      delay(steer->delay);
    }
    if (steer->state != ON_LINE && steer->state != DEAD_END) {
      prevBitMap = state;
    }
    return true;
  }
  return false;
}

void haltMotors(void) {
  for (uint8_t i = 0; i < ARRAY_SIZE(motor); i++) {
    digitalWrite(motor[i].ctrl0, HIGH);  // turn on pullup resistors
    digitalWrite(motor[i].ctrl1, HIGH);  // turn on pullup resistors
  }
}

void handleSteering(void) {
  noInterrupts();
  readAllSensors();
  if (!stayOnLine(sensorBitMap)) {
    stayOnLine(prevBitMap);
  }
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
  pinMode(8, OUTPUT);     // set pin to output
  pinMode(9, OUTPUT);     // set pin to output
  pinMode(10, OUTPUT);    // set pin to output
  pinMode(11, OUTPUT);    // set pin to output
  digitalWrite(8, LOW);   // turn on pullup resistors
  digitalWrite(9, LOW);   // turn on pullup resistors
  digitalWrite(10, LOW);  // turn on pullup resistors
  digitalWrite(11, LOW);  // turn on pullup resistors
}

void setup() {
  initInputs();
  initOutputs();
  Serial.begin(9600);
}

void loop() {
  handleSteering();  //FROM GPT
  //Serial.println(sensorBitMap, HEX);
  // put your main code here, to run repeatedly:
}