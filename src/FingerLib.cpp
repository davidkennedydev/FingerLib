/*
 * FingerLib.cpp
 *
 * Created: 02/11/2015 11:12:48
 * Author: Olly McBride
 *
 * This work is licensed under the Creative Commons Attribution 4.0
 * International License. To view a copy of this license, visit
 * http://creativecommons.org/licenses/by/4.0/.
 *
 * Modified by David Kennedy S. Araujo (software@davidkennedy.dev) to clean code
 *
 */

#include "FingerLib.h"

static uint8_t _TotalFingerCount = 0; // the total number of finger instances
static uint8_t _TotalAttachedFingers =
    0; // the total number of attached/configured fingers

static Finger *fingerISRList[MAX_FINGERS] = {
    nullptr}; // pointer to an instance of the Finger class
static bool _posCtrlTimerInit =
    false; // flag to prevent multiple timer initialisations

constexpr bool isTooManyFingersInitialized(uint8_t index) {
  return index >= MAX_FINGERS;
}

////////////////////////////// Constructor/Destructor
/////////////////////////////////
Finger::Finger() {
  if (isTooManyFingersInitialized(_fingerIndex)) {
    // TODO: Negative values shouldn't be used with unsiged types
    _fingerIndex = -1; // set current finger number to be empty
    _isActive = false; // set current finger as inactive
  } else {
    _fingerIndex =
        _TotalFingerCount++; // count the total number of fingers initialised
  }

  motorEnable(false);

  _pos = {0};
  _dir = {0};
  _speed = {0};
  _PWM = {0};
#ifdef FORCE_SENSE
  _force = {0};
#endif

  _interruptEn = true; // use the timer interrupt by default
}

////////////////////////////// Public Methods //////////////////////////////

// INITIALISATION

// attach pins to a finger using only position control
uint8_t Finger::attach(uint8_t dir0, uint8_t dir1, uint8_t posSns) {
  return attach(dir0, dir1, posSns, (uint8_t)-1,
                0); // no force sense, do not invert
}

// attach pins to a finger using only position control, but allow the direction
// to be inverted
uint8_t Finger::attach(uint8_t dir0, uint8_t dir1, uint8_t posSns, bool inv) {
  return attach(dir0, dir1, posSns, (uint8_t)-1, inv); // no force sense
}

// attach pins to a finger using position control and force control, and allow
// the direction to be inverted
uint8_t Finger::attach(uint8_t dir0, uint8_t dir1, uint8_t posSns,
                       uint8_t forceSns, bool inv) {
  if (isTooManyFingersInitialized(_fingerIndex)) {
    _isActive = false; // set the current finger to be inactive
    // TODO: Negative values shouldn't be used with unsiged types
    return (uint8_t)(-1); // return BLANK
  }

  motorEnable(false); // disable the motor

  // attach all finger pins
  _pin.dir[0] = dir0;
  _pin.dir[1] = dir1;
  _pin.posSns = posSns;
#ifdef FORCE_SENSE
  _pin.forceSns = forceSns;
#endif
  _invert = inv; // store whether to invert the finger direction

  // configure pins
  pinMode(dir0, OUTPUT);  // set direction1 pin to output
  pinMode(dir1, OUTPUT);  // set direction2 pin to output
  pinMode(posSns, INPUT); // set position sense pin to input
#ifdef FORCE_SENSE
  pinMode(forceSns, INPUT); // set force sense pin to input
#endif

  // initialise circle buffer
  _velBuff.begin(VEL_BUFF_SIZE);
#if defined(FORCE_SENSE)
  _IBuff.begin(CURR_SENSE_BUFF_SIZE);
#endif

  // set limits and initial values
  setPosLimits(MIN_FINGER_POS, MAX_FINGER_POS);
  setPWMLimits(MIN_FINGER_PWM, MAX_FINGER_PWM);
  writeDir(OPEN);
  writeSpeed(MAX_FINGER_PWM);
#ifdef FORCE_SENSE
  forceSenseEnable(true);
#endif

  // if finger is being attached for the first time
  if (!_isActive) {
    _TotalAttachedFingers++; // update the count of total attached fingers
  }

  // add a pointer to the current finger instance to the list for position
  // control calls, used by the ISR
  fingerISRList[_fingerIndex] = this;

  // initialise the position control timer
  if (!_posCtrlTimerInit) {
    if (_interruptEn) // if the interrupt is enabled for motor control
    {
      _passMotorPtr(&_fingerControlCallback); // attach the finger control
                                              // function to the timer
    }

    _posCtrlTimerSetup(); // initialise and start the timer
    _posCtrlTimerInit = true;
  }

#ifdef FORCE_SENSE
  // attach current sense control interrupt, initialise and sync PWM timers
  initCurrentSense(_pin.dir[0], _pin.dir[1], &_currentSenseCallback);
#endif

#ifdef ARDUINO_AVR_MEGA2560
  // change the PWM timer frequency to be out of the audible range
  constexpr auto max_frequency = 0x01; // > 20kHz to prevent humming
  setPWMFreq(dir0, max_frequency);
  setPWMFreq(dir1, max_frequency);
#endif

  motorEnable(true);   // re-enable the motor
  _isActive = true;    // set the current finger to be active
  return _fingerIndex; // return the current finger number
}

// deactivate the finger
void Finger::detach() {
  _isActive = false;
  _TotalAttachedFingers--;
}

// return true if the current finger is attached and initialised correctly
bool Finger::attached() { return _isActive; }

// invert the current finger direction
void Finger::invertFingerDir() { _invert = !_invert; }

// LIMITS

// set the maximum and minimum position limits
void Finger::setPosLimits(int min, int max) {
  // set limits
  _pos.limit.min = min;
  _pos.limit.max = max;
}

// set the maximum and minimum speed limits
void Finger::setPWMLimits(int min, int max) {
  // set limits
  _PWM.limit.min = min;
  _PWM.limit.max = max;

#if defined(USE_PID)
  _PID.setLimits(-(double)_PWM.limit.max, _PWM.limit.max);
#endif
}

#ifdef FORCE_SENSE
// set the maximum and minimum force limits
void Finger::setForceLimits(int min, int max) {
  // set limits
  _force.limit.min = convertForceToADC(
      min); // force values converted to ADC values for quicker maths
  _force.limit.max = convertForceToADC(
      max); // force values converted to ADC values for quicker maths
}
#endif

// write a target position to the finger
void Finger::writePos(int value) {
  // TODO check if value should really be signed, it is being casted to a
  // unsiged type on all usages
  const uint16_t position = value;

  _pos.targ = constrain(position, _pos.limit.min, _pos.limit.max);

  // calculate new position error (to remove false positives in reachedPos() )
  _pos.error = (_pos.targ - _pos.curr);

  // determine direction of travel
  _dir.targ = (position > _pos.curr) ? CLOSE : OPEN;
}

// write a change in position to the finger
void Finger::movePos(int value) {
  // change position
  _pos.targ += readPos() + value;

  // constrain position value to limits
  _pos.targ = constrain((uint16_t)_pos.targ, _pos.limit.min, _pos.limit.max);

  // calculate new position error (to remove false positives in reachedPos() )
  _pos.error = (_pos.targ - _pos.curr);
}

// return the current position
int16_t Finger::readPos() {
  // read finger position
  noInterrupts();
  _pos.curr = analogRead(_pin.posSns);

  // invert finger direction if enabled
  if (_invert) {
    _pos.curr = MAX_POS_SENSOR_VAL - _pos.curr;
  }
  interrupts();

  return _pos.curr;
}

// return the error between the current position and the target position
int16_t Finger::readPosError() { return _pos.error; }

// return the target position
uint16_t Finger::readTargetPos() { return _pos.targ; }

// returns true if position reached within a tolerance
bool Finger::reachedPos(uint16_t posErr) {
  return abs(readPosError()) < posErr;
}

// returns true if position reached
bool Finger::reachedPos() { return reachedPos(POS_REACHED_TOLERANCE); }

// DIR

// write a target direction to the finger
void Finger::writeDir(int value) {
  // store direction
  _dir.targ = value;

  // set new target position based on input direction
  if (_dir.targ == OPEN) {
    _pos.targ = _pos.limit.min;
  } else if (_dir.targ == CLOSE) {
    _pos.targ = _pos.limit.max;
  }
}

// return the current direction
uint8_t Finger::readDir() { return _dir.targ; }

// open the finger
void Finger::open() { writePos(_pos.limit.min); }

// close the finger
void Finger::close() { writePos(_pos.limit.max); }

// toggle finger between open/closed
void Finger::open_close() { open_close(!readDir()); }

// set finger to open/close
void Finger::open_close(boolean dir) {

  if (dir == OPEN) {
    open();
  } else if (dir == CLOSE) {
    close();
  }
}

// SPEED

// write a target speed to the finger
void Finger::writeSpeed(int value) {
  // TODO check if value should really be signed, it is being casted to a
  // unsiged type on all usages
  _speed.targ = constrain((int16_t)value, -_PWM.limit.max,
                          _PWM.limit.max); // store vectorised speed
  _PWM.targ = _speed.targ;

#if defined(USE_PID)
  _PID.setLimits(-abs(_PWM.targ), abs(_PWM.targ));
#endif
}

// return the current movement speed
float Finger::readSpeed() {
  calcVel(); // read finger speed (ADC/per)
  _velBuff.write(_speed.raw);
  _speed.prev = _speed.curr;
  _speed.curr = _velBuff.readMean();

  return _speed.raw;
}

// return the target movement speed
float Finger::readTargetSpeed() { return _speed.targ; }

// return the current speed being written to the finger
int Finger::readPWM() { return _PWM.curr; }

// return the target speed
int Finger::readTargetPWM() { return _PWM.targ; }

#ifdef FORCE_SENSE
// FORCE

// return the current force value. If force sense is disabled, return blank (-1)
float Finger::readForce(void) {
  // if force sense is not enabled, return
  if (!_forceSnsEn) {
    return (-1);
  } else {
    return convertADCToForce(readCurrent());
  }
}

// return the latest force sense ADC value
uint16_t Finger::readCurrent(void) {
  // if force sense is not enabled, return
  if (!_forceSnsEn) {
    return 0;
  } else {
    return _IBuff.readMean();
  }
}

// return true if the force limit has been reached
bool Finger::reachedForceLimit(void) { return _force.limit.reached; }

// read the current force and discount the current spike
void Finger::calcCurrentSns(void) {
  // if still within current spike period
  if (_currentSpikeTimer.started() && !_currentSpikeTimer.finished()) {
    _force.curr = 0;
  } else {
    noInterrupts();
    _force.curr = analogRead(_pin.forceSns);
    interrupts();
  }

  // store the read current into a buffer for smoothing
  _IBuff.write((uint16_t)_force.curr);
}

// convert ADC current sense value to force value (float)
float Finger::convertADCToForce(int ADCVal) {
  const float m = CURRENT_SENSE_CONST_M; // generated from the equation of the
                                         // line from the force-current graph
  const float c = CURRENT_SENSE_CONST_C; // generated from the equation of the
                                         // line from the force-current graph
  const float k =
      ADC_VALS_PER_MA_DRAW; // number of ADC values per mA draw of the motor

  float force = (((float)ADCVal / k) - c) /
                m; // convert ADC value to current draw to output force
  force = constrain(force, 0,
                    1000); // constrain the force value to reasonable limits

  return force;
}

// convert force value to ADC current sense value (int)
int Finger::convertForceToADC(float force) {
  const float m = CURRENT_SENSE_CONST_M; // generated from the equation of the
                                         // line from the force-current graph
  const float c = CURRENT_SENSE_CONST_C; // generated from the equation of the
                                         // line from the force-current graph
  const float k =
      ADC_VALS_PER_MA_DRAW; // number of ADC values per mA draw of the motor

  int ADCvals = (int)(((m * force) + c) *
                      k); // convert the force to current draw to ADC values
  ADCvals = constrain(ADCvals, 0,
                      1024); // constrain the ADC value to 10-bit ADC limit

  return ADCvals;
}

#endif // FORCE_SENSE

// STOP/START

// stop the motor and hold position
void Finger::stopMotor() {
  // set target position to current position
  writePos(readPos());
}

// ENABLE/DISABLE

// set motor to be enabled/disabled
void Finger::motorEnable(bool en) {
  _motorEn = en;

  if (_motorEn) {
    _PID.reset();
  }
}

// return true if the motor is enabled
bool Finger::enabled() { return _motorEn; }

#ifdef FORCE_SENSE
// set force sensing to be enabled/disabled
void Finger::forceSenseEnable(bool en) { _forceSnsEn = en; }
#endif

// enable timer interrupt for motor control
void Finger::enableInterrupt() {
  _passMotorPtr(&_fingerControlCallback); // attach the finger control function
                                          // to the timer

  _interruptEn = true;
}

// disable timer interrupt for motor control
void Finger::disableInterrupt() {
  _passMotorPtr(
      nullptr); // prevent the interrupt from calling the motor control function

  _interruptEn = false;
}

void Finger::control() {
  // read finger position (reads to _pos.curr internally() )
  readPos();

  // read finger speed (including calculating the vel)
  readSpeed();

#ifdef FORCE_SENSE
  if (_forceSnsEn && _motorEn) {
    stallDetection();
  }
#endif

  positionController(); // run the position controller
}

////////////////////////////// Private Methods //////////////////////////////

#if defined(USE_PID)
// position controller (using PID)
void Finger::positionController() {
  // run pos PID controller to calculate target speed
  _speed.targ = _PID.run(_pos.targ, _pos.curr);

  motorControl(_speed.targ);
}

#else
// position controller (using custom P controller)
// controls motor PWM values based on current and target position using a
// proportional controller (triggered by interrupt) total duration = 439us,
// therefore max freq = 2kHz. We use 200Hz (5ms), where 0.5ms = motor
// control, 4.5ms = program runtime
void Finger::positionController(void) {
  signed int motorSpeed =
      0;   // used to calculate the motor speed as a vector (±255)
  float m; // the proportional gradient
  signed int vectorise = 1; // changes the sign '±' of the value

#if defined(ARDUINO_AVR_MEGA2560)
  int proportionalOffset = 300;
  signed int motorStopOffset = 25;
#elif defined(ARDUINO_ARCH_SAMD)
  int proportionalOffset = 300;
  signed int motorStopOffset = 20;
#endif

  // calc positional error
  _pos.error = (signed int)(_pos.targ - _pos.curr);

  // speed/position line gradient
  m = (float)(((float)_PWM.targ) / ((float)proportionalOffset));

  // change the ± sign on the motorSpeed depending on required direction
  if (_pos.error >= 0)
    vectorise = -1;

  // constrain speed to posError/speed graph
  if (abs(_pos.error) < motorStopOffset) // motor dead zone
  {
    motorSpeed = 0;
  } else if (_pos.error >
             (signed int)(proportionalOffset +
                          motorStopOffset)) // set to max speed depending on
                                            // direction
  {
    motorSpeed = _PWM.targ;
  } else if (_pos.error <
             -(signed int)(proportionalOffset +
                           motorStopOffset)) // set to -max speed depending on
                                             // direction
  {
    motorSpeed = -_PWM.targ;
  } else if (abs(_pos.error) <=
             (proportionalOffset + motorStopOffset)) // proportional control
  {
    motorSpeed = (m * (_pos.error + (motorStopOffset * vectorise))) -
                 (_PWM.limit.min * vectorise);
  }

  // constrain speed to limits
  motorSpeed = constrain(motorSpeed, -((signed int)_PWM.limit.max),
                         (signed int)_PWM.limit.max);

  // if motor disabled, set speed to 0
  if (!_motorEn)
    motorSpeed = 0;

  // send speed to motors
  motorControl(motorSpeed); // 15us
}
#endif

// split the vectorised motor speed into direction and speed values and write to
// the motor
void Finger::motorControl(signed int speed) {
  bool dir = OPEN;

  // if the motor is disabled, set the speed to 0
  if (!_motorEn) {
    speed = 0;
  }

  // determine direction and invert speed if necessary
  if (speed > 0) {
    dir = OPEN;
  } else if (speed < 0) {
    dir = CLOSE;
    speed = -speed;
  }

  // if speed is not within min/max, set to 0
  speed = window(speed, 0, _PWM.limit.max);

  // store previous and current speed
  _PWM.prev = _PWM.curr;
  _PWM.curr = (uint8_t)speed;

  // store previous and current direction
  _dir.prev = _dir.curr;
  _dir.curr = dir;

#ifdef FORCE_SENSE
  // if motor has just started a movement or direction has changed or the PWM
  // speed has changed by > 1/3 of the max
  if (((_PWM.prev == 0) && (_PWM.curr > 0)) || (_dir.curr != _dir.prev) ||
      ((_PWM.curr - _PWM.prev) > (MAX_FINGER_PWM / 3))) {
    // start current spike timer
    _currentSpikeTimer.start(CURR_SPIKE_DUR_US);
  }
#endif

  // 	invert finger direction if enabled
  if (_invert) {
    dir = !dir;
  }

  // write the speed to the motors
  analogWrite(_pin.dir[dir], speed); // write motor speed to one direction pin
  analogWrite(_pin.dir[!dir], 0);    // write 0 to other direction pin
}

#ifdef FORCE_SENSE

bool Finger::stallDetection(void) {
  // if the motor is stopped and drawing a lot of current, set the targ pos to
  // be the curr pos
  if ((abs(_speed.curr) <= 1) && (_IBuff.readMean() >= STALL_CURRENT_THRESH)) {
    if (!_motorStallTimer.started()) {
      _motorStallTimer.start(MAX_STALL_TIME_MS);
    } else if (_motorStallTimer.finished()) // if the motor has been stalled for
                                            // the MAX_STALL_TIME_MS
    {
      _motorStallTimer.stop();

      // hold the motor at the current pos
      _pos.targ = _pos.curr;
      _pos.error = (_pos.targ - _pos.curr);

      return true;
    }
  } else {
    _motorStallTimer.stop();
  }

  return false;
}

// stop the finger if the force limit is reached, or move to reach a target
// force
void Finger::forceController(void) {
  // if force sense is not enabled, return
  if (!_forceSnsEn) {
    return;
  }

  US_NB_DELAY forceTimer; // timer used to detect if the force is being
                          // sustained or is just a small peak
  const int force_pos_increment =
      5; // rate at which the actuator 'searches' for the target force

  // clear force limit flag
  _force.limit.reached = false;

  // if force limit is reached
  if (_force.curr > _force.limit.max) {
    // store current movement direction so that stopping the motor is not
    // perceived as a direction change
    bool tempDir = _dir.curr;

    // stop the motor by setting the target position to the current position
    writePos(readPos());

    // restore the previous direction, as it may have been changed by stopping
    // the motor
    _dir.curr = tempDir;

    _force.limit.reached = true;
  }
}

#endif

// calculate the velocity of the motor
void Finger::calcVel() {
  // if the duration timer is currently running
  // if (_velTimer.started())
  // if (_velTimer.now() > 100000)		// 100ms
  if (_velTimer.now() > 50000) // 50ms
  {
    // get the time and position at the current time
    double duration = _velTimer.now(); // us. time since last vel calc
    double pos = _pos.curr; // store current pos to prevent race condition
    double dist =
        (pos - _pos.prev); // calculate distance moved (using stored pos)

    if (abs(dist) > 0) // if there has been movement
    {
      _speed.raw =
          (dist * (double)100000) / duration; // calc vel in ADC ticks per s?
    } else                                    // else if 0 distance has moved
    {
      _speed.raw = 0.0; // set the velocity to 0
    }

    _velTimer.start();             // restart timer
    _pos.prev = pos;               // save previous pos
  } else if (!_velTimer.started()) // if the vel timer is not currently running
  {
    _velTimer.start(); // start the vel timer
    _speed.raw = 0;    // clear the velocity
  }
}

////////////////////////////// END OF FINGER CLASS
/////////////////////////////////

// runs the control() function of a Finger instance at each call by the timer
// interrupt
void _fingerControlCallback() {
  static int i = 0;

  if (fingerISRList[i] != nullptr) {
    fingerISRList[i]->control(); // run the control() member function of each
                                 // attached Finger instance
  }

  // TODO: Something is terribly wrong, it's not about just disconected finger
  // but also fingers with the wrong index
  i++;
  if (i > _TotalAttachedFingers) {
    i = 0;
  }
}

#ifdef FORCE_SENSE

// runs the calcCurrentSns() function of a Finger instance at each call
void _currentSenseCallback(void) {
  static int i = 0;

  if (fingerISRList[i] != NULL) {
    fingerISRList[i]
        ->calcCurrentSns(); // run the calcCurrentSns() member function of each
                            // attached Finger instance
  }

  i++;
  if (i > _TotalAttachedFingers) {
    i = 0;
  }
}

#endif

#ifdef ARDUINO_AVR_MEGA2560

void setPWMFreq(const uint8_t pin, const uint8_t value) {
  static volatile uint8_t *const timers[] = {nullptr, &TCCR1B, &TCCR2B,
                                             &TCCR3B, &TCCR4B, &TCCR5B};
  // TODO: use an STL with constexpr std::array to replace macros and size_of's
  constexpr auto max_timer_id = sizeof(timers) / sizeof(timers[0]);
  const uint8_t timer_id = PWM_pin_to_timer(pin);
  if (timer_id > 0 && timer_id <= max_timer_id) {
    auto &timer = *timers[timer_id];
    timer = (timer & 0b11111000) | value;
  }
}

#endif