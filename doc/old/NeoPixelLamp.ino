/* The MIT License (MIT)
 *
 * Copyright (c) 2016 - 2017, Andreas Merkle
 * http://www.blue-andi.de
 * web@blue-andi.de
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/

/**
 * @brief  Fun with NeoPixels
 * @file   NeoPixelLamp.ino
 * @author Andreas Merkle, http://www.blue-andi.de
 *
 * @section desc Description
 * This arduino sketch controls a lamp with
 * - an Adafruit FLORA V2 (https://www.adafruit.com/products/659),
 * - an Adafruit NeoPixel Ring - 16 (https://www.adafruit.com/product/1463),
 * - an Adafruit FLORA 9-DOF Accelerometer/Gyroscope/Magnetometer - LSM9DS0 - v1.0 (https://www.adafruit.com/product/2020),
 * - an Sparkfun USB LiPoly Charger - Single Cell (https://www.sparkfun.com/products/12711) and
 * - a LiPo Akku with 400mAh.
 * 
 * Shake the lamp according to the z-axis to switch between different programs.
 * Turn the lamp about 90 degree and rotate it to increase or decrease the LED brightness.
 * Turn the lamp about 180 degree and it will jump into deep standby.
 *
 * Implemented programs:
 * 1. Rainbow colors
 * 2. Theatre-style crawling lights with rainbow effect
 * 3. Color changes dependen on the temperature
 * 4. Color can be changed by rotating around the z-axis
 * 5. Color moves like knight rider
 */
 
/*******************************************************************************
    INCLUDES
*******************************************************************************/

#include <Adafruit_NeoPixel.h>  // Adafruit NeoPixel ring library
#ifdef __AVR__
  #include <avr/power.h>
  #include <avr/sleep.h>
  #include <avr/wdt.h>
#endif
#include <Adafruit_LSM9DS0.h>   // Adafruit triple-axis accelerometer/magnetometer/gyroscope LSM9DS0 library
#include <Adafruit_Sensor.h>    // Adafruit Unified Sensor Driver

/*******************************************************************************
    CONSTANTS
*******************************************************************************/

/**
 * Set to 0 to disable the debug output
 * Set to 1 for debug output on the serial console
 * Set to 2 for plotting the acceleration
 * Set to 3 for calibration
 */
#define NEOPIXELLAMP_DEBUG                  0

/** For debug purposes it is better to wait for an established USB connection (1)
 * otherwise nothing will be seen in the serial console.
 * In normal mode (no debugging) without a USB connection, it must be disabled,
 * otherwise the program will hang in a infinite loop.
 */
#define NEOPIXELLAMP_WAIT_FOR_USB           0

/** Serial baudrate */
#define NEOPIXELLAMP_SERIAL_BAUDRATE        9600

/** The arduino pin, where the NeoPixels are connected. */
#define NEOPIXELLAMP_PIN                    6

/** The arduino pin, where the Flora onboard LED pin is connected. */
#define NEOPIXELLAMP_ONBOARD_LED_PIN        7

/** Number of NeoPixels */
#define NEOPIXELLAMP_NUM_PIXELS             16

/** Max. temperature in degree celsius of the temperature sensor. */
#define NEOPIXELLAMP_MAX_TEMPERATURE        (85.0f)

/** Min. temperature in degree celsius of the temperature sensor */
#define NEOPIXELLAMP_MIN_TEMPERATURE        (-40.0f)

/** Acceleration threshold in m/s^2, which triggers a mode change */
#define NEOPIXELLAMP_MODE_CHANGE_THRESHOLD  (4.0f)

/** Earth gravity acceleration in m/s^2 */
#define NEOPIXELLAMP_EARTH_GRAVITY          (9.81f)

/** Min. brightness (0-255) of the NeoPixels. */
#define NEOPIXELLAMP_MIN_BRIGHTNESS         10

/** Max. brightness (1-255) of the NeoPixels. */
#define NEOPIXELLAMP_MAX_BRIGHTNESS         160

/* ---------- Note, the following constants must be calibrated for every lamp instance. ---------- */

/** Acceleration offset calibration value for x in m/s^2 */
#define NEOPIXELLAMP_ACCELERATION_OFFSET_X  (-1.81f)

/** Acceleration offset calibration value for y in m/s^2 */
#define NEOPIXELLAMP_ACCELERATION_OFFSET_Y  (-0.45f)

/** Acceleration offset calibration value for z in m/s^2 */
#define NEOPIXELLAMP_ACCELERATION_OFFSET_Z  (9.79f - NEOPIXELLAMP_EARTH_GRAVITY)

/** Gyroscope offset calibration value for x in degree/s */
#define NEOPIXELLAMP_GYRO_OFFSET_X          (-1.24f)

/** Gyroscope offset calibration value for y in degree/s */
#define NEOPIXELLAMP_GYRO_OFFSET_Y          (2.23f)

/** Gyroscope offset calibration value for z in degree/s */
#define NEOPIXELLAMP_GYRO_OFFSET_Z          (-1.99f)

/** Magnetic offset calibration value for x in gauss */
#define NEOPIXELLAMP_MAGNETIC_OFFSET_X      (0.10f)

/** Magnetic offset calibration value for y in gauss */
#define NEOPIXELLAMP_MAGNETIC_OFFSET_Y      (0.40f)

/** Magnetic offset calibration value for z in gauss */
#define NEOPIXELLAMP_MAGNETIC_OFFSET_Z      (0.48f)

/*******************************************************************************
    MACROS
*******************************************************************************/

/** Get number of elements in the given array. */
#define ARRAY_NUM(__arr)  (sizeof(__arr) / sizeof((__arr)[0]))

/*******************************************************************************
    TYPES AND STRUCTURES
*******************************************************************************/

/** Supported application modes */
typedef enum
{
  NEOPIXELLAMP_MODE_ID_RAINBOW = 0, /**< Rainbow colors */
  NEOPIXELLAMP_MODE_ID_THEATER,     /**< Theatre-style crawling lights with rainbow effect */
  NEOPIXELLAMP_MODE_ID_TEMPERATURE, /**< Color changes dependend on the temperature */
  NEOPIXELLAMP_MODE_ID_COLOR,       /**< Color can be changed by rotating */
  NEOPIXELLAMP_MODE_ID_KNIGHT_RIDER,/**< Color moves like knight rider */
  NEOPIXELLAMP_MODE_ID_SLEEP,       /**< Lamp is off */
  NEOPIXELLAMP_MODE_ID_DEBUG,       /**< Just debug output on the serial console */
  NEOPIXELLAMP_MODE_ID_DEBUG_PLOT,  /**< Plot data on the serial console */
  NEOPIXELLAMP_MODE_ID_DEBUG_CAL,   /**< Debug output on the serial console for calibration */
  NEOPIXELLAMP_MODE_ID_INVALID      /**< Invalid mode */
  
} NEOPIXELLAMP_MODE_ID;

/** Simple function pointer type */
typedef void (*SimpleFunc)(void);

/** Mode context */
typedef struct
{
  NEOPIXELLAMP_MODE_ID  id;         /**< Mode id */
  char*                 name;       /**< Mode name as string */
  SimpleFunc            enter;      /**< Function which is called once before start processing */
  SimpleFunc            leave;      /**< Function which is called once before next mode starts */
  SimpleFunc            taskFunc;   /**< Function which is called periodically, if the mode is active */
  
} Mode;

/** Task definition */
typedef struct
{
  uint8_t       period;     /**< Calling period in ms */
  bool          isActive;   /**< Task is active or not */
  unsigned long lastTime;   /**< Last time, the task was called */
  SimpleFunc    taskFunc;   /**< Task function */
  
} Task;

/*******************************************************************************
    PROTOTYPES
*******************************************************************************/

static void ledOn(void);
static void ledOff(void);
static const Mode* findMode(NEOPIXELLAMP_MODE_ID id, uint8_t* modeIndex);
static void changeMode(NEOPIXELLAMP_MODE_ID id);
static void scheduler(void);
static Task* findTask(SimpleFunc taskFunc);
static void resumeTask(SimpleFunc taskFunc);
static void suspendTask(SimpleFunc taskFunc);
static void controlBrightness(float absz, float az, float gz);
static void task20ms(void);
static void leaveModeRainbow(void);
static void taskRainbow(void);
static void enterModeTheater(void);
static void leaveModeTheater(void);
static void taskTheater(void);
static void enterModeTemperature(void);
static void leaveModeTemperature(void);
static void taskTemperature(void);
static void enterModeColor(void);
static void leaveModeColor(void);
static void taskColor(void);
static void taskKnightRider(void);
static void enterModeSleep(void);
static void leaveModeSleep(void);
static void taskSleep(void);
static void taskDebug(void);
static void taskDebugPlot(void);
static void taskDebugCal(void);
static float getAbsAcceleration(sensors_vec_t* acceleration);
static void getAcceleration(sensors_vec_t* acceleration);
static void getMagnetic(sensors_vec_t* magnetic);
static float getHeading(void);
static void getGyro(sensors_vec_t* gyro);
static float getTemperatureC(void);
static void getGyroAvg(sensors_vec_t* gyroAvg);
static void redSplash(uint8_t num);
static uint32_t colorWheel(uint8_t wheelPos);

/*******************************************************************************
    LOCAL VARIABLES
*******************************************************************************/

/**
 * List of available modes. The order of the modes is the order after which
 * the user can change it by shaking the lamp along the z-axis.
 */
static const Mode modes[] = {
  
#if (1 == NEOPIXELLAMP_DEBUG)
  { NEOPIXELLAMP_MODE_ID_DEBUG,         "DEBUG",        NULL,                 NULL,                 taskDebug       },
#endif

#if (2 == NEOPIXELLAMP_DEBUG)
  { NEOPIXELLAMP_MODE_ID_DEBUG_PLOT,    "DEBUG-PLOT",   NULL,                 NULL,                 taskDebugPlot   },
#endif

#if (3 == NEOPIXELLAMP_DEBUG)
  { NEOPIXELLAMP_MODE_ID_DEBUG_CAL,     "DEBUG-CAL",    NULL,                 NULL,                 taskDebugCal    },
#endif

  { NEOPIXELLAMP_MODE_ID_RAINBOW,       "RAINBOW",      NULL,                 leaveModeRainbow,     taskRainbow     },
  { NEOPIXELLAMP_MODE_ID_THEATER,       "THEATER",      enterModeTheater,     leaveModeTheater,     taskTheater     },
  { NEOPIXELLAMP_MODE_ID_TEMPERATURE,   "TEMPERATURE",  enterModeTemperature, leaveModeTemperature, taskTemperature },
  { NEOPIXELLAMP_MODE_ID_COLOR,         "COLOR",        enterModeColor,       leaveModeColor,       taskColor       },
  { NEOPIXELLAMP_MODE_ID_KNIGHT_RIDER,  "KNIGHT-RIDER", NULL,                 NULL,                 taskKnightRider },
  { NEOPIXELLAMP_MODE_ID_SLEEP,         "SLEEP",        enterModeSleep,       leaveModeSleep,       taskSleep       }
};

#if (2 == NEOPIXELLAMP_DEBUG)

/** Tasks used by debug mode 2 */
static Task tasks[] = {
  
  //  Period  Active  Last time   Task function
  {   20,     false,  0,          taskDebugPlot   }   // Just for debug purposes some information on the serial console
};

#elif (3 == NEOPIXELLAMP_DEBUG)

/** Tasks used by debug mode 3 */
static Task tasks[] = {
  
  //  Period  Active  Last time   Task function
  {   20,     false,  0,          taskDebugCal    }   // Print average values to the serial console, used for calibration
}; 
    
#else

/** Tasks used in normal mode (no debugging) */
static Task tasks[] = {
  
  //  Period  Active  Last time   Task function
  {   20,     true,   0,          task20ms        },  // Handles the main user interface, reads the whole LSM9DS0 data
  {   100,    false,  0,          taskRainbow     },  // Rainbow colors
  {   60,     false,  0,          taskTheater     },  // Theater like rainbow colors
  {   100,    false,  0,          taskTemperature },  // Color dependend on the measured temperature
  {   20,     false,  0,          taskColor       },  // User choosable color (turn the lamp to choose)
  {   20,     false,  0,          taskKnightRider },  // Colors like knight rider
  {   0,      false,  0,          taskSleep       },  // System sleeps to reduce power consumption
  {   100,    false,  0,          taskDebug       }   // Just for debug purposes some information on the serial console
};

#endif

/** Current application mode */
static const Mode*        mode                  = NULL;

/** Request a mode initialization */
static bool               reqModeInit           = false;

/**
 * When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
 * Parameter 1 = number of pixels in strip
 * Parameter 2 = Arduino pin number (most are valid)
 * Parameter 3 = pixel type flags, add together as needed:
 *    NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
 *    NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
 *    NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
 *    NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
 */
static Adafruit_NeoPixel  neoPixel              = Adafruit_NeoPixel(NEOPIXELLAMP_NUM_PIXELS, NEOPIXELLAMP_PIN, NEO_GRB + NEO_KHZ800);

/** Create the LSM9DS0 magnetic sensor object, I2C address 0xD4 */
static Adafruit_LSM9DS0   lsm                   = Adafruit_LSM9DS0(0xD4);

/** Mode changed triggered by acceleration? */
static bool               isModeChangeTriggered = false;

/** Last measured temperature in degree celsius */
static float              lastTemperatureC      = 0.0f;

/*******************************************************************************
    GLOBAL FUNCTIONS
*******************************************************************************/

/**
 * Setup the system, once called after power-up.
 */
void setup() {

  uint8_t lsmRetry  = 3;

  // Enable the flora onboard LED
  pinMode(NEOPIXELLAMP_ONBOARD_LED_PIN, OUTPUT);
  
  // The onboard LED shall be on during the whole setup
  ledOn();

  // Set serial baudrate
  Serial.begin(NEOPIXELLAMP_SERIAL_BAUDRATE);
  
#if (0 != NEOPIXELLAMP_WAIT_FOR_USB)
  
  // Wait for serial port to connect. Needed for native USB
  while (!Serial);

#endif
  
  Serial.println("Setup NeoPixelLamp ...");

  // Disable some peripherals to reduce power consumption
  power_adc_disable();
  power_spi_disable();
  power_timer2_disable();
  power_timer3_disable();
  
  // Initialize the NeoPixel library
  neoPixel.begin();
  // Initialize all pixels to 'off'
  neoPixel.show();
  // Set brightness to reduce power consumption
  neoPixel.setBrightness(NEOPIXELLAMP_MAX_BRIGHTNESS);

  // Make sure the LSM9DS0 sensor is found
  while((0 < lsmRetry) && (0 == lsm.begin())) {
    --lsmRetry;
  }

  // LSM9DS0 not found?
  if (0 == lsmRetry) {
    Serial.println("Couldn't find LSM9DS0!");
    delay(100);

    // Enable watchdog and wait for reset
    wdt_enable(WDTO_15MS);
    while(1);
  }
  else {
    Serial.println("Found LSM9DS0.");

    // Set accelerometer anti-alias filter bandwidth to 50 Hz
    uint8_t reg = lsm.read8(XMTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG2_XM);
    reg &= 0x3f;
    reg |= 0xc0;
    lsm.write8(XMTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG2_XM, reg);

    // 1.) Set the accelerometer range
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
    //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
    //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
    //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
    
    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
    //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
    //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);
  
    // 3.) Setup the gyroscope
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
    //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
    //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);

    Serial.println("NeoPixelLamp is ready.");
    
    // Change to the first mode
    changeMode(NEOPIXELLAMP_MODE_ID_INVALID);
  }

  // Clear onboard LED
  ledOff();

  return;
}

/** Main loop, called as fast as possible. */
void loop() {

  /* The scheduler calls the activated tasks according to their configured period. */
  scheduler();
  
  return;
}

/*******************************************************************************
    LOCAL FUNCTIONS
*******************************************************************************/

/** Set onboard LED on. */
static void ledOn(void) {
  digitalWrite(NEOPIXELLAMP_ONBOARD_LED_PIN, HIGH);
  return;
}

/** Set onboard LED off. */
static void ledOff(void) {
  digitalWrite(NEOPIXELLAMP_ONBOARD_LED_PIN, LOW);
  return;
}

/**
 * Find mode id in the mode list and return a pointer to the mode.
 * If requested, return the index in the mode list as well.
 *
 * @param[in]   id        Mode id, which to find
 * @param[out]  modeIndex Index in the mode list
 *
 * @return Pointer to the mode entry in the list.
 * @retval NULL Mode not found.
 */
static const Mode* findMode(NEOPIXELLAMP_MODE_ID id, uint8_t* modeIndex) {
  
  const Mode* mode  = NULL;
  uint8_t     index = 0;
  
  for(index = 0; index < ARRAY_NUM(modes); ++index) {
    
    if (id == modes[index].id) {
      mode = &modes[index];
      
      if (NULL != modeIndex) {
        *modeIndex = index;
      }
      break;
    }
  }
  
  return mode;
}

/** Change application mode.
 *
 * @param[in] id    Mode id
 */
static void changeMode(NEOPIXELLAMP_MODE_ID id) {

  uint8_t     index     = 0;
  const Mode* nextMode  = NULL;
  const Mode* reqMode   = findMode(id, NULL);

  // If no mode is set yet, set the first one
  if (NULL == mode) {
    nextMode = &modes[0];
  }
  // Set a specific mode?
  else if (NULL != reqMode) {
    nextMode = reqMode;
  }
  else {
    
    (void)findMode(mode->id, &index);
    
    do {
      ++index;
      
      // Wrap around
      if (ARRAY_NUM(modes) <= index) {
        index = 0;
      }
    }
    while(NEOPIXELLAMP_MODE_ID_SLEEP == modes[index].id);
    
    nextMode = &modes[index];
  }

  if (mode != nextMode) {

    Serial.print("Change mode to ");
    Serial.println(nextMode->name);
  
    redSplash((uint8_t)nextMode->id + 1);
  
    // Leave current mode
    if (NULL != mode) {
      
      suspendTask(mode->taskFunc);
      
      if (NULL != mode->leave) {
        mode->leave();
      }
    }
  
    // Enter next mode
    if (NULL != nextMode->enter) {
      nextMode->enter();
    }

    resumeTask(nextMode->taskFunc);
  
    mode = nextMode;
  }

  return;
}

/**
 * Simple task scheduler.
 * It schedules all tasks which are contained in the tasks[] array according to
 * the configured period, dependend whether a task is activated or not.
 */
static void scheduler(void) {

  uint8_t       index       = 0;
  unsigned long currentTime = millis();
  unsigned long diffTime    = 0;

  for(index = 0; index < ARRAY_NUM(tasks); ++index) {

    if ((true == tasks[index].isActive) &&
        (NULL != tasks[index].taskFunc)) {
      
      if (currentTime < tasks[index].lastTime) {
        diffTime = currentTime + ((unsigned long)(-1) - tasks[index].lastTime);
      }
      else {
        diffTime = currentTime - tasks[index].lastTime;
      }

      if (tasks[index].period <= diffTime) {
        tasks[index].taskFunc();

        tasks[index].lastTime = currentTime;
      }
    }
  }

  return;
}

/**
 * Find a task in the task list and return a pointer to it.
 *
 * @param[in] taskFunc  Task function pointer
 *
 * @return Task context
 * @retval NULL Task not found.
 */
static Task* findTask(SimpleFunc taskFunc) {
  
  uint8_t index = 0;
  Task*   task  = NULL;
  
  for(index = 0; index < ARRAY_NUM(tasks); ++index) {
    
    if (tasks[index].taskFunc == taskFunc) {
      task = &tasks[index];
      break;
    }
  }

  return task;
}

/**
 * Resume the task.
 * Note, the task will be called immediately in the scheduler once after
 * periodically as configured.
 *
 * @param[in]   taskFunc    Task function pointer
 */
static void resumeTask(SimpleFunc taskFunc) {
  
  Task* task = findTask(taskFunc);

  if (NULL != task) {   
    task->isActive  = true;

    // Task shall be called immediately
    task->lastTime  = millis() + task->period;
  }
    
  return;
}

/**
 * Suspend the task.
 *
 * @param[in]   taskFunc    Task function pointer
 */
static void suspendTask(SimpleFunc taskFunc) {
  
  Task* task = findTask(taskFunc);

  if (NULL != task) {
    task->isActive = false;
  }
    
  return;
}

/**
 * Control the NeoPixel ring LEDs brightness by rotating the lamp
 * around z-axis. The z-axis must be horizontal to earth.
 *
 * @param[in]   absz    Absolute acceleration value in m/s^2
 * @param[in]   az      Acceleration along the z-axis in m/s^2
 * @param[in]   gz      Angle speed around the z-axis in degree/s
 */
static void controlBrightness(float absz, float az, float gz) {
  
  const float   cMinAngularVelocity = 20.0f;
  const float   cMinAcceleration    = 1.6f;
  const float   cMinAbsAcceleration = NEOPIXELLAMP_EARTH_GRAVITY - 1.6f;
  uint8_t       brightness          = 0;
  bool          hasChanged          = false;

  // Nearly no acceleration shall take place
  if (cMinAbsAcceleration < fabs(absz - NEOPIXELLAMP_EARTH_GRAVITY)) {
    // Do nothing
  }
  // The lamp shall be horizontal to earth. That means the z-axis
  // acceleration should be low, otherwise it would be around 9.81 m/s^2
  else if (cMinAcceleration > fabs(az)) {

    // If the lamp is turned right, the brightness increases
    if (cMinAngularVelocity < gz) {
      
      brightness = neoPixel.getBrightness();

      if (NEOPIXELLAMP_MAX_BRIGHTNESS > brightness) {
        
        ++brightness;

        hasChanged = true;
      }
    }
    // If the lamp is turned left, the brightness decreases
    else if (-cMinAngularVelocity > gz) {
      
      brightness = neoPixel.getBrightness();

      if (NEOPIXELLAMP_MIN_BRIGHTNESS < brightness) {
        
        --brightness;

        hasChanged = true;
      }
    }
    // Nothing to do
    else {
      ;
    }
  }

  if (true == hasChanged) {

    Serial.print("Change brightness to ");
    Serial.println(brightness);
    
    neoPixel.setBrightness(brightness);
    neoPixel.show();
  }
  
  return;
}

/**
 * The 20ms task handles the main user interface.
 */
static void task20ms(void) {

  float         absAcceleration     = 0.0f;
  float         absAccelDiff        = 0.0f;
  sensors_vec_t acceleration;
  sensors_vec_t gyro;
  const float   cMinAbsAcceleration = NEOPIXELLAMP_EARTH_GRAVITY - 2.0f;
  
  // Read all sensor values
  lsm.read();
  
  getAcceleration(&acceleration);
  getGyroAvg(&gyro);
  
  absAcceleration = getAbsAcceleration(&acceleration);
  absAccelDiff    = fabs(absAcceleration - NEOPIXELLAMP_EARTH_GRAVITY);

  // Control brightness
  controlBrightness(absAcceleration, acceleration.z, gyro.z);

  // Sleep mode requested, by turning the lamp on the head?
  if ((-cMinAbsAcceleration > acceleration.z) &&
      ((NEOPIXELLAMP_MODE_ID_SLEEP != mode->id))) {
        
    // Change to SLEEP mode
    changeMode(NEOPIXELLAMP_MODE_ID_SLEEP);
  }
  // Resume from sleep mode requested, by turning lamp on the bottom?
  else if ((cMinAbsAcceleration < acceleration.z) &&
           (NEOPIXELLAMP_MODE_ID_SLEEP == mode->id)) {

    // Wake up and change to next mode
    changeMode(NEOPIXELLAMP_MODE_ID_INVALID);
  }
  // Does the user requested a mode change by shaking the lamp?
  else if ((NEOPIXELLAMP_MODE_CHANGE_THRESHOLD <= absAccelDiff) && // Lamp shall be shaked with a min. acceleration
           (1.0f > fabs(absAcceleration - acceleration.z)) &&     // Lamp shall be shaked along the z-axis
           (20.0f > fabs(gyro.x)) &&                              // Lamp shall not be rotated at all
           (20.0f > fabs(gyro.y)) &&
           (20.0f > fabs(gyro.z)) &&
           (false == isModeChangeTriggered)) {

    // Change to next mode
    changeMode(NEOPIXELLAMP_MODE_ID_INVALID);    

    isModeChangeTriggered = true;
  }
  else if (0.5f > absAccelDiff) {
    isModeChangeTriggered = false;
  }
  
  return;  
}

/**
 * Clean up rainbow mode.
 */
static void leaveModeRainbow(void) {

  neoPixel.clear();
  neoPixel.show();
  
  return;
}

/**
 * Rainbow colors.
 */
static void taskRainbow(void) {

  uint16_t run = 0;
  
  static uint16_t index = 0;

  for(run = 0; run < neoPixel.numPixels(); ++run) {
    neoPixel.setPixelColor(run, colorWheel((uint8_t)((run + index) & 0x00ff)));
  }
  neoPixel.show();

  ++index;

  if (256 <= index) {
    index = 0;
  }
  
  return;
}

/**
 * Setup mode theater.
 */
static void enterModeTheater(void) {

  reqModeInit = true;
  
  return;
}

/**
 * Clean up mode theater.
 */
static void leaveModeTheater(void) {

  neoPixel.clear();
  neoPixel.show();
  
  return;
}

/**
 * Theatre-style crawling lights with rainbow effect
 */
static void taskTheater(void) {

  uint8_t       run = 0;

  static uint8_t  colorIndex  = 0;
  static uint8_t  q           = 0;

  if (false == reqModeInit) {
    
    for (run = 0; run < neoPixel.numPixels(); run += 3) {
      // Turn every third pixel off
      neoPixel.setPixelColor(run + q, 0);
    }

    if (2 == q) {
      
      q = 0;

      if (255 == colorIndex) {
        colorIndex = 0;
      }
      else {
        ++colorIndex;
      }
      
    }
    else {
      ++q;
    }
  }
  else {
    reqModeInit = false;
  }
  
  for (run = 0; run < neoPixel.numPixels(); run += 3) {
    
    // Turn every third pixel on
    neoPixel.setPixelColor(run + q, colorWheel( (run + colorIndex) % 255));
  }
  
  neoPixel.show();

  return;
}

/**
 * Setup mode temperature.
 */
static void enterModeTemperature(void) {

  // Force color update
  lastTemperatureC = NEOPIXELLAMP_MAX_TEMPERATURE + 10.0f;
 
  return;
}

/**
 * Clean up mode temperature.
 */
static void leaveModeTemperature(void) {

  neoPixel.clear();
  neoPixel.show();
  
  return;
}

/**
 * A color is shown, dependend on the measured temperature.
 */
static void taskTemperature(void) {

  int16_t temperatureC      = getTemperatureC();  // Range: -40 to +85 degree celsius
  uint8_t index             = 0;                  // NeoPixel ring LED index
  int32_t tmp               = 0;                  // Temporary variable, used for temperature average calculation
  float   temperatureAvgC   = 0.0f;               // Temperature average in degree celsius

  static uint8_t  temperatureIndex  = 0xff; // Temperature array index
  static int16_t  temperaturesC[10];        // Single temperature values for average calculation

  // Shall the temperature array be initialized?
  if (0xff == temperatureIndex) {

    for(index = 0; index < ARRAY_NUM(temperaturesC); ++index) {
      temperaturesC[index] = temperatureC;
    }

    temperatureIndex = 0;
  }
  else {
    // Add temperature to temperature array, like in a ring
    temperaturesC[temperatureIndex] = temperatureC;
    ++temperatureIndex;

    // Wrap around?
    if (ARRAY_NUM(temperaturesC) <= temperatureIndex) {
      temperatureIndex = 0;
    }
  }

  // Calculate temperature average
  for(index = 0; index < ARRAY_NUM(temperaturesC); ++index) {
    temperatureAvgC += (float)temperaturesC[index];
  }

  temperatureAvgC /= (float)ARRAY_NUM(temperaturesC);

  if (0.1f <= fabs(lastTemperatureC - temperatureAvgC)) {

    uint16_t  run         = 0;
    uint32_t  colors[]    = {
      neoPixel.Color(255, 0, 255),  // Purple
      neoPixel.Color(0, 0, 255),    // Blue
      neoPixel.Color(0, 255, 255),  // Cyan
      neoPixel.Color(0, 255, 0),    // Green
      neoPixel.Color(255, 255, 0),  // Yellow
      neoPixel.Color(255, 0, 0)     // Red
    };
    float     normalizedTemperature = 0.0f;
    float     colorIndex            = 0.0f;
    uint8_t   colorIndexHigh        = 0;
    uint8_t   colorIndexLow         = 0;
    float     dx                    = 0.0f;
    uint8_t   red                   = 0;
    uint8_t   green                 = 0;
    uint8_t   blue                  = 0;
    uint32_t  color                 = 0;

    // Limit measured temperature to sensor range
    if (NEOPIXELLAMP_MIN_TEMPERATURE > temperatureAvgC) {
      temperatureAvgC = NEOPIXELLAMP_MIN_TEMPERATURE;
    }
    else if (NEOPIXELLAMP_MAX_TEMPERATURE < temperatureAvgC) {
      temperatureAvgC = NEOPIXELLAMP_MAX_TEMPERATURE;
    }

    normalizedTemperature = (temperatureAvgC - NEOPIXELLAMP_MIN_TEMPERATURE) / (NEOPIXELLAMP_MAX_TEMPERATURE - NEOPIXELLAMP_MIN_TEMPERATURE);
    colorIndex            = normalizedTemperature * (ARRAY_NUM(colors) - 1);  // Calculate color index
    colorIndexHigh        = (int16_t)ceil(colorIndex);                        // Round up
    colorIndexLow         = (int16_t)floor(colorIndex);                       // Round down
    dx                    = colorIndexHigh - colorIndex;                      // Calculate color index distance

    // Interpolate
    red   = (uint8_t) ( (float)((colors[colorIndexHigh] >> 16) & 0xff) * (1.0f - dx) + (float)((colors[colorIndexLow] >> 16) & 0xff) * dx );
    green = (uint8_t) ( (float)((colors[colorIndexHigh] >>  8) & 0xff) * (1.0f - dx) + (float)((colors[colorIndexLow] >>  8) & 0xff) * dx );
    blue  = (uint8_t) ( (float)((colors[colorIndexHigh] >>  0) & 0xff) * (1.0f - dx) + (float)((colors[colorIndexLow] >>  0) & 0xff) * dx );
    
    Serial.print("Temperature: ");
    Serial.print(temperatureAvgC, 1);
    Serial.println(" *C");
    Serial.print("RGB        : ");
    Serial.print(red);
    Serial.print(" ");
    Serial.print(green);
    Serial.print(" ");
    Serial.println(blue);

    color = neoPixel.Color(red, green, blue);
    
    // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
    for(run = 0; run < neoPixel.numPixels(); ++run) {
  
      // neoPixel.Color takes RGB values, from 0,0,0 up to 255,255,255
      neoPixel.setPixelColor(run, color);
  
      // This sends the updated pixel color to the hardware.
      neoPixel.show();  
    }

    lastTemperatureC = temperatureAvgC;
  }
  
  return;
}

/**
 * Setup mode color.
 */
static void enterModeColor(void) {

  reqModeInit = true;
  
  return;
}

/**
 * Clean up mode color.
 */
static void leaveModeColor(void) {

  neoPixel.clear();
  neoPixel.show();
  
  return;
}

/**
 * User can choose a color by turning the lamp.
 */
static void taskColor(void) {
  
  uint16_t      run         = 0;
  sensors_vec_t gyro;
  sensors_vec_t acceleration;
  bool          hasChanged  = false;
  
  static uint8_t  wheelPos  = 0;

  getGyroAvg(&gyro);
  getAcceleration(&acceleration);

  // The lamp shall stand on the tail
  if (8.0f < fabs(acceleration.z)) {

    // If the lamp is turned right, the color wheel position increases
    if (20.0f < gyro.z) {
      ++wheelPos;
      hasChanged = true;
    }
    // If the lamp is turned left, the color wheel position decreases
    else if (-20.0f > gyro.z) {
      --wheelPos;
      hasChanged = true;
    }
    // Nothing to do
    else {
      ;
    }
  }

  if ((true == reqModeInit) ||
      (true == hasChanged)) {

    Serial.print("Wheel pos.: ");
    Serial.println(wheelPos);
  
    // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
    for(run = 0; run < neoPixel.numPixels(); ++run) {
  
      // neoPixel.Color takes RGB values, from 0,0,0 up to 255,255,255
      neoPixel.setPixelColor(run, colorWheel(wheelPos));
  
      // This sends the updated pixel color to the hardware.
      neoPixel.show();  
    }

    reqModeInit = false;
  }
    
  return;
}

/**
 * Colors like knight rider.
 */
static void taskKnightRider(void) {

  uint16_t  run = 0;
  uint8_t   pos = 0;
  
  static uint8_t  index   = 0;
  static uint8_t  oldPos  = neoPixel.numPixels() - 1;
  static uint8_t  dir     = 0;
  static uint8_t  retPos  = neoPixel.numPixels() - 1;

  if (0 == dir) {

    if ((neoPixel.numPixels() - 1) == oldPos) {
      pos = 0;
    }
    else {
      pos = oldPos + 1;
    }

    // If the LED reached the return position, change the direction.
    if (retPos == pos) {
      dir = 1;
    }
    
  }
  else {

    if (0 == oldPos) {
      pos = neoPixel.numPixels() - 1;
    }
    else {
      pos = oldPos - 1;
    }

    // If the LED reached the return position, change the direction.
    if (retPos == pos) {
      dir = 0;

      // Rotate the LED return position
      if (0 == retPos) {
        retPos = neoPixel.numPixels() - 1;
      }
      else {
        --retPos;
      }
      
    }
    
  }
  
  neoPixel.setPixelColor(oldPos, 0);
  neoPixel.setPixelColor(pos, colorWheel(index));

  neoPixel.show();

  ++index;
  oldPos = pos;
  
  return;
}

/**
 * Setup sleep mode.
 */
static void enterModeSleep(void) {

  uint8_t run           = 0;
  uint8_t oldBrightness = neoPixel.getBrightness(); // Remember current brightness

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for(run = 0; run < neoPixel.numPixels(); ++run) {

    // neoPixel.Color takes RGB values, from 0,0,0 up to 255,255,255
    neoPixel.setPixelColor(run, neoPixel.Color(255, 0, 0));

    // This sends the updated pixel color to the hardware.
    neoPixel.show();
  }

  for(run = NEOPIXELLAMP_MAX_BRIGHTNESS; run > 0; --run) {

    neoPixel.setBrightness(run);
    neoPixel.show();

    delay(10);
  }

  neoPixel.clear();
  neoPixel.show();

  // Restore original brightness
  neoPixel.setBrightness(oldBrightness);

  // Disable onboard LED
  ledOff();

  // Disable as much as possible
  suspendTask(task20ms);
  
  return;
}

/**
 * Clean up sleep mode.
 */
static void leaveModeSleep(void) {
  
  resumeTask(task20ms);
  
  return;
}

/**
 * System enters sleep mode to reduce power consumption.
 */
static void taskSleep(void) {

  sensors_vec_t acceleration;

  lsm.readAccel();
  
  getAcceleration(&acceleration);

  // Resume from sleep mode requested?
  if ((NEOPIXELLAMP_EARTH_GRAVITY - 2.0f) < acceleration.z) {

    // Wake up and change to next mode
    changeMode(NEOPIXELLAMP_MODE_ID_INVALID);
  }
  else {

    // Power down the LSM9DS0 sensor
    uint8_t regCtrlReg1G = lsm.read8(GYROTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG1_G);
    regCtrlReg1G &= ~(0x08);
    lsm.write8(GYROTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG1_G, regCtrlReg1G); 

    // Configure sleep mode power down
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Enable watchdog
    // Watchdog timer          : 1 s
    // Watchdog timer interrupt: enabled
    // Watchdog reset          : disabled
    {
      cli();
      
      wdt_reset();
      
      // Prepare watchdog for timer interrupt without reset
      MCUSR &= ~(1 << WDRF);  // Clear WDRF, which overwrites WDE
      
      // In the same operation, write a logic one to the Watchdog
      // change enable bit (WDCE) and WDE. A logic one must be
      // written to WDE regardless of the previous value of the WDE bit.
      WDTCSR |= (1 << WDCE) | (1 << WDE);

      // Within the next four clock cycles, write the WDE and Watchdog
      // prescaler bits (WDP) as desired, but with the WDCE bit cleared.
      // This must be done in one operation.
      // Watchdog timer: 1 s
      // Watchdog interrupt mode
      WDTCSR = (1 << WDIE) | (0 << WDCE) | (0 << WDE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);

      sei();
    }
    
    // Sleep now
    sleep_mode();
    
    // Waked up again ...
    sleep_disable();

    // Power up the LSM9DS0 sensor
    regCtrlReg1G |= 0x08;
    lsm.write8(GYROTYPE, Adafruit_LSM9DS0::LSM9DS0_REGISTER_CTRL_REG1_G, regCtrlReg1G); 
  }
  
  return;
}

/**
 * Console output for debugging purposes.
 */
static void taskDebug(void) {

  sensors_vec_t acceleration;
  sensors_vec_t magnetic;
  sensors_vec_t gyro;
  float         absAcceleration = 0.0f;
  float         temperature     = 0.0f;
  float         heading         = 0.0f;

  getAcceleration(&acceleration);
  getMagnetic(&magnetic);
  getGyro(&gyro);
  absAcceleration = getAbsAcceleration(&acceleration);
  temperature     = getTemperatureC();
  heading         = getHeading();

  Serial.print("Acceleration values  : X = ");
  Serial.print(acceleration.x, 2);
  Serial.print(" Y = ");
  Serial.print(acceleration.y, 2);
  Serial.print(" Z = ");
  Serial.println(acceleration.z, 2);
  
  Serial.print("Magnetic values      : X = ");
  Serial.print(magnetic.x, 2);
  Serial.print(" Y = ");
  Serial.print(magnetic.y, 2);
  Serial.print(" Z = ");
  Serial.println(magnetic.z, 2);
  
  Serial.print("Gyroscope values     : X = ");
  Serial.print(gyro.x, 2);
  Serial.print(" Y = ");
  Serial.print(gyro.y, 2);
  Serial.print(" Z = ");
  Serial.println(gyro.z, 2);
  
  Serial.print("Absolute acceleration: ");
  Serial.print(absAcceleration, 2);
  Serial.println(" m/s^2");
  
  Serial.print("Temperature          : ");
  Serial.print(temperature, 1);
  Serial.println(" degree celsius");
  
  Serial.print("Heading              : ");
  Serial.print(heading, 2);
  Serial.println(" degree");
  
  Serial.println("");
  
  return;
}

/**
 * Console output for debugging purposes.
 */
static void taskDebugPlot(void) {

#if 0

  sensors_vec_t acceleration;
  float         absAcceleration = 0.0f;

  lsm.readAccel();
  getAcceleration(&acceleration);

  Serial.print(acceleration.x, 2);
  Serial.print(",");
  Serial.print(acceleration.y, 2);
  Serial.print(",");
  Serial.println(acceleration.z, 2);

  //absAcceleration = getAbsAcceleration(&acceleration);
  //Serial.println(absAcceleration, 2);

#elif 1

  const uint8_t         cNum    = 10;
  static sensors_vec_t  gyro[cNum];
  static uint8_t        index   = 0;
  uint8_t               run     = 0;
  sensors_vec_t         gyroAvg = { 0.0f, 0.0f, 0.0f };

  lsm.readGyro();
  getGyro(&gyro[index]);

  ++index;
  if (cNum <= index) {
    index = 0;
  }

  for(run = 0; run < cNum; ++run) {
    gyroAvg.x += gyro[run].x;
    gyroAvg.y += gyro[run].y;
    gyroAvg.z += gyro[run].z;
  }

  gyroAvg.x /= cNum;
  gyroAvg.y /= cNum;
  gyroAvg.z /= cNum;

  Serial.print(gyroAvg.x, 2);
  Serial.print(",");
  Serial.print(gyroAvg.y, 2);
  Serial.print(",");
  Serial.println(gyroAvg.z, 2);

#else

  Serial.println(acceleration.z, 2);

#endif
  
  return;
}

/**
 * Console output for calibration.
 */
static void taskDebugCal(void) {

  const uint8_t         cNum    = 10;

  static uint8_t        index   = 0;
  static sensors_vec_t  acceleration[cNum];
  static sensors_vec_t  gyro[cNum];
  static sensors_vec_t  magnetic[cNum];
  static uint8_t        counter = 0;
  
  uint8_t       run              = 0;
  sensors_vec_t accelerationAvg  = { 0.0f, 0.0f, 0.0f };
  sensors_vec_t gyroAvg          = { 0.0f, 0.0f, 0.0f };
  sensors_vec_t magneticAvg      = { 0.0f, 0.0f, 0.0f };
  
  // Read all sensor values
  lsm.read();
  
  getAcceleration(&acceleration[index]);
  getGyro(&gyro[index]);
  getMagnetic(&magnetic[index]);
  
  // Calculate average
  for(run = 0; run < cNum; ++run) {
      
    accelerationAvg.x += acceleration[run].x;
    accelerationAvg.y += acceleration[run].y;
    accelerationAvg.z += acceleration[run].z;
    
    gyroAvg.x += gyro[run].x;
    gyroAvg.y += gyro[run].y;
    gyroAvg.z += gyro[run].z;

    magneticAvg.x += magnetic[run].x;
    magneticAvg.y += magnetic[run].y;
    magneticAvg.z += magnetic[run].z;
  }

  accelerationAvg.x /= (float)cNum;
  accelerationAvg.y /= (float)cNum;
  accelerationAvg.z /= (float)cNum;
  
  gyroAvg.x /= (float)cNum;
  gyroAvg.y /= (float)cNum;
  gyroAvg.z /= (float)cNum;

  magneticAvg.x /= (float)cNum;
  magneticAvg.y /= (float)cNum;
  magneticAvg.z /= (float)cNum;
  
  ++index;
  
  if (cNum <= index) {
    index = 0;
  }

  ++counter;

  if (10 == counter) {

    Serial.print("Acceleration values  : X = ");
    Serial.print(accelerationAvg.x, 2);
    Serial.print(" Y = ");
    Serial.print(accelerationAvg.y, 2);
    Serial.print(" Z = ");
    Serial.println(accelerationAvg.z, 2);
    
    Serial.print("Magnetic values      : X = ");
    Serial.print(magneticAvg.x, 2);
    Serial.print(" Y = ");
    Serial.print(magneticAvg.y, 2);
    Serial.print(" Z = ");
    Serial.println(magneticAvg.z, 2);
    
    Serial.print("Gyroscope values     : X = ");
    Serial.print(gyroAvg.x, 2);
    Serial.print(" Y = ");
    Serial.print(gyroAvg.y, 2);
    Serial.print(" Z = ");
    Serial.println(gyroAvg.z, 2);
  
    Serial.println("");

    counter = 0;
  }
  
  return;
}

/**
 * Get absolute acceleration in m/s^2.
 *
 * @param[in] acceleration  3D acceleration values in m/s^2
 *
 * @return Absolute acceleration value in m/s^2.
 */
static float getAbsAcceleration(sensors_vec_t* acceleration) {

  float absAccel  = 0.0f;

  if (NULL != acceleration) {
    
    absAccel = sqrtf(
      acceleration->x * acceleration->x +
      acceleration->y * acceleration->y +
      acceleration->z * acceleration->z);
  }
  
  return absAccel;
}

/**
 * Get acceleration vector in m/s^2.
 *
 * @param[out]  acceleration    3D acceleration vector in m/s^2
 */
static void getAcceleration(sensors_vec_t* acceleration) {

  sensors_event_t accelerationEvent;
  
  if (NULL == acceleration) {
    return;
  }

  lsm.getEvent(&accelerationEvent, NULL, NULL, NULL);
 
  *acceleration = accelerationEvent.acceleration;

#if (3 != NEOPIXELLAMP_DEBUG)
  
  // Offset compensation
  acceleration->x -= NEOPIXELLAMP_ACCELERATION_OFFSET_X;
  acceleration->y -= NEOPIXELLAMP_ACCELERATION_OFFSET_Y;
  acceleration->z -= NEOPIXELLAMP_ACCELERATION_OFFSET_Z;

#endif
  
  return;
}

/**
 * Get magnetic vector in gauss.
 *
 * @param[out]  acceleration    3D magnetic vector in gauss
 */
static void getMagnetic(sensors_vec_t* magnetic) {

  sensors_event_t magneticEvent;
  
  if (NULL == magnetic) {
    return;
  }

  lsm.getEvent(NULL, &magneticEvent, NULL, NULL);

  *magnetic = magneticEvent.magnetic;

#if (3 != NEOPIXELLAMP_DEBUG)
  
  // Offset compensation
  magnetic->x -= NEOPIXELLAMP_MAGNETIC_OFFSET_X;
  magnetic->y -= NEOPIXELLAMP_MAGNETIC_OFFSET_Y;
  magnetic->z -= NEOPIXELLAMP_MAGNETIC_OFFSET_Z;

#endif
  
  return;
}

/**
 * Get your heading, using Earth's magnetic field.
 * It only works if the sensor is flat (z-axis normal to Earth).
 * Additionally, you may need to add or subtract a declination
 * angle to get the heading normalized to your location.
 * See: http://www.ngdc.noaa.gov/geomag/declination.shtml
 *
 * @return Heading in degree
 */
static float getHeading(void) {

  sensors_vec_t magnetic;
  float         heading   = 0.0f;
  
  getMagnetic(&magnetic);
 
  if (0.0f < magnetic.y)
  {
    heading = 90.0f - (atan(magnetic.x / magnetic.y) * (180.0f / PI));
  }
  else if (0.0f > magnetic.y)
  {
    heading = -1.0f * (atan(magnetic.x / magnetic.y) * (180.0f / PI));
  }
  else // magnetic.y = 0.0f
  {
    if (0.0f > magnetic.x) {
      heading = 180.0f;
    }
    else {
      heading = 0.0f;
    }
  }

  return heading;
}

/**
 * Get gyroscope vector in degree/s.
 *
 * @param[out]  gyro    3D gyroscope vector in degree/s
 */
static void getGyro(sensors_vec_t* gyro) {

  sensors_event_t gyroEvent;
  
  if (NULL == gyro) {
    return;
  }

  lsm.getEvent(NULL, NULL, &gyroEvent, NULL);

  *gyro = gyroEvent.gyro;

#if (3 != NEOPIXELLAMP_DEBUG)
  
  // Offset compensation
  gyro->x -= NEOPIXELLAMP_GYRO_OFFSET_X;
  gyro->y -= NEOPIXELLAMP_GYRO_OFFSET_Y;
  gyro->z -= NEOPIXELLAMP_GYRO_OFFSET_Z;

#endif
  
  return;
}

/**
 * Get temperature in degree celsius.
 *
 * @return Temperature in degree celsius.
 */
static float getTemperatureC(void) {

  sensors_event_t tempEvent;

  lsm.getEvent(NULL, NULL, NULL, &tempEvent);

  return tempEvent.temperature;
}

/**
 * Get gyroscope average values in degree/s.
 *
 * @param[out]  gyroAvg 3D average gyroscope values in degree/s
 */
static void getGyroAvg(sensors_vec_t* gyroAvg) {

  const uint8_t         cNum  = 10;
  static sensors_vec_t  gyro[cNum];
  static uint8_t        index = 0;
  static bool           init  = true;
  uint8_t               run   = 0;

  getGyro(&gyro[index]);

  ++index;
  if (cNum <= index) {
    index = 0;
  }

  if (true == init) {
    
    for(run = 1; run < cNum; ++run) {
      gyro[run].x = gyro[0].x;
      gyro[run].y = gyro[0].y;
      gyro[run].z = gyro[0].z;
    }
  }

  if (NULL != gyroAvg) {

    gyroAvg->x = 0.0f;
    gyroAvg->y = 0.0f;
    gyroAvg->z = 0.0f;

    for(run = 0; run < cNum; ++run) {
      gyroAvg->x += gyro[run].x;
      gyroAvg->y += gyro[run].y;
      gyroAvg->z += gyro[run].z;
    }

    gyroAvg->x /= cNum;
    gyroAvg->y /= cNum;
    gyroAvg->z /= cNum;
  }

  return;
}

/**
 * A red LED splash.
 *
 * @param[in]   num Number of splashes
 */
static void redSplash(uint8_t num) {

  uint16_t  run           = 0;
  uint8_t   count         = 0;
  uint8_t   oldBrightness = neoPixel.getBrightness(); // Store current brightness

  neoPixel.clear();
  neoPixel.show();
  delay(200);

  neoPixel.setBrightness(NEOPIXELLAMP_MAX_BRIGHTNESS / 2);

  for(count = 0; count < num; ++count) {

    // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
    for(run = 0; run < neoPixel.numPixels(); ++run) {
  
      // neoPixel.Color takes RGB values, from 0,0,0 up to 255,255,255
      neoPixel.setPixelColor(run, neoPixel.Color(255, 0, 0));
  
      // This sends the updated pixel color to the hardware.
      neoPixel.show();  
    }
  
    delay(500);
  
    neoPixel.clear();
    neoPixel.show();
    delay(500);

  }

  // Restore brightness
  neoPixel.setBrightness(oldBrightness);

  return;
}

/**
 * Input a value 0 to 255 to get a color value.
 * The colours are a transition r - g - b - back to r.
 *
 * @param[in]   wheelPos    Color wheel position (0-255)
 *
 * @return Color
 */
static uint32_t colorWheel(uint8_t wheelPos) {

  uint32_t  color = 0;
  
  wheelPos = 255 - wheelPos;
  
  if (wheelPos < 85) {
    color = neoPixel.Color(255 - wheelPos * 3, 0, wheelPos * 3);
  }
  else if (wheelPos < 170) {
    wheelPos -= 85;
    color = neoPixel.Color(0, wheelPos * 3, 255 - wheelPos * 3);
  }
  else {
    wheelPos -= 170;
    color = neoPixel.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
  }
  
  return color;
}

/**
 * Watchdog timer interrupt, used to wake up from sleep mode periodically.
 */
ISR(WDT_vect)
{
  // Disable watchdog
  {
    cli();

    wdt_reset();
    
    // Prepare watchdog for timer interrupt without reset
    MCUSR &= ~(1 << WDRF);  // Clear WDRF, which overwrites WDE

    // In the same operation, write a logic one to the Watchdog
    // change enable bit (WDCE) and WDE. A logic one must be
    // written to WDE regardless of the previous value of the WDE bit.
    WDTCSR |= (1 << WDCE) | (1 << WDE);

    // Within the next four clock cycles, write the WDE and Watchdog
    // prescaler bits (WDP) as desired, but with the WDCE bit cleared.
    // This must be done in one operation.
    // Watchdog timer: disabled
    WDTCSR = 0;

    sei();
  }
}

/**
 * Disable activated watchdog right after reset.
 * http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 */
void get_mcusr(void) \
__attribute__((naked)) \
__attribute__((section(".init3")));
 
/** Copy of the MCUSR register. */
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

