/** BOB=========================================================================
 * @file SonarWithWifiModemHERRICK.ino
 * @brief Working Maxbotix Sonar, based on simple logging example from
 * Anthony Aufdenkampe with Xbee wifi modem code and MonitorMyWatershed (MMW) logging.
 * If you do not want to log your data to MMW, you can comment out a lot of code below
 * Most of this is the code that controls the wifi modem
 *
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @author Anthony Aufdenkampe <aaufdenkampe@limno.com>
 * @author Tom Hickson <tahickson@stthomas.edu>
 * @copyright (c) 2017-2022 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 *
 * Build Environment: Visual Studios Code with PlatformIO
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 * Firmware Library: ModularSensors v0.34.0, released 2023-03-16
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */
// THIS CODE IS CURRENTLY WORKING ON THE MAYFLY, V. 0.5!!!
// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
#include <EnableInterrupt.h>

// Include the main header for ModularSensors
#include <ModularSensors.h>

// For the OLED display, from switchdoclabs/SDL_Arduino_SSD1306
// #include <AMAdafruit_GFX.h>      // For the OLED display, from switchdoclabs/SDL_Arduino_SSD1306
// #include <SDL_Arduino_SSD1306.h> // For the OLED display

/** End [includes] */

// ==========================================================================
//  Creating Additional Serial Ports (mainly for MMW/xBee coms)
// ==========================================================================

// NeoSWSerial (https://github.com/SRGDamia1/NeoSWSerial) is the best software
// serial that can be used on any pin supporting interrupts.
// You can use as many instances of NeoSWSerial as you need.
// Not all AVR boards are supported by NeoSWSerial.
/** Start [neoswserial] */
#include <NeoSWSerial.h>         // for the stream communication
const int8_t neoSSerial1Rx = 7;  // data in pin
const int8_t neoSSerial1Tx = -1; // data out pin
NeoSWSerial neoSSerial1(neoSSerial1Rx, neoSSerial1Tx);
// To use NeoSWSerial in this library, we define a function to receive data
// This is just a short-cut for later
void neoSSerial1ISR()
{
    NeoSWSerial::rxISR(*portInputRegister(digitalPinToPort(neoSSerial1Rx)));
}
/** End [neoswserial] */

// ==========================================================================
//  Assigning Serial Port Functionality
// ==========================================================================
/** Start [assign_ports_hw] */
// If there are additional hardware Serial ports possible - use them!

// We give the modem first priority and assign it to hardware serial
// All of the supported processors have a hardware port available named Serial1
#define modemSerial Serial1

/** End [assign_ports_hw] */

/** Start [assign_ports_sw] */

// The Maxbotix sonar is the only sensor that communicates over a serial port
// but does not use modbus
// Since the Maxbotix only needs one-way communication and sends a simple text
// string repeatedly, almost any software serial port will do for it.
#define sonarSerial neoSSerial1 // For Neo software serial

/** End [assign_ports_sw] */

// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char *sketchName = "Sump_Sonar_Code.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "Sump_Sonar";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 1;
// Your logger's timezone.
const int8_t timeZone = -6; // Central Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 115200; // Baud rate for debugging
const int8_t greenLED = 8;         // Pin for the green LED
const int8_t redLED = 9;           // Pin for the red LED
const int8_t buttonPin = 21;       // Pin for debugging mode (ie, button pin)
const int8_t wakePin = A7;         // MCU interrupt/alarm pin to wake from sleep
// Mayfly 0.x D31 = A7
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin = -1;   // MCU SD card power pin
const int8_t sdCardSSPin = 12;    // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22; // MCU pin controlling main sensor power
/** End [logging_options] */

// ==========================================================================
//  Wifi/Cellular Modem Options (can comment out if not using)
// ==========================================================================
/** Start [digi_xbee_wifi] */
// For the Digi Wifi XBee (S6B)
#include <modems/DigiXBeeWifi.h>

// NOTE: Extra hardware and software serial ports are created in the "Settings
// for Additional Serial Ports" section
const int32_t modemBaud = 9600; // All XBee's use 9600 by default

// Modem Pins - Describe the physical pin connection of your modem to your board
// NOTE:  Use -1 for pins that do not apply
// The pin numbers here are for a Digi XBee direcly connected to a Mayfly v0.5 from
// https://envirodiy.github.io/ModularSensors/group__modem__digi.html#modem_digi_raw_pins
const int8_t modemVccPin = -1;    // MCU pin controlling modem power
const int8_t modemStatusPin = 19; // MCU pin used to read modem status

// NOTE:  If possible, use the `STATUS/SLEEP_not` (XBee pin 13) for status, but
// the CTS pin can also be used if necessary
const bool useCTSforStatus = true; // Flag to use the XBee `CTS` pin for status
const int8_t modemResetPin = -1;   // MCU pin connected to modem reset pin
const int8_t modemSleepRqPin = 23; // MCU pin used for modem sleep/wake request
const int8_t modemLEDPin = redLED; // MCU pin connected an LED to show modem status

// Network connection information for UST IoT network in OSS/OWS
const char *wifiId = "YOUR_ACCESS_POINT_NAME";      // WiFi access point name
const char *wifiPwd = "YOUR_ACCESS_POINT_PASSWORD"; // WiFi password (WPA2)

// Create the modem object
DigiXBeeWifi modemXBWF(&modemSerial, modemVccPin, modemStatusPin,
                       useCTSforStatus, modemResetPin, modemSleepRqPin, wifiId,
                       wifiPwd);
// Create an extra reference to the modem by a generic name
DigiXBeeWifi modem = modemXBWF;
/** End [digi_xbee_wifi] */

// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_stats] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char *mcuBoardVersion = "v0.5b"; // CRITICAL: CHANGE TO MATCH VERSION OF YOUR MAYFLY BOARD
ProcessorStats mcuBoard(mcuBoardVersion);

// Create sample number, battery voltage, and free RAM variable pointers for the
// processor
// Variable* mcuBoardBatt = new ProcessorStats_Battery(
//     &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
// Variable* mcuBoardAvailableRAM = new ProcessorStats_FreeRam(
//     &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
// Variable* mcuBoardSampNo = new ProcessorStats_SampleNumber(
//     &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
/** End [processor_stats] */

// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [maxim_ds3231] */
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);

// Create a temperature variable pointer for the DS3231
// Variable* ds3231Temp =
//     new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab");
/** End [maxim_ds3231] */

// ==========================================================================
//    Settings for Additional Sensors
// ==========================================================================

// ==========================================================================
//  Maxbotix HRXL Ultrasonic Range Finder
// ==========================================================================
/** Start [max_botix_sonar] */
#include <sensors/MaxBotixSonar.h>

// A Maxbotix sonar with the trigger pin disconnect CANNOT share the serial port
// A Maxbotix sonar using the trigger may be able to share but YMMV

// NOTE: Extra hardware and software serial ports are created in the "Settings
// for Additional Serial Ports" section

// NOTE: Use -1 for any pins that don't apply or aren't being used.
const int8_t SonarPower = sensorPowerPin; // Excite (power) pin
const int8_t Sonar1Trigger = -1;          // Trigger pin
// Trigger should be a *unique* negative number if unconnected
const uint8_t sonar1NumberReadings = 3; // The number of readings to average

// Create a MaxBotix Sonar sensor object
MaxBotixSonar sonar1(sonarSerial, SonarPower, Sonar1Trigger,
                     sonar1NumberReadings);

// Create an ultrasonic range variable pointer
Variable *sonar1Range =
    new MaxBotixSonar_Range(&sonar1, "12345678-abcd-1234-ef00-1234567890ab"); // CAN LEAVE AS IS. SSID OF SONAR IF SENDING TO MMW
/** End [max_botix_sonar] */

// ==========================================================================
//  Calculated Variable[s]
// ==========================================================================
/** Start [calculated_variables] */
// Create the function to give your calculated result.
// The function should take no input (void) and return a float.
// You can use any named variable pointers to access values by way of
// variable->getValue()
// Create a function in order to have a UUID for sonar distance
float sonarDistanceMMW(void)
{
    float sonarDistanceMeasured = sonar1Range->getValue();
    return sonarDistanceMeasured;
}
const uint8_t sonarDistanceMMWVarResolution = 3;
const char *sonarDistanceMMWVarName = "distance";
const char *sonarDistanceMMWVarUnit = "Millimeter";
const char *sonarDistanceMMWVarCode = "sonarRangeMMW";
const char *sonarDistanceMMWVarUUID = "12345678-abcd-1234-ef00-1234567890ab"; // SUMP UUID FOR MMW
Variable *sonarDistanceMMWdone = new Variable(sonarDistanceMMW, sonarDistanceMMWVarResolution, sonarDistanceMMWVarName,
                                              sonarDistanceMMWVarUnit, sonarDistanceMMWVarCode, sonarDistanceMMWVarUUID);

// Create the function to calculate water level / gage height variable
float calculateSonarGageHeight(void)
{
    float sonarGageHeight = -9999;    // Always safest to start with a bad value
    float sonarGageHeight_mm = -9999; // Always safest to start with a bad value
    // float minimumRange = 300;    // in millimeters; not used here
    float maximumRange = 2344; // in millimeters. 99th percentile of max distances measured over a week-long interval. Should be determined after running sensor for a while
    // to see what the maximum measurement is for each sump.
    // Relative to reference stage, where add up measured lake stage/elevation plus height of sensor above water
    // float sonarDistanceToZeroStage = 844 * 304.8; // Constant, in millimeters, where 304.8 mm = 1.00 ft
    // Where 844' is approximate elevation of the water level, in ft. above sea level.
    float sonarDistanceMeasured = sonar1Range->getValue();
    if (sonarDistanceMeasured != -9999)
    { // make sure all inputs are good
        sonarGageHeight_mm = maximumRange - sonarDistanceMeasured;
        sonarGageHeight = sonarGageHeight_mm / 304.8; // to convert to feet, divide by 304.8, or divide by 1 to remain in mm.
    }
    return sonarGageHeight;
}

// Properties of the calculated water level / gage height variable
// The number of digits after the decimal place
const uint8_t sonarGageHeightVarResolution = 3;
// This must be a value from http://vocabulary.odm2.org/variablename/
const char *sonarGageHeightVarName = "gageHeight";
// This must be a value from http://vocabulary.odm2.org/units/
const char *sonarGageHeightVarUnit = "Foot";
// A short code for the variable
const char *sonarGageHeightVarCode = "SonarGageHeight";
// The (optional) universallly unique identifier
const char *sonarGageHeightVarUUID = "12345678-abcd-1234-ef00-1234567890ab"; // Sump gage height UUID IF USING MMW

// Create the calculated gage height variable objects and return a variable pointer to it
Variable *calculatedSonarGageHeight = new Variable(
    calculateSonarGageHeight, sonarGageHeightVarResolution, sonarGageHeightVarName,
    sonarGageHeightVarUnit, sonarGageHeightVarCode, sonarGageHeightVarUUID);

// Create the function to calculate water volume in cubic feet
float calculateSonarVolume(void)
{
    float sonarVolume = 3.14159 * pow(2, 2) * (calculatedSonarGageHeight->getValue());
    // In feet. Our sumps have diameter = 4 feet
    // Relative to reference stage, where add up measured lake stage/elevation plus height of sensor above water

    return sonarVolume;
}

// Properties of the calculated water volume variable
// The number of digits after the decimal place
const uint8_t sonarVolumeResolution = 4;
// This must be a value from http://vocabulary.odm2.org/variablename/
const char *sonarVolumeVarName = "Volume (ft^3)";
// This must be a value from http://vocabulary.odm2.org/units/
const char *sonarVolumeVarUnit = "ft^3";
// A short code for the variable
const char *sonarVolumeVarCode = "SonarVolume";
// The (optional) universallly unique identifier
const char *sonarVolumeUUID = "12345678-abcd-1234-ef00-1234567890ab"; // Sump Volume UUID IF USING MMW
// Create the calculated volume of water in sump and return a variable pointer to it
Variable *calculatedSonarVolume = new Variable(
    calculateSonarVolume, sonarVolumeResolution, sonarVolumeVarName,
    sonarVolumeVarUnit, sonarVolumeVarCode, sonarVolumeUUID);

/** End [calculated_variables] */

// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
//  NOTE:  There are three different ways of creating the same variable array
//         and filling it with variables. Here we blend methods.
// ==========================================================================
/** Start [variables_create_in_array_pre_named] */
Variable *variableList[] = {
    new ProcessorStats_SampleNumber(&mcuBoard,
                                    "12345678-abcd-1234-ef00-1234567890ab"),
    new ProcessorStats_FreeRam(&mcuBoard,
                               "12345678-abcd-1234-ef00-1234567890ab"),
    new ProcessorStats_Battery(&mcuBoard,
                               "12345678-abcd-1234-ef00-1234567890ab"),
    // new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab"),
    // Add variable using method 3, pre_named
    sonar1Range,
    calculatedSonarGageHeight,
    sonarDistanceMMWdone,
    //  ... Add more variables as needed!
    new Modem_RSSI(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
    new Modem_SignalPercent(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
    // new Modem_Temp(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
    // new Variable(calculateVariableValue, calculatedVarResolution,
    //              calculatedVarName, calculatedVarUnit, calculatedVarCode,
    //              calculatedVarUUID),
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);
// Create the VariableArray object
VariableArray varArray(variableCount, variableList);
/** End [variables_create_in_array_pre_named] */

// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);
/** End [loggers] */

// ==========================================================================
//  Creating Data Publisher[s]
// ==========================================================================
/** Start [publishers] */
// A Publisher to Monitor My Watershed / EnviroDIY Data Sharing Portal
// Device registration and sampling feature information can be obtained after
// registration at https://monitormywatershed.org or https://data.envirodiy.org
const char *registrationToken =
    "12345678-abcd-1234-ef00-1234567890ab"; // Device registration token for Sump IF SENDING TO MMW
const char *samplingFeature =
    "12345678-abcd-1234-ef00-1234567890ab"; // Sampling feature UUID for Sump IF SENDING TO MMW

// Create a data publisher for the Monitor My Watershed/EnviroDIY POST endpoint
#include <publishers/EnviroDIYPublisher.h>
EnviroDIYPublisher EnviroDIYPOST(dataLogger, &modem.gsmClient,
                                 registrationToken, samplingFeature);
/** End [publishers] */

// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75)
{
    for (uint8_t i = 0; i < numFlash; i++)
    {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}

// Uses the processor sensor object to read the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage()
{
    if (mcuBoard.sensorValues[0] == -9999)
        mcuBoard.update();
    return mcuBoard.sensorValues[0];
}
/** End [working_functions] */

// Set up the OLED display
SDL_Arduino_SSD1306 display(-1); // using I2C and not bothering with a reset pin

// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
void setup()
{

    /** Start [setup_prints] */
    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Start the OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    // Print a start-up note to the OLED display
    display.println(sketchName);
    display.println(LoggerID);
    display.display();

    // Print a start-up note to the first serial port
    Serial.print(F("\n\nNow running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);
    Serial.print(F("TinyGSM Library version "));
    Serial.println(TINYGSM_VERSION);
    Serial.println();

/** Start [setup_softserial] */
// Allow interrupts for software serial
#if defined SoftwareSerial_ExtInts_h
    enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt,
                    CHANGE);
#endif
#if defined NeoSWSerial_h
    enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
#endif
    /** End [setup_softserial] */

    /** Start [setup_serial_begins] */

    // Start the SoftwareSerial stream for the sonar; it will always be at 9600
    // baud
    sonarSerial.begin(9600);
    /** End [setup_serial_begins] */

    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);

    /** Start [setup_flashing_led] */
    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();
    /** End [setup_flashing_led] */

    /** Start [setup_logger] */
    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Set information pins
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);

    // Begin the logger
    dataLogger.begin();
    /** End [setup_logger] */

    /** Start [setup_sensors] */
    Serial.println(F("Setting up sensors..."));
    varArray.setupSensors();
    /** End [setup_sensors] */

    /** Start [setup_file] */
    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    dataLogger.createLogFile(true); // true = write a new header
    /** End [setup_file] */

    /** Start [setup_sleep] */
    // Call the processor sleep
    Serial.println(F("Putting processor to sleep\n"));
    dataLogger.systemSleep();
    /** End [setup_sleep] */
}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [complex_loop] */
// Use this long loop when you want to do something special
// Because of the way alarms work on the RTC, it will wake the processor and
// start the loop every minute exactly on the minute.
// The processor may also be woken up by another interrupt or level change on a
// pin - from a button or some other input.
// The "if" statements in the loop determine what will happen - whether the
// sensors update, testing mode starts, or it goes back to sleep.
void loop()
{
    // Reset the watchdog
    dataLogger.watchDogTimer.resetWatchDog();

    // Assuming we were woken up by the clock, check if the current time is an
    // even interval of the logging interval
    // We're only doing anything at all if the battery is above 3.4V
    if (dataLogger.checkInterval() && getBatteryVoltage() > 3.4)
    {
        // Flag to notify that we're in already awake and logging a point
        Logger::isLoggingNow = true;
        dataLogger.watchDogTimer.resetWatchDog();

        // Print a line to show new reading
        Serial.println(F("------------------------------------------"));
        // Turn on the LED to show we're taking a reading
        dataLogger.alertOn();
        // Power up the SD Card, but skip any waits after power up
        dataLogger.turnOnSDcard(false);
        dataLogger.watchDogTimer.resetWatchDog();
        Serial.print("\n Range (mm): ");
        Serial.println(sonar1Range->getValueString());
        Serial.print("Gage Height (ft): ");
        Serial.println(calculatedSonarGageHeight->getValueString());
        display.print("Sonar Range (mm): ");
        display.println(sonar1Range->getValueString());
        display.print("Gage Height (ft): ");
        display.println(calculatedSonarGageHeight->getValueString());
        display.display();
        // Turn on the modem to let it start searching for the network
        // Only turn the modem on if the battery at the last interval was high
        // enough
        // NOTE:  if the modemPowerUp function is not run before the
        // completeUpdate
        // function is run, the modem will not be powered and will not
        // return a signal strength reading.
        if (getBatteryVoltage() > 3.6)
            modem.modemPowerUp();

        // Do a complete update on the variable array.
        // This this includes powering all of the sensors, getting updated
        // values, and turing them back off.
        // NOTE:  The wake function for each sensor should force sensor setup
        // to run if the sensor was not previously set up.
        varArray.completeUpdate();

        dataLogger.watchDogTimer.resetWatchDog();

        // Create a csv data record and save it to the log file
        dataLogger.logToSD();
        dataLogger.watchDogTimer.resetWatchDog();

        // Connect to the network
        // Again, we're only doing this if the battery is doing well
        if (getBatteryVoltage() > 3.55)
        {
            dataLogger.watchDogTimer.resetWatchDog();
            if (modem.connectInternet())
            {
                dataLogger.watchDogTimer.resetWatchDog();
                // Publish data to remotes
                Serial.println(F("Modem connected to internet."));
                dataLogger.publishDataToRemotes();

                // Sync the clock at noon
                dataLogger.watchDogTimer.resetWatchDog();
                if (Logger::markedLocalEpochTime != 0 &&
                    Logger::markedLocalEpochTime % 86400 == 43200)
                {
                    Serial.println(F("Running a daily clock sync..."));
                    dataLogger.setRTClock(modem.getNISTTime());
                    dataLogger.watchDogTimer.resetWatchDog();
                    modem.updateModemMetadata();
                    dataLogger.watchDogTimer.resetWatchDog();
                }

                // Disconnect from the network
                modem.disconnectInternet();
                dataLogger.watchDogTimer.resetWatchDog();
            }
            // Turn the modem off
            modem.modemSleepPowerDown();
            dataLogger.watchDogTimer.resetWatchDog();
        }

        // Cut power from the SD card - without additional housekeeping wait
        dataLogger.turnOffSDcard(false);
        dataLogger.watchDogTimer.resetWatchDog();
        // Turn off the LED
        dataLogger.alertOff();
        // Print a line to show reading ended
        Serial.println(F("------------------------------------------\n"));

        // Unset flag
        Logger::isLoggingNow = false;
    }

    // Check if it was instead the testing interrupt that woke us up
    if (Logger::startTesting)
    {
        // Testing mode with output to serial monitor
        // Comment this line out if you want to use the OLED display
        // dataLogger.testingMode();

        // Testing mode using OLED Display
        // Flag to notify that we're in testing mode
        Logger::isTestingNow = true;
        // Unset the startTesting flag
        Logger::startTesting = false;
        PRINTOUT(F("------------------------------------------"));
        PRINTOUT(F("Entering sensor testing mode with OLED display"));
        delay(100); // This seems to prevent crashes, no clue why ....

        display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false);

        // Power up all of the sensors
        varArray.sensorsPowerUp();
        // Wake up all of the sensors
        varArray.sensorsWake();

        // Loop through 10 measurement cycles of 3 seconds each
        // before exiting testing mode
        for (uint8_t i = 0; i <= 10; i++)
        {

            // Turn on the LED to show we're taking a reading
            digitalWrite(greenLED, HIGH);

            dataLogger.watchDogTimer.resetWatchDog();
            // Update the values from all attached sensors
            // NOTE:  NOT using complete update because we want the sensors to be
            // left on between iterations in testing mode.
            varArray.updateAllSensors();

            // Print the sensor result to serial port
            Serial.print("Number ");
            Serial.println(i);
            Serial.print("Range (mm): ");
            Serial.println(sonar1Range->getValueString());
            Serial.print("Gage Height (ft): ");
            Serial.println(calculatedSonarGageHeight->getValueString());

            // Reset the OLED display
            display.clearDisplay();
            display.setCursor(0, 0);
            display.setTextSize(2);

            // Print the sensor result to OLED display
            display.print("Number ");
            display.println(i);
            display.print("Sonar Range (mm): ");
            display.println(sonar1Range->getValueString());
            display.print("Gage Height (ft): ");
            display.println(calculatedSonarGageHeight->getValueString());
            display.display();

            // Add a delay?
            // delay(3000);
        }

        // Put sensors to sleep
        varArray.sensorsSleep();
        varArray.sensorsPowerDown();

        PRINTOUT(F("Exiting testing mode"));
        PRINTOUT(F("------------------------------------------"));
        dataLogger.watchDogTimer.resetWatchDog();

        // Unset testing mode flag
        Logger::isTestingNow = false;

        // Sleep
        dataLogger.systemSleep();
    }

    // Call the processor sleep
    dataLogger.systemSleep();
}
/** End [complex_loop] */
