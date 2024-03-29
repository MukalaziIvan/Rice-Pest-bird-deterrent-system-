/*/*
  Project Title: A Robust Distress Sound-Based Bird Deterrent System For Rice Farming In Uganda
  Version: 1.0
  Date: 27/03/2024
  
  Description:
  This Arduino project detects bird motion or vocalization using a PIR sensor and a microphone. It employs Fast Fourier Transform (FFT) analysis to identify the type of bird based on its vocalization frequency. If a known bird is detected, distress sounds specific/suitable to that bird are played to deter it from the area.

  Components:
  - Arduino Mega 2560
  - PIR Sensor
  - Microphone
  - DFPlayer Mini MP3 Module
  - LCD Display
  
  Libraries Used:
  - arduinoFFT
  - SoftwareSerial
  - DFRobotDFPlayerMini
  - LiquidCrystal_I2C
  
  Author: Mukalazi Ivan & Naikazi Fauzia Majid 
  Affiliation: Makerere University
  Supervisor: Mr Frank Ssemakula
  Contact: mukalaziivan4@gmail.com
  
  License: This code is licensed under the GNU General Public License v3.0.
  See https://www.gnu.org/licenses/gpl-3.0.en.html for details.
*/
*/



//Including neccessary Libraries to be used.
#include <SPI.h>
#include <SD.h>
#include "arduinoFFT.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <LiquidCrystal_I2C.h> 
#include <Wire.h>


LiquidCrystal_I2C lcd(0x27, 16, 2); //Initializing the LCD object with the address 0x27, 16 columns, and 2 rows


// Class definition for the MP3 player
class MP3Player {
private:
    SoftwareSerial *mySoftwareSerial;// Pointer to a SoftwareSerial object for communication with the MP3 player
    void waitPlayIsTerminated();
    int p_RX;
    int p_TX;

public:
    DFRobotDFPlayerMini player;// Instance of the DFRobotDFPlayerMini class for controlling the MP3 player
    MP3Player(int RX, int TX); // Constructor for the MP3Player class
    ~MP3Player();
    void initialize();
    void playTrackNumber(int trackNumber, int volume, boolean waitPlayTerminated = true);
};
// Constructor for the MP3Player class
MP3Player::MP3Player(int RX, int TX) {
    p_TX = TX;
    p_RX = RX;
}
// Destructor for the MP3Player class
MP3Player::~MP3Player() {
}

// Function to initialize the MP3 player
void MP3Player::initialize() {
    mySoftwareSerial = new SoftwareSerial(p_RX, p_TX);


    mySoftwareSerial->begin(9600);// Initialize serial communication for MP3 player
    lcd.init(); // Initializing the LCD
    lcd.backlight();
    lcd.print("Initializing...");
    delay(2000);
    
    
    if (!player.begin(*mySoftwareSerial, true, false)) {

        lcd.clear();
        lcd.print("MP3 Init Error!");
        while (true)
            ;
    }
    player.volume(28);
    lcd.clear();
    lcd.print("G0-Bird-Welcome..");
    delay(2000);
}


void MP3Player::playTrackNumber(int trackNumber, int volume, boolean waitPlayTerminated) {
    player.volume(volume);
    player.play(trackNumber);
    if (waitPlayTerminated) {
        waitPlayIsTerminated();
    }
}

void MP3Player::waitPlayIsTerminated() {
    while (!player.available()) {
    }
}

#define BIRDX "BirdX"
#define WEAVER "Weaver"
#define RED_BILLED "Red Billed"
#define OTHERS "Others"

const int SAMPLES = 256; // SAMPLES
const int SAMPLING_FREQUENCY = 9000; // SAMPLING_FREQUENCY 

arduinoFFT FFT = arduinoFFT(); // Define FFT object

double vReal[SAMPLES]; // Creating vector to hold real values
double vImag[SAMPLES]; // Creating vector to hold imaginary values

const int RBQ_MIN = 2301;
const int RBQ_MAX = 4100;
const int VW_MIN = 4101;
const int VW_MAX = 5100;

// Define PIR pin
const int pirPin = 8; // PIR sensor is connected to digital pin 8

// Define microphone pin
const int micPin = A0; // The microphone is connected to analog pin A0

// MP3 player object
MP3Player mp3(10, 11); // The MP3 player is connected to RX pin 10 and TX pin 11

// Function prototypes
int detectMotion();
String identifyBird(double peakFrequency);
int selectRandomSound(String birdType);
void playDistressSound(int soundIndex);

void setup() {
  
  //Initialize MP3 player
  mp3.initialize();
}

void loop() {
    String birdType = detectBird(); // Checking for bird motion or vocalization
  
    if (birdType != OTHERS) {
        int soundIndex = selectRandomSound(birdType);
        playDistressSound(soundIndex);

        lcd.setCursor(0, 1); // Setting cursor to the second row
        lcd.print(F("PlayingSound:"));// Selecting Suitable sound number
        lcd.print(soundIndex);

        delay(3000); //Delay to control loop timing
        lcd.clear(); // Clear the LCD after playing sound
       
    }

    delay(2000); // Delay to control loop timing
}

String detectBird() {
    int motionDetected = detectMotion(); // Checking for motion

    if (motionDetected) {
        lcd.clear();
        lcd.setCursor(0, 0); // Set cursor to the  row 1
        lcd.print(F("Bird X"));
        return BIRDX; // Return  BirdX type if only motion is detected
    } else {
        //Read microphone input
        double peakFrequency = getMicrophoneFrequency(); // Obtain peak frequency from microphone input
        return identifyBird(peakFrequency); // Returns either WEAVER or RED_BILLED or OTHERS based on FFT analysis
    }
}

double getMicrophoneFrequency() {
    unsigned int samplingPeriod;
    unsigned long microSeconds;

    double peakFrequency;

    samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY)); // Period in microseconds

    /* Sample SAMPLES times */
    for (int i = 0; i < SAMPLES; i++) {
        microSeconds = micros();    // Returns the number of microseconds since the Arduino board began running the current script.
        vReal[i] = analogRead(micPin); // Reads the value from analog pin A0, quantizes it and saves it as a real term.
        vImag[i] = 0; // Makes imaginary term 0 always

        /* Remaining wait time between samples if necessary */
        while (micros() < (microSeconds + samplingPeriod)) {
            // Do nothing
        }
    }

    /* Performing FFT on samples */
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    /* Finding peak frequency */
    peakFrequency = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

    return peakFrequency;
}

String identifyBird(double peakFrequency) {
    if (peakFrequency >= VW_MIN && peakFrequency <= VW_MAX) {
        lcd.clear();
        lcd.setCursor(0, 0); // Set cursor to  row 1
        lcd.print(F("Weaver Bird"));
        //lcd.setCursor(0, 1); // Set cursor to  row 2
        //lcd.print(peakFrequency);
       
        return WEAVER;
    } else if (peakFrequency >= RBQ_MIN && peakFrequency <= RBQ_MAX) {
        lcd.clear();
        lcd.setCursor(0, 0); // Set cursor to row 1
        lcd.print("Red-Billed Bird");
        //lcd.setCursor(0, 1); // Set cursor to  row 2
        //lcd.print(peakFrequency);
        
        return RED_BILLED;
    } else {
      lcd.clear();
      lcd.setCursor(0, 0); // Set cursor to  row 1
        lcd.print("No Bird");
        return OTHERS;
    }
}

void playDistressSound(int soundIndex) {
    mp3.playTrackNumber(soundIndex, 29, true); // Play distress sound at volume level 29 and wait for play to be terminated
    delay(5000);//Delay to play another sound on calling,to allow the former to do it's job first.
}

int selectRandomSound(String birdType) {
    int randomIndex;

    if (birdType == WEAVER) {
        randomIndex = random(5, 10); // Random number between 5 and 10 for village weaver distress sounds
    } else if (birdType == RED_BILLED) {
        randomIndex = random(1, 7); // Random number between 1 and 7 for red-billed quelea distress sounds
    } else if (birdType == BIRDX) {
        randomIndex = random(1, 10); // Random number between 1 and 10 for unknown birds
    }
    return randomIndex; // Return the randomly selected sound index
}

int detectMotion() {
    if (digitalRead(pirPin) == HIGH) {
        return 1; // Motion detected
    } else {
        return 0; // No motion detected
    }
}

