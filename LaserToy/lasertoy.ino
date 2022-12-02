#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <TimeLord.h>

////////////////////////////////////////
// GLOBAL CONSTANTS
////////////////////////////////////////
const int servoFreq = 50; // Analog servos run at ~50 Hz updates
const int numServos = 2;  // Assumes servos are connected to 0,1,2,etc


////////////////////////////////////////
// PINS
////////////////////////////////////////
const int minPulse[numServos] = {250, 120}; //[110,  90];
const int maxPulse[numServos] = {340, 250}; //[480, 490];
const int laserPin = 7;
const int btnPin = D13;


////////////////////////////////////////////////////
// CLOUD VARIABLES
////////////////////////////////////////////////////
int nowVal = 0;
int enTimerVal = 0;
int timerVal = 10;
int timerCounter, timerWait;


////////////////////////////////////////
// INTERNAL VARIABLES
////////////////////////////////////////
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int servoPulse[numServos];
int servoDir[numServos];

int servoTarget[numServos] = {295, 185};
bool quiver;
unsigned long quiverStart;
int quiverTime = 3000;
const int quiverSpeed = 2;

bool enableToy = false;
int speed = 5, speedSet = 5;
int laserPulse = 4096;

unsigned long timerStart;
bool enableToyLast = false;

unsigned long btnPressed;
const int btnDelay = 3000;
int btnVal, btnValLast;

int nowMin, nowMinLast, nowHour;

// what is our longitude (west values negative) and latitude (south values negative)
float const LATITUDE = 43.67;
float const LONGITUDE = -79.36;
TimeLord tardis;
byte today[] = {  0, 0, 12, Time.day(), Time.month(), Time.year()-2000 }; // store today's date (at noon) in an array for TimeLord to use
int onTimeVal = 0;
int offTimeVal = 0;


////////////////////////////////////////
// SETUP
////////////////////////////////////////
void setup() {
    
    Particle.function("togEnable",       togEnable);
    Particle.function("togReset",         togReset);
    Particle.function("setSpeed",         setSpeed);
    Particle.function("setLaser",         setLaser);
    Particle.function("centreMotors", centreMotors);
    Particle.variable("timerVal",         timerVal);
    Particle.variable("timerWait",       timerWait);
    Particle.variable("timerCounter", timerCounter);
    Particle.variable("getEnableTimer", enTimerVal);
    Particle.variable("getTime",            nowVal);
    Particle.variable("getOnTimeVal",    onTimeVal);
    Particle.variable("getOffTimeVal",  offTimeVal);

    pinMode(btnPin, INPUT_PULLUP);

    pwm.begin();
    pwm.setPWMFreq(servoFreq);  // Analog servos run at ~50 Hz updates
    delay(10);
    
    // Init Time + timelord
    Time.beginDST();
    Time.zone(-6);                  // Spring to Fall is -5; Fall to Spring is -6
    tardis.TimeZone(-5 * 60);       // Spring to Fall is -4; Fall to Spring is -5
    tardis.DstRules(3,2,11,1,60);   // Second Sunday in March thru first Sunday in November
    tardis.Position(LATITUDE, LONGITUDE);
    updateTimerVal();

    togReset("");
    
    Particle.publish("log", "Ready!", PRIVATE);
}


////////////////////////////////////////
// LOOP
////////////////////////////////////////
void loop() {
    
    unsigned long now = millis();
    
    //////////////////////
    // TIMER
    //////////////////////
    time_t nowTime = Time.now();
    nowMin = Time.minute(nowTime);
    nowHour = Time.hour(nowTime);
    nowVal  = nowHour*100;
    nowVal += nowMin;
    if (!enableToy && nowMin != nowMinLast) {
        if (nowVal > onTimeVal || nowVal < offTimeVal) {
            timerCounter++;
            if (timerCounter > timerWait) togEnable("");
            nowMinLast = nowMin;
        }
    }
    
    //////////////////////
    // OPERATION
    //////////////////////
    if (enableToy) {
    
        // Drive the servos
        /*
        // Jittery Dancing
        for(int i=0; i<numServos; i++) {
            // 2/3 of the time, keep going in the same direction (creates more of a path/trail)
            int dir = random(3);
            if (dir > 1) servoDir[i] = !servoDir[i];
            
            int delta = random(speed);
            if (servoDir[i]) servoPulse[i] += delta;
              else servoPulse[i] -= delta;
            if (servoPulse[i] > maxPulse[i]) servoPulse[i] = maxPulse[i];
            if (servoPulse[i] < minPulse[i]) servoPulse[i] = minPulse[i];
            pwm.setPWM(i, 0, servoPulse[i]);
            delay(50);
        }
        */
        
        // Find a spot, quiver
        for(int i=0; i<numServos; i++) {
            
            int delta = random(speed);
            if (servoTarget[i] > servoPulse[i]) servoPulse[i] += delta;
              else servoPulse[i] -= delta;
            
            if (servoPulse[i] > maxPulse[i]) servoPulse[i] = maxPulse[i];
            if (servoPulse[i] < minPulse[i]) servoPulse[i] = minPulse[i];
            pwm.setPWM(i, 0, servoPulse[i]);
            delay(50);
        }
        
        if (!quiver && abs(servoTarget[0] - servoPulse[0]) < speed && abs(servoTarget[1] - servoPulse[1]) < speed) {
            quiver = true;
            speed = quiverSpeed;
            quiverStart = now;
            // Particle.publish("log", "Target found!", PRIVATE);
        }
        
        if (quiver) delay(random(500));
    
        if (quiver && now - quiverStart > quiverTime) {
            servoTarget[0] = random(minPulse[0],maxPulse[0]+1);
            servoTarget[1] = random(minPulse[1],maxPulse[1]+1);
            quiver = false;
            speed = speedSet;
            quiverTime = random(1000,5000);
            // Particle.publish("log", "New Target: "+String(servoTarget[0])+","+String(servoTarget[1]), PRIVATE);
        }
        
        
        // Timer
        if (!enableToyLast) timerStart = now;
        if ((now - timerStart) > (timerVal*60*1000)) togEnable("");
        
    }
    enableToyLast = enableToy;
    
    // Button toggle
    btnVal = !digitalRead(btnPin);
    if (btnVal && !btnValLast && (now - btnPressed) > btnDelay) {
        togEnable("");
        btnPressed = now;
    }
    btnValLast = btnVal;
    

}


////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////

void laserOn() {
    pwm.setPWM(laserPin, 0, laserPulse);
}

void laserOff() {
    pwm.setPWM(laserPin, 4096, 0);
}

void updateTimerVal() {
    // Update Sunset time
    today[3] = Time.day();
    today[4] = Time.month();
    today[5] = Time.year()-2000;
    if (tardis.SunSet(today)) { // if the sun will set today (it might not, in the [ant]arctic)
        onTimeVal  = (int) today[tl_hour]*100;
        onTimeVal += (int) today[tl_minute];
    }
    if (tardis.SunRise(today)) { // if the sun will rise today (it might not, in the [ant]arctic)
        offTimeVal  = (int) today[tl_hour]*100;
        offTimeVal += (int) today[tl_minute];
    }

    enTimerVal  = onTimeVal *10000;
    enTimerVal += offTimeVal;

    Particle.publish("log", "Enable Timer set: "+String(enTimerVal), PRIVATE);
}


////////////////////////////////////////
// CLOUD FUNCTIONS
////////////////////////////////////////

int togEnable(String command) {
    if (enableToy) {
        enableToy = false;
        laserOff();
        timerCounter = 0;
        timerWait = random(60,180);
        timerVal = random(5,15);
    } else {
        enableToy = true;
        laserOn();
    }
    return int(enableToy);
}

int togReset(String command) {
    enableToy = false;
    laserOff();
    timerCounter = 0;
    timerWait = random(60,180);
    timerVal = random(5,15);
    for(int i=0; i<numServos; i++) {
        servoPulse[i] = (minPulse[i] + maxPulse[i]) / 2;
    }
    speed = speedSet;
    laserPulse = 4096;
    return 1;
}

int setSpeed(const char *data) {
    if (data != "") {
        int value = atoi(data);
        if (value > 0) speedSet = value;
    }
    return speedSet;
}

int setLaser(const char *data) {
    if (data != "") {
        int value = atoi(data);
        if (value < 0) value = 0;
        if (value > 4096) value = 4096;
        laserPulse = value;
        if (enableToy) laserOn();
    }
    return laserPulse;
}

int centreMotors(const char *data) {
    if (!enableToy) {
        for(int i=0; i<numServos; i++) {
            pwm.setPWM(i, 0, (maxPulse[i] + minPulse[i])/2);
            delay(50);
        }
        return 1;
    }
    return 0;
}
