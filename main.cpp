#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "FastPWM.h"
#define PI 3.14159265358979323846
#define PWMfrequency 5000
#define PWMperiodTicks 12000 // == 60 000 000 / PWMfrequency

// Trick to bypass ISR error
class MyAnalogIn : public AnalogIn {
public:
    MyAnalogIn(PinName inp) : AnalogIn(inp) { }
    virtual void lock() { }
    virtual void unlock() { }
};

// Define number of communication parameters with matlab
#define NUM_INPUTS 5
#define NUM_OUTPUTS 5

Serial pc(USBTX, USBRX,115200);     // USB Serial Terminal for debugging
ExperimentServer server;            // Object that lets us communicate with MATLAB
Timer t;                            // Timer to measure elapsed time of experiment
Ticker currentLoopTicker;           // Ticker to call high frequency current loop
DigitalOut led_g(LED_GREEN,1);      // UDP server state indicator

/************************Complete the code in this block**************************/
// Assign digital/analog pins for control and sensing
FastPWM    M1PWM(D9);               // Motor PWM output (we are using the "FastPWM" rather than built-in "PwmOut" for higher resolution)
DigitalOut M1INA(D2);               // Motor forward enable
DigitalOut M1INB(D4);               // Motor backward enable
MyAnalogIn   CS(A2);                // Current sensor
/*********************************************************************************/

// Create a quadrature encoder
// 64(counts/motor rev)*18.75(gear ratio) = 1200(counts/rev)
// Pins A, B, no index, 1200 counts/rev, Quadrature encoding
// Note: Reversed A & B to match motor direction
QEI encoder(D5,D3, NC, 1200 , QEI::X4_ENCODING); 
const float radPerTick = 2.0*PI/1200.0;

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM);

const float SupplyVoltage = 12;     // Supply voltage in Volts
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM);

void currentLoopFunc();



float angle = 0.0;
float velocity = 0.0;
float voltage = 0.0;
float current = 0.0;

float current_des   = 0.0f; // Desired current
float Kp   = 0.0; // Kp
float Rm   = 0.0; // Resistance
float Kb   = 0.0; // Kb

int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    led_g = 0;  // UDP server is ready, turn on green led

    // Setting PWM frequency. Here is 5k Hz, same as the current loop
    M1PWM.prescaler(1);
    M1PWM.period_ticks(PWMperiodTicks);
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    
    while(1) {
        if (server.getParams(input_params,NUM_INPUTS)) {
            // Unpack parameters from MATLAB
            current_des   = input_params[0]; // Desired current
            Rm            = input_params[1]; // Resistance of motor
            Kb            = input_params[2]; // Kb (Back EMF)
            Kp            = input_params[3]; // Kp
            float ExpTime = input_params[4]; // Expriement time in second

            // Setup experiment
            t.reset();
            t.start();
            encoder.reset();
            setMotorVoltage(0,M1INA,M1INB,M1PWM);
            // Set high frequency corrent loop control
            currentLoopTicker.attach(&currentLoopFunc,0.0002);

            // Run experiment
            while( t.read() < ExpTime ) { 

                // Form output to send to MATLAB    
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = angle;
                output_data[2] = velocity;
                output_data[3] = voltage;
                output_data[4] = current;
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);
                wait(.001);                  // Control and sending data in 1kHz
            }     
            // Cleanup after experiment
            currentLoopTicker.detach();
            server.setExperimentComplete();
            setMotorVoltage(0,M1INA,M1INB,M1PWM);
        } // end if
    } // end while
} // end main


void currentLoopFunc(){
/************************Complete the computation of current sensing***************/
    // Copy current sensing code from previous part
    // Read the current sensor value
    current = 36.7f * (CS - 0.5f);
/*********************************************************************************/
    angle = (float)encoder.getPulses()*radPerTick;
    velocity = encoder.getVelocity()*radPerTick;
/************************Current Control Code***************/
    voltage = Rm * current_des + Kb * velocity - Kp*(current - current_des);
/*********************************************************************************/
    setMotorVoltage(voltage,M1INA,M1INB,M1PWM);

}

//Set motor voltage (nagetive means reverse)
void setMotorVoltage(float voltage, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM){
    setMotorDuty(voltage / SupplyVoltage, INA, INB, PWM);
}

// Set motor duty [-1.0f, 1.0f]
void setMotorDuty(float duty, DigitalOut &INA, DigitalOut &INB, FastPWM &PWM)
{
    unsigned char reverse = 0;

    if (duty < 0) {
        duty = -duty;  // Make duty a positive quantity
        reverse = 1;  // Preserve the direction
    }

    if (duty == 0) {
        INA = 0;  // Make the motor coast no
        INB = 0;  // matter which direction it is spinning.
    } else if (reverse) {
        INA = 0;
        INB = 1;
    } else {
        INA = 1;
        INB = 0;
    }

    // The standard way to set built-in PWM is: 
    // PWM.write(duty);
    // However, we are using a high resolution PWM (FastPWM library). 
    // We have to set the PWM is the following way.
    PWM.pulsewidth_ticks((int) (PWMperiodTicks*duty));
}