//Import libraries
#include <Servo.h>
#include <SharpIR.h>
#include <SoftwareSerial.h>

// Serial for Bluetooth
SoftwareSerial bleSerial (10, 11); // RX | TX

// Data type for serial
typedef union {
    int number;
    byte bytes[1];
} FLOATUNION_t;

// Variable for serial 
FLOATUNION_t dato1;

// Define distance sensor model, input pin and variable to store distance:
#define IRPin A0
#define model 1080
SharpIR mySensor = SharpIR(IRPin, model);

// Define servo variables
Servo servo;
#define Umax 66 // maximum angle of the servomotr in degrees
#define Umin -66 // minimun angle
#define Umax_rad 1.151 // maximum angle of the servomotr in radiants
#define Umin_rad -1.151 // minimun angle
#define T 0.09 // sampling time

double setpoint=0.15, setpoint_prec = 0.15;  // In meters : 30cm --> 0.3m
double y, y_prec;
double error;
double P, I, D, U;
double I_prec=0, U_prec=0, D_prec=0;        
boolean Saturation = false;

double Kp = 7.4; 
double Ki = 0; 
double Kd = 7.9; 

float measure_1 (void);
void move_servo(int);

void setup() {
   Serial.begin(9600);
   bleSerial.begin(38400);
   
   servo.attach(3);
   delay(1000); 
   move_servo(90);
   delay(3000);
   y_prec = measure_1(); 
   delay(1000);
}

void loop() {
   if (bleSerial.available()>= 2){ 
      for(int i = 1; i>=0; i--){
         dato1.bytes[i] = bleSerial.read(); 
      }
      if (dato1.number != 0 ) {
         //Serial.println(dato1.number);
         setpoint = 0.01*dato1.number;
         //setpoint = 0.53*setpoint + 0.47*setpoint_prec;
         Serial.println(setpoint);
      }
   }   
         
   delay(3);
   
   y = average_measurement();  // distance of the cart from the sensor ( meters )   
   y =  0.53*y + 0.47*y_prec  ;   // (  alfa*y :   if alfa increases, y less attenuated and similar to the measured y  --> so the measurement is noisy but fast )
   delay (3);
   
   error = round( 100*(y - setpoint) )*0.01;     // meters            
    
   P = Kp*error;
   
   if ( ! Saturation )  I = I_prec + T*Ki*error;

   D = (Kd/T)*(y - y_prec);
   
   D = 0.56*D + 0.44*D_prec;    // filtering D    
   
   U = P + I + round(100*D)*0.01 ;  // U in radiants
   
   if ( U < Umin_rad)  {
                        U=Umin_rad; 
                        Saturation = true;
                       }
                   
   else if ( U > Umax_rad)  {
                             U=Umax_rad; 
                             Saturation = true;
                            }

   else     Saturation = false;                   
    
   U=round(U*180/M_PI);     // Transform U in degrees. Now I have :   -63° < U < 63°   
    
   U=map(U, Umax, Umin, 30, 180); // I map the computed value of U to the corresponding value of the servomotor
   //Serial.println(U);
   if (U < 83 || U > 95 || abs(error) > 0.02 ) move_servo( round(U) );   // I continue until I have error and the control action U is greater than a threshold.
   
   delay (8);  
   
   I_prec = I;
   y_prec = y;
   D_prec = D;
   setpoint_prec = setpoint;     
}


float measure_1 (void) {
   int distance_cm;
   distance_cm = mySensor.distance();
   //Serial.println(distance_cm);
   return 0.01*(distance_cm);   // meters   
}

float average_measurement(void){
   float AverageDistance = 0;
   int MeasurementsToAverage = 12;
   for(int i = 0; i < MeasurementsToAverage; ++i)
    {
      AverageDistance += measure_1();
      delay(1);
    }  
   AverageDistance /= MeasurementsToAverage;
   //Serial.println(AverageDistance);  
   return (AverageDistance);
} 

void move_servo(int u) {
   servo.write(u-map(u, 30, 150, 14, 3));
}
