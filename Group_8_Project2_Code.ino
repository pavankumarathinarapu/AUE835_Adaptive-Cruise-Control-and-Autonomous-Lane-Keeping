#include <Servo.h> //define the servo library
// Sensor 1 ( Distance, Average Time, Time Variance)
double Sensor1CalData[][3] = {{20,1164.764706,20.70352941},
{30,1739.764706,11.42352941},
{40,2316.411765,9.647058824},
{50,2893.352941,24.95294118},
{60,3468.490196,11.09490196},
{70,4045.215686,4.33254902},
{80,4618.705882,3.771764706},
{90,5199.058824,4.656470588},
{100,5774.901961,10.81019608},
{110,6350.882353,9.185882353},
{120,6928.980392,5.979607843},
{130,7506.588235,10.32705882},
{140,8084.352941,4.112941176},
{150,8659.666667,10.90666667},
};

// Sensor 4 ( Distance, Average Time, Time Variance)
double Sensor4CalData[][3] = {{20,1153.12,5.209795918},
{30,1728.901961,7.170196078},
{40,2329.529412,8.494117647},
{50,2883.803922,7.840784314},
{60,3461.960784,12.19843137},
{70,4063.431373,2.650196078},
{80,4613.647059,2.712941176},
{90,5213.647059,78.79294118},
{100,5793.843137,10.37490196},
{110,6348.156863,7.654901961},
{120,6951.1,7.234693878},
{130,7529.4,12.48979592},
{140,8107.76,2.308571429},
{150,8683.45098,11.93254902},
};

// Sensor 6 ( Distance, Average Time, Time Variance)
double Sensor6CalData[][3] = {{20,1216.58,6.69755102},
{30,1777,8.4},
{40,2350.784314,10.17254902},
{50,2924.333333,0.346666667},
{60,3478.54902,11.13254902},
{70,4058.529412,1.214117647},
{80,4618.666667,0.226666667},
{90,5203.039216,5.318431373},
{100,5791.588235,7.967058824},
{110,6362.960784,11.31843137},
{120,6920.02,57.12204082},
{130,7514.24,11.6555102},
{140,8070.38,2.689387755},
{150,8666,68.24},
};

#define trigPin 13 //Trig to Arduino pin 13
#define echoPin 12 //Echo to Arduino pin 12
#define trigPin2 9 //Trig to Arduino pin 9
#define echoPin2 8 //Echo to Arduino pin 8



Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc
double throttle, x1, s1, time1, error1,error2, S_Kp_in, S_Kp_out,T_Kpf,T_Kpr,time2,x2,s2;
int count;
int steering=90,velocity=90; //defining global variables to use later

// Setup Initilization
void setup() {
  Serial.begin(9600); //start serial connection. Uncomment for PC
  ssm.attach(10);    //define that ssm is connected at pin 10
  esc.attach(11);     //define that esc is connected at pin 11
  pinMode(trigPin, OUTPUT); //Set the trigPin as output
  pinMode(echoPin, INPUT); //Set the echoPin as input
  pinMode(trigPin2, OUTPUT); //Set the trigPin as output
  pinMode(echoPin2, INPUT); //Set the echoPin as input

}

//Loop for Cotrol Logic
void loop() {
count = add(count);
Serial.print(count);
Serial.print("\t");
steering=90;  //set steering to 90
throttle=1500;  //set throttle to 1500
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
time1 = pulseIn(echoPin, HIGH);
Serial.print("Time: ");
Serial.print(time1);
Serial.print("\t");

// Limiting distance from Throttle sensor
if (time1 >= 7500)
{
  time1 = 7500;
}
else if (time1 <= 1220)
{
  time1 = 1220;
}


// Distance Variance Calculation for sensors using function
x1 = Interpolate_Dist(time1, Sensor6CalData);
Serial.print("Interp. Dist2: ");
Serial.print(x1);
Serial.print("\t");

// Calculating Distance using function
s1 = Dist_Sens6(time1);
//Serial.print("Eq. Distance : ");
//Serial.print(s1);
//Serial.print("\t");

error1 = 41 - x1;

Serial.print("Error1: ");
Serial.print(error1);
Serial.print("\t");

// condition for throttle using kp and errors
T_Kpf = 0.70;   // KP for front throttle
T_Kpr =5.4;     // KP for rear throttle


if (error1 < 0)
{
  throttle = 1548 - (T_Kpf*error1); // 1548 deadzone of throttle in front direction 
}
else if (error1 > 0)
{
  throttle = 600 - (T_Kpr*error1); // 600 deadzone of throttle in reverse direction
}

// Condition to limit throttle
if (throttle >=1572)
{
    throttle = 1572;
}

Serial.print("Throttle: ");
Serial.print(throttle);
Serial.print("\t");

// Sensor 4
digitalWrite(trigPin2, LOW);
delayMicroseconds(2);
digitalWrite(trigPin2, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin2, LOW);
time2 = pulseIn(echoPin2, HIGH);
Serial.print("Time2: ");
Serial.print(time2);
Serial.print("\t");

// Limiting distance from Steering sensor 
if (time2 >= 2500)
{
  time2 = 2500;
}
else if (time2 <= 1175)
{
  time2 = 1175;
}


// Distance Variance Calculation for sensors using function
x2 = Interpolate_Dist(time2, Sensor4CalData);
Serial.print("Interp. Dist2 : ");
Serial.print(x2);
Serial.print("\t");

// Calculating Distance using function
s2 = Dist_Sens4(time2);
//Serial.print("Eq. Distance : ");
//Serial.print(s2);
//Serial.print("\t");

// Steering error and output conditions
error2 = 27.25 - x2;
if (error2 <= -6.87)
{
  error2 = -7.2;
}
Serial.print("Error2: ");
Serial.print(error2);
Serial.print("\t");
// condition for steering using 2 stage kp and errors
S_Kp_in = 1.75;
S_Kp_out = 3.1;
if (abs(error2) < 3.5)
{
  steering = 90 - (S_Kp_in*error2); // In range calculation
}
else if (abs(error2) > 3.5)
{
  steering = 90 - (S_Kp_out*error2); // Out range calculation
}
//steering = 90 - (S_Kp*error2);
Serial.print("Steering: ");
Serial.print(steering);
Serial.println("\t");

// Call the set Vehicle function to set the vehicle steering and velocity(speed) values 
if (count%4 == 1)
{
setVehicleHR(steering, 1500);
}
else
{
setVehicleHR(steering, throttle);
}
}

//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//
void setVehicleHR(int s, int v) 
{
  s=min(max(0,s),180);  //saturate steering command
  v=min(max(1388,v),1611); //saturate throttle/velocity command
  ssm.write(s); //write steering value to steering servo
  esc.writeMicroseconds(v); //write throttle/velocity value to the ESC unit
}
//***************** Do not change above part *****************//

// Sensor 1 distance linear equation
double Dist_Sens1(double t1)
{
  double s1;
  s1 = (0.1759*t1) - 4.8342 ;  // distance equation for sensor 1
  return s1;
}

// Sensor 4 distance linear equation
double Dist_Sens4(double t2)
{
  double s4;
  s4 = (0.1757*t2)  - 8.2042 ;  // distance equation for sensor 4
  return s4;
}

// Sensor 6 distance linear equation
double Dist_Sens6(double t3)
{
  double s6;
  s6 = (0.1758*t3)  - 9.1741 ;  // distance equation for sensor 6
  return s6;
}

// Function to interpolate Distance from raw time values
double Interpolate_Dist(double y, double data[][3])
{
  double x, x0, x1, y0, y1;
  for (int i = 0; i < 13; i++)
  {
    if (y > data[i][1] && y < data[i + 1][1])
    {
      y0 = data[i][1];          //Time lower bound
      y1 = data[i + 1][1];     //Time upper bound
      x0 = data[i][0];        //Distance lower bound
      x1 = data[i + 1][0];   //Distance upper bound
      x = x0 + ((x1 - x0) * ((y - y0) / (y1 - y0)));     // Interpolation Equation 
    }
  }
  return x;
}

// Count Function
int add(int count)
{
  count = count + 1;
  return count;
}