//Task1.1
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


double z;                 // z is Kalman filter output
int LED = 4;            //connect LED to pin 4
int beep = 5;          //connect buzzer to pin 5
int count = 1;
double P = 10;        // Inititation Value for P(Co Variance) in Kalman filter function
double Kalman_Output[1][3];     //Kalman Filter output in Array
double time1,time2;

#define trigPin 13 //Trig to Arduino pin 13
#define echoPin 12 //Echo to Arduino pin 12

// Initiation
void setup()
{
  Serial.begin(9600);         //Serial communications at 9600 bps
  pinMode(trigPin, OUTPUT); //Set the trigPin as output
  pinMode(echoPin, INPUT); //Set the echoPin as input

  // Initial Sensor Measured time to inititate  Kalman Filter output
  double t;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  t = pulseIn(echoPin, HIGH);
  z = t;

// LED & Buzzer Pin Setup  
  pinMode(LED, OUTPUT); //pin 4 is output 
  pinMode(beep, OUTPUT); //pin 5 is output
  time1=millis();
  Serial.println("Start");
}

// Loop
void loop()
{
  double time,myTime;
  double a, s;
  // Sensor  Time Calculation
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  time = pulseIn(echoPin, HIGH);
  count = add(count);
  Serial.print(count);
  Serial.print("\t");
  Serial.print("Time : ");
  Serial.print(time);
  Serial.print("\t");
  Serial.print("\t");

  // Distance Calculation for  sensor using function
  double y = time;
  double n = time;
  double x, x0, x1, y0, y1;
  double m, m0, m1, n0, n1;

  // Distance Variance Calculation for sensors using function
  x = Interpolate_Dist(y, Sensor1CalData);
  //Serial.print("Distance : ");
  //Serial.print(x);
  //Serial.print("\t");
  m = Interpolate_Var(n, Sensor1CalData);
  Serial.print("Variance : ");
  Serial.print(m);
  Serial.print("\t");
  Serial.print("\t");

  // Running Kalman Filter using function
  Kalman_Output[1][3] = Kalman_Filter(m, y);
  Serial.print("Filtered Time : ");
  Serial.print(Kalman_Output[0][0]);
  Serial.print("\t");
  Serial.print("\t");

  // Calculating Distance using function
  s = Dist_Sens1(Kalman_Output[0][0]);
  Serial.print("Filtered Distance : ");
  Serial.print(s);
  Serial.print("\t");
  Serial.print("P : ");
  Serial.print(Kalman_Output[0][2]);
  Serial.println("\t");
  //Serial.print("Time: ");
  myTime = millis();
  //Serial.println(myTime);

  // Loop exit Strategy
  if(Kalman_Output[0][2] < 0.4)
  {
    digitalWrite(LED, HIGH); //turn on LED 
    //digitalWrite(LED, LOW); //turn off LED 
    time2=millis();
    Serial.println("Finish");
    Serial.print("Time Cost");
    Serial.print("\t");
    Serial.println(time2-time1);
    delay(100);
    digitalWrite(beep, HIGH); //turn on buzzer
    delay(1000);
    digitalWrite(beep, LOW); //turn off buzzer
    exit(0);
  }
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

// Function to interpolate Time Variance from  raw time values
double Interpolate_Var(double n, double data[][3])
{
  double m, m0, m1, n0, n1;
  for (int j = 0; j < 13; j++)
  {
    if (n > data[j][1] && n < data[j + 1][1])
    {
      n0 = data[j][1];          //Time lower bound
      n1 = data[j + 1][1];     //Time upper bound
      m0 = data[j][2];        //Variance Lower bound
      m1 = data[j + 1][2];   //Variance Upper bound
      m = m0 + ((m1 - m0) * ((n - n0) / (n1 - n0)));     // Interpolation Equation 
    }
  }
  return m;
}

// Kalman filter function for single sensor 
double Kalman_Filter(double m, double y)
{
  double K;     
                              // no prediction step, as the loop  uses the old P value
  K = P / (P + m);           // Kalman gain calculation // m =  sensor  variance 
  z = z + (K*(y - z));      // Y = sensor raw readings. // z = Kalman filter corrected value 
  P = (1 - K)*P;           // P -> Convariance, updation step
  
  Kalman_Output[0][0] = z;
  Kalman_Output[0][1] = K;
  Kalman_Output[0][2] = P;
  return Kalman_Output[1][3];
}

// Count Function
int add(int count)
{
  count = count + 1;
  return count;
}

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