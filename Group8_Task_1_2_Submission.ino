// TASK 1.2 Sensor Fusion
// Sensor 1 ( Distance, Average Time, Time Variance , Distance Variance)
double Sensor1CalData[][4] = {{200,1164.764706,20.70352941,0.086890119},
{300,1739.764706,11.42352941,0.01252619},
{400,2316.411765,9.647058824,0.01828619},
{500,2893.352941,24.95294118,0.437277536},
{600,3468.490196,11.09490196,0.039734783},
{700,4045.215686,4.33254902,0.042031225},
{800,4618.705882,3.771764706,0.029367667},
{900,5199.058824,4.656470588,0.023184783},
{1000,5774.901961,10.81019608,0.017554113},
{1100,6350.882353,9.185882353,0.74956809},
{1200,6928.980392,5.979607843,0.05563834},
{1300,7506.588235,10.32705882,0.041309091},
{1400,8084.352941,4.112941176,0.04133913},
{1500,8659.666667,10.90666667,0.051993676},
};

//Sensor 6 Data  ( Distance, Average Time, Time Variance , Distance Variance)
double Sensor6CalData[][4] = {{200,1216.58,6.69755102,0.114453333},
{300,1777,8.4,0.053420109},
{400,2350.784314,10.17254902,0.04503619},
{500,2924.333333,0.346666667,0.076427706},
{600,3478.54902,11.13254902,0.48369},
{700,4058.529412,1.214117647,0.52461783},
{800,4618.666667,0.226666667,0.073099209},
{900,5203.039216,5.318431373,0.019501515},
{1000,5791.588235,7.967058824,0.013275714},
{1100,6362.960784,11.31843137,0.010783158},
{1200,6920.02,57.12204082,0.0559619},
{1300,7514.24,11.6555102,0.029119048},
{1400,8070.38,2.689387755,0.053541},
{1500,8666,68.24,0.015223529},
};

//Sensor 4 data  ( Distance, Average Time, Time Variance , Distance Variance)
double Sensor4CalData[][4] = {{200,1153.12,5.209795918,0.085296371},
{300,1728.901961,7.170196078,0.143567641},
{400,2329.529412,8.494117647,0.012870565},
{500,2883.803922,7.840784314,0.036938306},
{600,3461.960784,12.19843137,0.07473485},
{700,4063.431373,2.650196078,0.144780871},
{800,4613.647059,2.712941176,0.385650568},
{900,5213.647059,78.79294118,0.07425758},
{1000,5793.843137,10.37490196,0.84184053},
{1100,6348.156863,7.654901961,0.505655114},
{1200,6951.1,7.234693878,0.03979678},
{1300,7529.4,12.48979592,0.012670455},
{1400,8107.76,2.308571429,0.264403209},
{1500,8683.45098,11.93254902,0.129135798},
};


double z;                        // z is Kalman filter output
int count; 
double P = 1;                   // Inititation Value for P(Co Variance) in Kalman filter function
double t; // t is time variable for setup
double Kalman_Output[1][3];     //Kalman Filter output in Array
int LED = 4;                    //connect LED to pin 4
int beep = 5; //connect buzzer to pin 5
double Initial,finish;          // Time Variable for Initial and Finish time 
#define trigPin1 10            //  Connect Trigpin1 to port 10
#define echoPin1 11           //  Connect echopin1 to port 11 
#define trigPin2 13          // Connect Trigpin2 to port 13
#define echoPin2 12         // Connect echopin2 to port 12

// Initiation
void setup() {
Serial.begin (9600);           //Serial communications at 9600 bps
pinMode(trigPin1, OUTPUT);    //Set the trigPin1 as output
pinMode(echoPin1, INPUT);    //Set the echoPin1 as input
pinMode(trigPin2, OUTPUT);  //Set the trigPin2 as output
pinMode(echoPin2, INPUT);  //Set the echoPin2 as input
// Initial Sensor Measured time to inititate  Kalman Filter output
double t;
digitalWrite(trigPin1, LOW);    
delayMicroseconds(2);        
digitalWrite(trigPin1, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin1, LOW);
t = pulseIn(echoPin1, HIGH);
z = t;
// LED & Buzzer Pin Setup
pinMode(LED, OUTPUT);     //pin 4 is output 
pinMode(beep, OUTPUT);   //pin 5 is output
  Initial = millis(); //Initial Time for Time Cost
  Serial.println("Start");
}

// Loop
void loop() {
double time1, d1, time2, d2;

// First Sensor  Time Calculation
digitalWrite(trigPin1, LOW);
delayMicroseconds(2);
digitalWrite(trigPin1, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin1, LOW);
time1 = pulseIn(echoPin1, HIGH); 
delay(10); // delay to avoid sensor interference 
Serial.print("T1 : ");
Serial.print(time1); 
Serial.print("\t");

// Second Sensor  Time Calculation
delayMicroseconds(20);
digitalWrite(trigPin2, LOW);
delayMicroseconds(2);
digitalWrite(trigPin2, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin2, LOW);
time2 = pulseIn(echoPin2, HIGH);
Serial.print("T2 : ");
Serial.print(time2);
Serial.print("\t"); 


//count = add(count);
double t1 = time1; 
double t2 = time2;
double r1, r2;
double x, x0, x1, y0, y1;
double m, m0, m1, n0, n1;

// Distance Calculation for both sensors using function
d1 = Dist_Sens4(t1);
//Serial.print("Dist1 : ");
//Serial.print(d1);
//Serial.print("\t");
d2 = Dist_Sens6(t2);
//Serial.print("Dist2 : ");
//Serial.print(d2);
//Serial.print("\t");

// Distance Variance Calculation for both sensors using function
r1 = Interpolate_Dist_Var(d1, Sensor4CalData);
Serial.print("Var1 : ");
Serial.print(r1,5);
Serial.print("\t");
r2 = Interpolate_Dist_Var(d2, Sensor6CalData);
Serial.print("Var2 : ");
Serial.print(r2,5);
Serial.print("\t");

// Running Kalman Filter using function
Kalman_Output[1][3] = Kalman_Filter(r1, d1, r2, d2);
Serial.print("Filtered Distance : " );
Serial.print(Kalman_Output[0][0]);
Serial.print("mm" );
Serial.print("\t");
Serial.print("P : ");
Serial.print(Kalman_Output[0][2], 5);
Serial.println("\t");

// Loop exit Strategy
if(Kalman_Output[0][2] < 0.0012)
  {
    digitalWrite(LED, HIGH); //turn on LED 
    //digitalWrite(LED, LOW); //turn off LED 
    finish = millis();
    Serial.println("Finish");
    Serial.print("Time Cost");
    Serial.print("\t");
    Serial.println(finish-Initial);
    delay(100);
    digitalWrite(beep, HIGH); //turn on buzzer
    delay(1000);
    digitalWrite(beep, LOW); //turn off buzzer
    exit(0);
  }
}

// Function to interpolate Distance from raw time values
double Interpolate_Dist(double y, double data[][4])
{
  double x, x0, x1, y0, y1;
  for (int i = 0; i < 13; i++)
  {
    if (y > data[i][1] && y < data[i + 1][1])
    {
      y0 = data[i][1];       //Time lower bound
      y1 = data[i + 1][1];  //Time upper bound
      x0 = data[i][0];     //Distance lower bound
      x1 = data[i + 1][0];//Distance upper bound
      x = x0 + ((x1 - x0) * ((y - y0) / (y1 - y0)));    // Interpolation Equation 
    }
  }
  return x;
}

// Function to interpolate Time Variance from  raw time values
double Interpolate_Var(double n, double data[][4])
{
  double m, m0, m1, n0, n1;
  for (int j = 0; j < 13; j++)
  {
    if (n > data[j][1] && n < data[j + 1][1])
    {
      n0 = data[j][1];         //Time lower bound
      n1 = data[j + 1][1];    //Time upper bound
      m0 = data[j][2];       //Variance Lower bound
      m1 = data[j + 1][2];  //Variance Upper bound
      m = m0 + ((m1 - m0) * ((n - n0) / (n1 - n0)));    // Interpolation Equation 
    }
  }
  return m;
}

// Function to interpolate Distance Variance from  raw time values
double Interpolate_Dist_Var(double q, double data[][4])
{
  double p,p0,p1,q0,q1;
  for (int k = 0; k < 13; k++)
  {
    if (q > data[k][0] && q < data[k + 1][0])
    {
      q0 = data[k][0];          //Time lower bound
      q1 = data[k + 1][0];     //Time upper bound
      p0 = data[k][3];        //Variance lower bound
      p1 = data[k + 1][3];   //Variance lower bound
      p = p0 + ((p1 - p0) * ((q - q0) / (q1 - q0)));     // Interpolation Equation 
    }
  }
  return p;
}

// Kalman filter function for sensor fusion
double Kalman_Filter(double m1, double y1,double m2, double y2 )
{
  double K;     
                                // no prediction step, as the loop  uses the old P value
  K = P / (P + m1);            // Kalman gain calculation // m1 = first sensor  variance 
  z = z + (K*(y1 - z));       // Y = sensor raw readings. // z = Kalman filter corrected value 
  P = (1 - K)*P;             // P -> Convariance, updation step

  K = P / (P + m2);           // Kalman gain correction // m2 = second sensor variance 
  z = z + (K*(y2 - z));      // Y = sensor raw readings. //z = Kalman filter corrected value 
  P = (1 - K)*P;            // P -> Convariance, correction step

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