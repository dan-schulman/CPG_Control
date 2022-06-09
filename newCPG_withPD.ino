#include <Servo.h>
#include <EEPROM.h>

void PD_S0(float theta, float amplitude);
void PD_S1(float theta, float amplitude);
const int numOscillators=3;
const int numChambers=numOscillators*2;
Servo myServo[numChambers];
int pressurepin[6]={A0,A1,A2,A3,A4,A5};
int solenoid[3]={6,7,8};

float set_angle[3]={0,0,0};
    float previous_set_angle[3]={0,0,0};

int servoPin[numChambers];
  float calib_volt[numChambers];
  
  float phi[numOscillators];
  float phi_d[numOscillators];
  float r[numOscillators];
  float r_d[numOscillators];
  float r_dd[numOscillators];
  float x[numOscillators];
  float x_d[numOscillators];
  float x_dd[numOscillators];
  
  float omega[numOscillators];
  float R[numOscillators];
  float X[numOscillators];
  
  float w[numOscillators][numOscillators];
  float gamma[numOscillators][numOscillators];
  
  float theta[numOscillators];

  //Initialize arrays for updated state variable based on Euler method
  float new_phi[numOscillators];
  float new_r_d[numOscillators];
  float new_r[numOscillators];
  float new_x_d[numOscillators];
  float new_x[numOscillators];

  float ax=20;
  float ar=20;
  float T=0.01; //Euler method time step
  
void setup() {
  Serial.begin(9600);
  
 
  
  
    for(int i=0; i<numOscillators; i++) 
    {
      calib_volt[i]=analogRead(pressurepin[i]);
      if (i==0)
      {
        calib_volt[i]=100;
      }
      if (i==1)
      {
        calib_volt[i]=83;
      }
    }
  //Calibration step to align servo with valve
  /*
  for(int i=0; i<numChambers; i++)
  {
    myServo[i].write(0);
    delay(2000);
  }
  
  for(int i=0; i<numChambers; i++)
  {
    myServo[i].write(125);
    delay(2000);
  }
  */

  

  
  //First initialize 1D arrays for state variables (initial conditions)
  for(int i=0; i<numOscillators; i++)
  {
    phi[i]=0;
    x[i]=0;
    r[i]=0;
    phi_d[i]=0;
    x_d[i]=0;
    r_d[i]=0;
    x_dd[i]=0;
    r_dd[i]=0;
  
    theta[i]=0;
  }
  
  for(int i=0; i<numOscillators; i++)
  {
    new_phi[i] = 0;
    new_r_d[i] = 0;
    new_r[i] = 0;
    new_x_d[i] = 0;
    new_x[i] = 0;
  }

  
  //Second initialize 1D arrays for desired parameters
  for(int i=0; i<numOscillators; i++)
  {
    omega[i]=7*(i+1);
    R[i]=5;
    X[i]=0; //Use this for turning
  }
  
  //Then initialize 2D arrays
  for(int i=0; i<numOscillators; i++)
  {
    for(int j=0; j<numOscillators; j++)
    {
      if(i==j)
      {
        w[i][j]=0;
        gamma[i][j]=0;
      }
      else
      {
        w[i][j]=0.6;
        gamma[i][j]=0.7;
      }
    }
  }

  
}

void loop() 
{
  Euler(numOscillators,  ax,  ar,  T, phi,  phi_d, 
   r,  r_d,  r_dd, 
   x,  x_d,  x_dd, 
   omega,  R,  X, 
   w,  gamma,  theta, 
   new_phi, new_r_d, 
   new_r,  new_x_d,  new_x);
}

void Euler(int numOscillators, float ax, float ar, float T,float phi[], float phi_d[], 
  float r[], float r_d[], float r_dd[], 
  float x[], float x_d[], float x_dd[], 
  float omega[], float R[], float X[], 
  float w[][3], float gamma[][3], float theta[], 
  float new_phi[],float new_r_d[], 
  float new_r[], float new_x_d[], float new_x[])
{

  //Initialization of CPG coupling parameter arrays
  /*
  
  phi[N]-->phase (state variable)
  r[N]-->amplitude (state variable)
  x[N]-->offset (state variable)
  
  omega[N]-->desired frequency
  R[N]-->desired amplitude
  X[N]-->desired offset
  
  w[N][N]-->coupling weight
  gamma[N][N]-->phase bias
  
  ar,ax-->gains
  
  theta-->output
  */
  for(int i=0; i<numOscillators; i++){
    phi_d[i] = omega[i] ;
    for(int j=0; j<numOscillators; j++)
    {
      phi_d[i] += w[i][j]*r[j]*sin(phi[j]-phi[i]-gamma[i][j]);
    }
    new_phi[i] = phi[i]+T*phi_d[i];
    r_dd[i]=ar*((ar/4)*(R[i]-r[i])-r_d[i]);
    new_r_d[i] = r_d[i]+T*r_dd[i];
    new_r[i] = r[i]+T*r_d[i];
    x_dd[i] = ax*((ax/4)*(X[i]-x[i])-x_d[i]);
    new_x_d[i] = x_d[i]+T*x_dd[i];
    new_x[i] = x[i]+T*x_d[i];
  }
  
  for(int i=0; i<numOscillators; i++) 
  {
    phi[i] = new_phi[i];
    r[i] = new_r[i];
    x[i] = new_x[i];
    r_d[i] = new_r_d[i];
    x_d[i] = new_x_d[i];
    theta[i] = x[i]+r[i]*sin(phi[i]);
  }
  
    Serial.print(theta[0]);
    Serial.print(",");
    Serial.print(theta[1]);
    Serial.print(",");
    Serial.println(theta[2]);
    Serial.print(",");
    
    
    float constants[8]={-1.74318228544700e-13,  1.68213037558633e-12,  -6.71663313801535e-12,
    1.43146154286253e-11,  -1.74961669337208e-11, 1.22078888318533e-11,  37.5657400450744,  -19.0458302028543};

    
    
    for(int i=0; i<numOscillators; i++) 
    {
      float voltage=analogRead(pressurepin[i]);
      voltage=voltage*0.507/calib_volt[i];
      float read_pressure=(constants[0]*pow(voltage,7))+(constants[1]*pow(voltage,6))+(constants[2]*pow(voltage,5))+
      (constants[3]*pow(voltage,4))+(constants[4]*pow(voltage,3))+(constants[5]*pow(voltage,2))
      +(constants[6]*(voltage))+constants[7];
      
      if(i==0)
      {
        //PD_S0(theta[i],R[i],i);
        //Serial.println(read_pressure);
        //Serial.print(",");
      }
      if(i==1)
      {
        
        //PD_S1(theta[i],R[i],i);
        //Serial.print(read_pressure);
        //Serial.print(",");
      }
      if(i==2)
      {
        
        //PD_S2(theta[i],R[i],i);
        //Serial.println(read_pressure);
        //Serial.print(",");
      }
    }
    //Serial.println(",");

}


void PD_S0(float theta, float amplitude, int i) {
         
      float T=0.01;
      
      float calib_volt=100;

      float tolerance=1;
      float error=5;
      float previous_error=0;
      float servo_desired_angle=0;
      float min_pressure=10;
      float max_pressure=14;
      float set_pressure=(max_pressure-min_pressure)*(theta+amplitude)/(2*amplitude)+min_pressure;
      int error_check=1;
      float error_array[5];
      for (int i=0; i<5; i++)
      {
        error_array[i]=5;
      }
      
      while(true)
      {
        float voltage=analogRead(pressurepin[i]);

        float constants[8]={-1.74318228544700e-13,  1.68213037558633e-12,  -6.71663313801535e-12,
        1.43146154286253e-11,  -1.74961669337208e-11, 1.22078888318533e-11,  37.5657400450744,  -19.0458302028543};
        voltage=voltage*0.507/calib_volt;
        float read_pressure=(constants[0]*pow(voltage,7))+(constants[1]*pow(voltage,6))+(constants[2]*pow(voltage,5))+
        (constants[3]*pow(voltage,4))+(constants[4]*pow(voltage,3))+(constants[5]*pow(voltage,2))
        +(constants[6]*(voltage))+constants[7];
        
        previous_error=error;
        error=set_pressure-read_pressure;
        if(read_pressure>set_pressure)
        {
          //Serial.println("solenoid_go");
          digitalWrite(solenoid[i],HIGH);
        }
        else
        {
          digitalWrite(solenoid[i],LOW);
        }
        for(int j=0; j<numOscillators; j++)
        {
          if(j==1)
          {
            digitalWrite(solenoid[j],LOW);
          }
          if(j==2)
          {
            digitalWrite(solenoid[j],HIGH);
          }
        }
        
        double deriv_error=(error-previous_error)/T;
        //tune
        
        float Kp=0.15;
        float Kd=0.06;
        
        servo_desired_angle=38+Kp*error+Kd*deriv_error;
        servo_desired_angle=125-servo_desired_angle;
        myServo[i].write(servo_desired_angle);

        //Serial.println(read_pressure);
        //Serial.print(",");
        
        delay(T*1000);
        for(int k=0; k<5; k++)
        {
          if(abs(error_array[k])>tolerance)
          {
             error_check=0;
             for(int i=0; k<4;k++)
             {
              error_array[k]=error_array[k+1];
             }
             error_array[4]=error;
          }
        }
        if(error_check==1)
        {
          break;
        }
        error_check=1;
       }     
       return;
}


void PD_S1(float theta, float amplitude, int i) {
         
      float T=0.01;
      
      float calib_volt=83;

      float tolerance=1;
      float error=5;
      float previous_error=0;
      float servo_desired_angle=0;
      float min_pressure=5;
      float max_pressure=8;
      float set_pressure=(max_pressure-min_pressure)*(theta+amplitude)/(2*amplitude)+min_pressure;
      int error_check=1;
      float error_array[20];
      for (int i=0; i<20; i++)
      {
        error_array[i]=5;
      }
      
      while(true)
      {
        float voltage=analogRead(pressurepin[i]);

        float constants[8]={-1.74318228544700e-13,  1.68213037558633e-12,  -6.71663313801535e-12,
        1.43146154286253e-11,  -1.74961669337208e-11, 1.22078888318533e-11,  37.5657400450744,  -19.0458302028543};
        voltage=voltage*0.507/calib_volt;
        float read_pressure=(constants[0]*pow(voltage,7))+(constants[1]*pow(voltage,6))+(constants[2]*pow(voltage,5))+
        (constants[3]*pow(voltage,4))+(constants[4]*pow(voltage,3))+(constants[5]*pow(voltage,2))
        +(constants[6]*(voltage))+constants[7];
        
        previous_error=error;
        error=set_pressure-read_pressure;
        if(read_pressure>set_pressure)
        {
          digitalWrite(solenoid[i],HIGH);
        }
        else
        {
          digitalWrite(solenoid[i],LOW);
        }
        for(int j=0; j<numOscillators; j++)
        {
          if(j==0)
          {
            digitalWrite(solenoid[j],LOW);
          }
          if(j==2)
          {
            digitalWrite(solenoid[j],HIGH);
          }
        }

        
        
        double deriv_error=(error-previous_error)/T;
        //tune
        
        float Kp=0.14;
        float Kd=0.07;
        
        servo_desired_angle=37+Kp*error+Kd*deriv_error;
        servo_desired_angle=125-servo_desired_angle;
        myServo[i].write(servo_desired_angle);
        /*
        Serial.print(set_pressure);
        Serial.print(",");
        Serial.println(read_pressure);
        Serial.print(",");
        */
     
        delay(T*1000);
        for(int k=0; k<20; k++)
        {
          if(abs(error_array[i])>tolerance)
          {
             error_check=0;
             for(int k=0; k<19;k++)
             {
              error_array[k]=error_array[k+1];
             }
             error_array[19]=error;
          }
        }
        if(error_check==1)
        {
          break;
        }
        error_check=1;
       }     
       return;
}


void PD_S2(float theta, float amplitude, int i) {
         
      float T=0.01;
      
      float calib_volt=97;

      float tolerance=1;
      float error=5;
      float previous_error=0;
      float servo_desired_angle=0;
      float min_pressure=3;
      float max_pressure=4;
      float set_pressure=(max_pressure-min_pressure)*(theta+amplitude)/(2*amplitude)+min_pressure;
      int error_check=1;
      float error_array[20];
      for (int i=0; i<20; i++)
      {
        error_array[i]=5;
      }
      
      while(true)
      {
        float voltage=analogRead(pressurepin[i]);

        float constants[8]={-1.74318228544700e-13,  1.68213037558633e-12,  -6.71663313801535e-12,
        1.43146154286253e-11,  -1.74961669337208e-11, 1.22078888318533e-11,  37.5657400450744,  -19.0458302028543};
        voltage=voltage*0.507/calib_volt;
        float read_pressure=(constants[0]*pow(voltage,7))+(constants[1]*pow(voltage,6))+(constants[2]*pow(voltage,5))+
        (constants[3]*pow(voltage,4))+(constants[4]*pow(voltage,3))+(constants[5]*pow(voltage,2))
        +(constants[6]*(voltage))+constants[7];
        
        previous_error=error;
        error=set_pressure-read_pressure;
        if(read_pressure>set_pressure)
        {
          digitalWrite(solenoid[i],LOW);
        }
        else
        {
          digitalWrite(solenoid[i],HIGH);
        }
        for(int j=0; j<numOscillators; j++)
        {
          if(j!=i)
          {
            digitalWrite(solenoid[j],LOW);
          }
        }

        
        
        double deriv_error=(error-previous_error)/T;
        //tune
        
        float Kp=0.08;
        float Kd=0.02;
        
        servo_desired_angle=36+Kp*error+Kd*deriv_error;
        servo_desired_angle=125-servo_desired_angle;
        myServo[i].write(servo_desired_angle);
        /*
        Serial.print(set_pressure);
        Serial.print(",");
        Serial.println(read_pressure);
        Serial.print(",");
        */
     
        delay(T*1000);
        for(int k=0; k<20; k++)
        {
          if(abs(error_array[i])>tolerance)
          {
             error_check=0;
             for(int k=0; k<19;k++)
             {
              error_array[k]=error_array[k+1];
             }
             error_array[19]=error;
          }
        }
        if(error_check==1)
        {
          break;
        }
        error_check=1;
       }     
       return;
}
