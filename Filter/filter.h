// code for computing orientation and position of AltIMU 10 V5
// utilizes a low-pass filter
// Roll, Pitch, Yaw - X, Y, Z

#include <Wire.h> // I2C lib
#include <LSM6.h> // Accel and Gyro lib
#include <LIS3MDL.h> // Magnetometer lib

LSM6 gyroAcc; // creating accelerometer/gyro object
LIS3MDL Mag; // Creating magnetometer object
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

float ax, ay, az;
float mx, my, mz;
float wx, wy, wz;
float roll, pitch, yaw;
float scaleA, scaleG, scaleM;
float magOffset[3], magGain[3];

void IMUinit()
{
  Wire.begin();
  if (!gyroAcc.init() || !Mag.init())
   {
     Serial.println("Failed to detect and initialize IMU");
     while (1); // preserves error message on serial monitor
   }
  gyroAcc.enableDefault();
  Mag.enableDefault();
}

float getRoll()
{
  gyroAcc.read();
  scaleG = 4.375;
  
  roll = gyroAcc.g.x*scaleG/1000.0;

  return roll;
}

float getPitch()
{
  gyroAcc.read();
  scaleG = 4.375;
  
  pitch = gyroAcc.g.y*scaleG/1000.0;

  return pitch;
}

float getYaw()
{
  gyroAcc.read();
  scaleG = 4.375;
  
  yaw = gyroAcc.g.z*scaleG/1000.0;

  return yaw;
}



float getAccX()
{
  gyroAcc.read();
  scaleA = 0.061;
 
  ax = gyroAcc.a.x*scaleA/100.0;

  return ax;
}

float getAccY()
{
  gyroAcc.read();
  scaleA = 0.061;
 
  ay = gyroAcc.a.y*scaleA/100.0;

  return ay;
}

float getAccZ()
{
  gyroAcc.read();
  scaleA = 0.061;
 
  az = gyroAcc.a.z*scaleA/100.0;

  return az;
}

float getMx()
{
  Mag.read();
  scaleM = 6842.0;

  mx = Mag.m.x/scaleM;

  return mx;
}

float getMy()
{
  Mag.read();
  scaleM = 6842.0;

  my = Mag.m.y/scaleM;

  return my;
}

float getMz()
{
  Mag.read();
  scaleM = 6842.0;

  mz = Mag.m.z/scaleM;

  return mz;
}

float phiAM(float aY, float aZ)
{
  float PhiAM = atan2(-aY, -aZ);

  return PhiAM;
}

float thetaAM(float aX, float aY, float aZ)
{
  
  float ThetaAM = atan2(aX, sqrt(sq(aY) + sq(aZ)));
  if(ThetaAM > 3.1415926/2)
  {
    ThetaAM = 1.39626;
  }
  else if(ThetaAM < -3.1415926/2)
  {
    ThetaAM = -1.39626;
  }
  return ThetaAM;
}

float psiAM(float mX, float mY, float mZ, float theta, float phi)
{
  float PsiAM = atan2(mX*cos(theta) + mY*sin(theta)*sin(phi) + mZ*sin(theta)*cos(phi), mY*cos(phi) - mZ*sin(phi));

  return PsiAM;
}

float phiGy(float gX, float gY, float gZ, float thetaIC, float phiIC, float dt)
{
  float phiDot = gX +(gZ*cos(phiIC) + gY*sin(phiIC))*tan(thetaIC);
  float PhiGy = phiIC + phiDot*dt;

  return PhiGy;
}

float thetaGy(float gY, float gZ, float thetaIC, float phiIC, float dt)
{
  float thetaDot = gY*cos(phiIC) - gZ*sin(phiIC);
  float ThetaGy = thetaIC + thetaDot*dt;
  if(ThetaGy > 3.1415926/2)
  {
    ThetaGy = 1.39626;
  }
  else if(ThetaGy < -3.1415926/2)
  {
    ThetaGy = -1.39626;
  }
  return ThetaGy;
}

float psiGy(float gY, float gZ, float thetaIC, float phiIC, float psiIC, float dt)
{
  float psiDot = (gZ*cos(phiIC) + gY*sin(phiIC))/cos(thetaIC);
  float PsiGy = psiIC + psiDot*dt;

  return PsiGy;
}

void magCalib()
{

  scaleM = 6842.0;
  delay(2000);
  Serial.println("Magnetometer Calibration - Please rotate IMU");
  delay(3000);

  int k = 0;
  while(k < 50000)
  {
    Mag.read();
    running_min.x = min(running_min.x, Mag.m.x);
    running_min.y = min(running_min.y, Mag.m.y);
    running_min.z = min(running_min.z, Mag.m.z);
  
    running_max.x = max(running_max.x, Mag.m.x);
    running_max.y = max(running_max.y, Mag.m.y);
    running_max.z = max(running_max.z, Mag.m.z);

    k++;
  }

//  running_min.x = -7287.0; // found calib values so hard coded
//  running_min.y = -3241.0;
//  running_min.z = -4801.0;
//
//  running_max.x = 2911.0;
//  running_max.y = 3188.0;
//  running_max.z = 3249.0;
  
  magOffset[0] = running_max.x/scaleM;
  magOffset[1] = running_max.y/scaleM;
  magOffset[2] = running_max.z/scaleM;

  magGain[0] = (running_min.x - running_max.x)/scaleM; //180.0;
  magGain[1] = (running_min.y - running_max.y)/scaleM; //180.0;
  magGain[2] = (running_min.z - running_max.z)/scaleM; //180.0;
  
  Serial.println("...Done");
  Serial.print("offsets: ");
  Serial.print(magOffset[0]);
  Serial.print(" ");
  Serial.print(magOffset[1]);
  Serial.print(" ");
  Serial.println(magOffset[2]);
  Serial.print("gains: ");
  Serial.print(magGain[0]);
  Serial.print(" ");
  Serial.print(magGain[1]);
  Serial.print(" ");
  Serial.println(magGain[2]);
  delay(5000);

  return;
}

//void DCoffset()
//{
////  Serial.println("Offset Calculation - Place IMU still");
////  delay(2000);
////  Serial.println("...Done");
////  delay(1000);
////  j = 0;
////  total = 500;
////  while(j <= total)
////  {
////    gyro[0] = deg2rad(getRoll()); //  - gyroOffset[0]
////    gyro[1] = deg2rad(getPitch()); // - gyroOffset[1]
////    gyro[2] = deg2rad(getYaw()); // - gyroOffset[2]
////    
////    acc[0] = getAccX();
////    acc[1] = getAccY();
////    acc[2] = getAccZ();
////    
////    mag[0] = (getMx()-magOffset[0])/magGain[0];
////    mag[1] = (getMy()-magOffset[1])/magGain[1];
////    mag[2] = (getMz()-magOffset[2])/magGain[2];
////    // 
////    // Serial.println(acc[2],3);
////    
////    Phi1 = phiAM(acc[1], acc[2]);
////    Theta1 = thetaAM(acc[0], acc[1], acc[2]);
////    Psi1 = psiAM(mag[0], mag[1], mag[2], Theta1, Phi1);
////    
////    
////    if(i < 1)
////    {
////     Phi2 = phiGy(gyro[0], gyro[1], gyro[2], Theta1, Phi1, dt);
////     Theta2 = thetaGy(gyro[1], gyro[2], Theta1, Phi1, dt);
////     Psi2 = psiGy(gyro[1], gyro[2], Theta1, Phi1, Psi1, dt);
////     i++;
////    }
////    else
////    {
////     Phi2 = phiGy(gyro[0], gyro[1], gyro[2], thIC, phIC, dt); // *(180/3.1415926);
////     Theta2 = thetaGy(gyro[1], gyro[2], thIC, phIC, dt);
////     Psi2 = psiGy(gyro[1], gyro[2], thIC, phIC, psIC, dt);
////    }
////    
////    
////    phIC = Phi2;
////    thIC = Theta2;
////    psIC = Psi2;
////    
////    Phi = rad2deg(Phi1*HPF) + Phi2*LPF;
////    Theta = rad2deg(Theta1*HPF) + Theta2*LPF;
////    Psi = rad2deg(Psi1*HPF) + Psi2*LPF;
////
////    phiSum = phiSum + Phi;
////    thetaSum = thetaSum + Theta;
////    psiSum = psiSum + Psi;
////
////    if(j  == total)
////    {
////      phiOffset = phiSum/j;
////      thetaOffset = thetaSum/j;
////      psiOffset = psiSum/j;
////    }
////    
////    j++;
////  }
//  return;
//}
