/* Tommy Le
 * Northern Illinois University, Mechanical Engineering Dept
 * 6/19/2022
 * This code uses an AltIMU 10 v5 from Pololu to estimate 
 * orientation. Currently, roll and pitch data are good
 * but yaw is giving problems. 
 */


// header files________________________
#include "filter.h"
#include "Math_stuff.h"

// variables___________________________
float gyro[3], acc[3], mag[3];
float Phi1, Theta1, Psi1;
float Phi2, Theta2, Psi2;
float phIC, thIC, psIC; 
float dt, t0 , tf;
float Phi, Theta, Psi;
float LPF = 0.98, HPF = 0.02;
float phiGySum, thetaGySum, psiGySum;
float phiAMSum, thetaAMSum, psiAMSum;
float phiGyOffset, thetaGyOffset, psiGyOffset;
float phiAMOffset, thetaAMOffset, psiAMOffset;
float magNorm, mxNorm, myNorm, mzNorm;
float magX, magY;
int i, j, total;



void setup() 
{

  Serial.begin(9600);
  IMUinit(); //initialize I2C
  magCalib(); // magnetometer calibration/comment after recording offsets and gains
  //DCoffset(); // calculate DC offsets

  thIC = 0;
  phIC = 0;
  psIC = 0;
  i = 0;
  t0 = millis();
  
//  phiGySum = 0;
//  thetaGySum = 0;
//  psiGySum = 0;
//  phiAMSum = 0;
//  thetaAMSum = 0;
//  psiAMSum = 0;
//  phiGyOffset = 0;
//  thetaGyOffset = 0;
//  psiGyOffset = 0;
//  phiAMOffset = 0;
//  thetaAMOffset = 0;
//  psiAMOffset = 0;

}

void loop() 
{
// gyro data________________________________________
 gyro[0] = deg2rad(getRoll()); //  - gyroOffset[0]
 gyro[1] = deg2rad(getPitch()); // - gyroOffset[1]
 gyro[2] = deg2rad(getYaw()); // - gyroOffset[2]

// Serial.print("Gyro: ");
// Serial.print(rad2deg(gyro[0]), 3);
// Serial.print(' ');
// Serial.print(rad2deg(gyro[1]), 3);
// Serial.print(' ');
// Serial.println(rad2deg(gyro[2]), 3);

// acc data_________________________________________
 acc[0] = getAccX();
 acc[1] = getAccY();
 acc[2] = getAccZ();

// Serial.print("Accel: ");
// Serial.print(acc[0], 3);
// Serial.print(' ');
// Serial.print(acc[1], 3);
// Serial.print(' ');
// Serial.println(acc[2], 3);

// mag data_________________________________________
 mag[0] = (getMx()-magOffset[0])/magGain[0];
 mag[1] = (getMy()-magOffset[1])/magGain[1];
 mag[2] = (getMz()-magOffset[2])/magGain[2];

// after first calib, uncomment below and use printed values as offset and gain
// mag[0] = getMx();//-0.34)/-1.20;
// mag[1] = getMy();//-0.27)/-0.78;
// mag[2] = getMz();//-0.40)/-1.03;

 magNorm = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
 mag[0] = mag[0]/magNorm;
 mag[1] = mag[1]/magNorm;
 mag[2] = mag[2]/magNorm;
 
 Serial.print("Mag: ");
 Serial.print(mag[0], 3);
 Serial.print(' ');
 Serial.print(mag[1], 3);
 Serial.print(' ');
 Serial.print(mag[2], 3);
 Serial.print(' ');
 Serial.println(sq(mag[0]) + sq(mag[1]) + sq(mag[2]));

// finding dt_______________________________________
 tf = millis();
 dt = (tf - t0)/1000.0;
 t0 = tf;
 //Serial.println(dt, 5);

 // finding angles from accel and mag___________
 Phi1 = phiAM(acc[1], acc[2]); // - phiAMOffset;
 Theta1 = thetaAM(acc[0], acc[1], acc[2]); // - thetaAMOffset;
 Psi1 = psiAM(mag[0], mag[1], mag[2], Theta1, Phi1); // - psiAMOffset;

// magX = mag[0]*cos(Theta1) + mag[1]*sin(Phi1)*sin(Theta1) + mag[2]*cos(Phi1)*sin(Theta1);
// magY = mag[1]*cos(Phi1) - mag[2]*sin(Phi1);
// yaw = atan2(-magY, magX);

 // finding angles from gyro
 // on first iter use angles found from accel and mag as ICs
 if(i < 1)
 {
   Phi2 = phiGy(gyro[0], gyro[1], gyro[2], Theta1, Phi1, dt);// - phiGyOffset;
   Theta2 = thetaGy(gyro[1], gyro[2], Theta1, Phi1, dt);// - thetaGyOffset;
   Psi2 = psiGy(gyro[1], gyro[2], Theta1, Phi1, Psi1, dt);// - psiGyOffset;
   i++;
   //Serial.println("first loop");
   
 }
 else
 {
   Phi2 = phiGy(gyro[0], gyro[1], gyro[2], thIC, phIC, dt);// - phiGyOffset; // *(180/3.1415926);
   Theta2 = thetaGy(gyro[1], gyro[2], thIC, phIC, dt);// - thetaGyOffset;
   Psi2 = psiGy(gyro[1], gyro[2], thIC, phIC, psIC, dt);// - psiGyOffset;
   //Serial.println("second loop");
 }

 // setting initial conditions__________________________
 phIC = Phi2;
 thIC = Theta2;
 psIC = Psi2;

 // fusion______________________________________________
 Phi = Phi1*LPF + Phi2*HPF; // - phiOffset;
 Theta = Theta1*LPF + Theta2*HPF; // - thetaOffset;
 Psi = Psi1*LPF + Psi2*HPF; // - psiOffset;

// printing accel and mag angles________________________
// Serial.print("A: ");
// Serial.print(rad2deg(Phi1), 3); //great
// Serial.print(' ');
// Serial.print(rad2deg(Theta1), 3); //good
// Serial.print(' '); 
// Serial.println(rad2deg(Psi1), 3); //can't get changes to read

// printing gyro angles_________________________________
// Serial.print("G: "); //drifts
// Serial.print(rad2deg(Phi2), 3);
// Serial.print(' ');
// Serial.print(rad2deg(Theta2), 3); //good
// Serial.print(' ');
// Serial.println(rad2deg(Psi2), 3); //drifts and can't get changes read

// printing filtered angles_____________________________
 Serial.print("F: ");
 Serial.print(rad2deg(Phi), 3);
 Serial.print(' ');
 Serial.print(rad2deg(Theta), 3);
 Serial.print(' ');
 Serial.println(rad2deg(Psi), 3);

// printing DC offset values_____________________________
// Serial.print("Offsets: ");
// Serial.print(phiOffset, 3);
// Serial.print(' ');
// Serial.print(thetaOffset, 3);
// Serial.print(' ');
// Serial.println(psiOffset, 3);

 delay(100);
}
