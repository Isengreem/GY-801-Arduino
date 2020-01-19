#include <Wire.h>

//Accel
#define ACCEL_ADDR 		0x53
#define ACC_POWER_CTL	0x2D
#define ACC_DATA_FORMAT	0x31  //D0 D1 range (00 2g) (01 4g) (10 8g) (11 16g)
#define ACC_X0 			0x32 
#define ACC_X1 			0x33 
#define ACC_Y0 			0x34 
#define ACC_Y1 			0x35
#define ACC_Z0 			0x36
#define ACC_Z1 			0x37

//Gyro
#define GYRO_ADDR		0x69
#define GYRO_CTRL_REG1	0x20
#define GYRO_CTRL_REG2	0x21
#define GYRO_CTRL_REG4	0x23
#define GYRO_X0 		0x28  
#define GYRO_X1 		0x29
#define GYRO_Y0 		0x2A
#define GYRO_Y1 		0x2B
#define GYRO_Z0 		0x2C  
#define GYRO_Z1 		0x2D

//Magneto
#define MAG_ADDR		0x30
#define MAG_STAT		0x07 //d0 = 1 when measurement is finished
#define MAG_IC0			0x08 //d0 magnetic measuring d1 temp measuring
#define MAG_IC1			0x09 //d0-d1 100/200/400/600 Hz (10ms/5ms/2.5ms/1.6ms)
#define MAG_IC2			0x0A
#define MAG_X0 			0x00 
#define MAG_X1 			0x01 
#define MAG_Y0 			0x02 
#define MAG_Y1 			0x03 
#define MAG_Z0 			0x04 
#define MAG_Z1 			0x05 
#define MAG_TEMP		0x06

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

void setup()
{
	Wire.begin();
	Serial.begin(9600);
	
	//setup accel
	Wire.beginTransmission(ACCEL_ADDR);
	Wire.write(ACC_POWER_CTL);
	Wire.write(8);//0000 1000 D3 measuring mode
	Wire.endTransmission();
	
	//setup accel range 8g full resolution
	Wire.beginTransmission(ACCEL_ADDR);
	Wire.write(ACC_DATA_FORMAT);
	Wire.write(0x0A); //0x0B 1011 16g //0x0A 1010 8g //0x09 1001 4g //0x08 1000 2g
	Wire.endTransmission();
	
	//setup gyro
	Wire.beginTransmission(GYRO_ADDR);
	Wire.write(GYRO_CTRL_REG1); // CTRL_REG1 - Power Mode
	Wire.write(15);   // Normal mode: 15d - 00001111b   
	Wire.endTransmission();
	
	Wire.beginTransmission(GYRO_ADDR);
	Wire.write(GYRO_CTRL_REG2); 
	Wire.write(0);   
	Wire.endTransmission();
	
	Wire.beginTransmission(GYRO_ADDR);
	Wire.write(GYRO_CTRL_REG4); // CTRL_REG4 - Sensitivity, Scale Selection
	Wire.write(48);   // 2000dps: 48d - 00110000b //00 250dps //01 500dps//10,11 2000dps
	Wire.endTransmission();	
	
	//setup magneto 
	Wire.beginTransmission(MAG_ADDR);
	Wire.write(MAG_IC0);  
	Wire.write(00001000); //set
	Wire.endTransmission();
	
	Wire.beginTransmission(MAG_ADDR);
	Wire.write(MAG_IC1);
	Wire.write(3); //output data rate
	Wire.endTransmission();
	
	Wire.beginTransmission(MAG_ADDR);
	Wire.write(MAG_IC2);
	Wire.write(4); //0100 1Hz
	Wire.endTransmission();
	
}

void loop()
{
	ax = getValue(ACCEL_ADDR, ACC_X0, ACC_X1);
	ay = getValue(ACCEL_ADDR, ACC_Y0, ACC_Y1);
	az = getValue(ACCEL_ADDR, ACC_Z0, ACC_Z1);
	ax = ax /256.0;
	ay = ay /256.0;
	az = az /256.0;
	
	gx = getValue(GYRO_ADDR, GYRO_X0, GYRO_X1);
	gy = getValue(GYRO_ADDR, GYRO_Y0, GYRO_Y1);
	gz = getValue(GYRO_ADDR, GYRO_Z0, GYRO_Z1);
	
	//deg/s
	gx = gx* 0.07; 
	gy = gy* 0.07; 
	gz = gz* 0.07;	
	
	measure();
	mx = getValue(MAG_ADDR, MAG_X0, MAG_X1);
	my = getValue(MAG_ADDR, MAG_Y0, MAG_Y1);
	mz = getValue(MAG_ADDR, MAG_Z0, MAG_Z1);
	
	//gauss
	mx = mx / 4096;
	my = my / 4096; 
	mz = mz / 4096;
	
	//serial output
	Serial.print(gx);
	Serial.print(",");
	Serial.print(gy);
	Serial.print(",");
	Serial.print(gz);
	Serial.print(",");
	Serial.print(ax);
	Serial.print(",");
	Serial.print(ay);
	Serial.print(",");
	Serial.print(az);
	Serial.print(",");
	Serial.print(mx);
	Serial.print(",");
	Serial.print(my);
	Serial.print(",");
	Serial.println(mz);
	
}

float getValue (int addDev, int addReg0, int addReg1)
{
	int outr;	
	int r0, r1;
	Wire.beginTransmission(addDev);
	Wire.write(addReg0);
	Wire.endTransmission();
	Wire.requestFrom(addDev,1);
	if(Wire.available()<=1)	r0 = Wire.read();
	Wire.beginTransmission(addDev);
	Wire.write(addReg1);
	Wire.endTransmission();
	Wire.requestFrom(addDev,1);
	if(Wire.available()<=1)	r1 = Wire.read();	
	r1 = r1 << 8;
	outr = r0+r1;
	return outr;
}

void measure()
{
	uint8_t stat = 0b00000000;
	//initiate measurement
	Wire.beginTransmission(MAG_ADDR);
	Wire.write(MAG_IC0);  
	Wire.write(0x01);
	Wire.endTransmission();

	//check status for measurement completion
	do
	{
		Wire.beginTransmission(MAG_ADDR);
		Wire.write(MAG_STAT);
		Wire.endTransmission(true);
		Wire.requestFrom(MAG_ADDR,1);
		stat = Wire.read();
		Wire.endTransmission();
	}
	while ((stat & 0b00000001) != 0b00000001);
}
