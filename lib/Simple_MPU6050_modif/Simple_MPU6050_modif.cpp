/* ============================================
  Simple_MPU6050 device library code is placed under the MIT license
  Copyright (c) 2021 Homer Creutz

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/
#include "Arduino.h"
#include <Wire.h>
#include "DMP_Image_modif.h"
#include "Simple_Wire_modif.h"
#include "Simple_MPU6050_modif.h"
#include "MPU_ReadMacros_modif.h"
#include "MPU_WriteMacros_modif.h"

// Blink Without Delay Serial Port Spam Timer Macro
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // A Way to do a Blink without Delay timer

//#define USE_OLD_GETYAWPITCHROLL // Calculation returns different values but possibly relevant for your project Try both out
// OLD Yaw +- 180, Pitch and Roll +- 90 (Peaks at 90 deg then fall back to zero, shows Negative when pointing down pitch and left roll)
// NEW Yaw +- 180, Pitch and Roll +- 180 (Continues to 180 deg then -180 back to zero, shows Negative when pointing down pitch and left roll)

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has changed
volatile uint8_t _maxPackets;

/**
@brief      Initialization functions
*/
#define Three_Axis_Low_Power_Quaternions 3
#define Six_Axis_Low_Power_Quaternions 6  // Default
Simple_MPU6050::Simple_MPU6050(uint8_t DMPMode) {

    _DMPMode = DMPMode;
	SetAddress(MPU6050_DEFAULT_ADDRESS);
	packet_length = 28;
	/*
	packet_length = 0;
	packet_length += 6;//DMP_FEATURE_SEND_RAW_ACCEL
	packet_length += 6;//DMP_FEATURE_SEND_RAW_GYRO
	packet_length += 16;//DMP_FEATURE_6X_LP_QUAT
    */
    packet_length += 4; //DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT
	
	_maxPackets = floor(512 / packet_length); // MPU 9250 can only handle 512 bytes of data in the FIFO
}

/**
@brief      Set Device Address
*/

Simple_MPU6050 &  Simple_MPU6050::SetAddress(uint8_t address) {
	devAddr = address;
	return *this;
}


/**
@brief      Returns Device Address
*/
uint8_t  Simple_MPU6050::CheckAddress() {
	return devAddr;
}

/**
@brief      Set FIFO Callback
*/
Simple_MPU6050 & Simple_MPU6050::on_FIFO(void (*CB)(int16_t *, int16_t *, int32_t *)) {
	on_FIFO_cb = CB;
	return *this; // return Pointer to this class
}

/**
@brief      Reset funnctions
*/
Simple_MPU6050 & Simple_MPU6050::reset_fifo() {
	USER_CTRL_WRITE_FIFO_RST(); //   Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::full_reset_fifo(void) { // Official way to reset fifo
	USER_CTRL_WRITE_RESET_FIFO_DMP();
	return *this;
}

/**
@brief      Start and Stop DMP int pin triggering  //   1  Enable, 0 = Disable
*/
Simple_MPU6050 & Simple_MPU6050::DMP_InterruptEnable(uint8_t Data) {
	INT_ENABLE_WRITE_RAW_DMP_INT_EN(Data);
	dmp_on = Data;
	return *this;
};

//***************************************************************************************
//**********************      Interrupt Functions        **********************
//***************************************************************************************
/**
@brief      manages properly testing interrupt trigger from interrupt pin
*/
uint8_t Simple_MPU6050::CheckForInterrupt(void) {
	uint8_t InterruptTriggered;
	noInterrupts ();
	InterruptTriggered = mpuInterrupt;
	mpuInterrupt = false;
	interrupts ();
	return InterruptTriggered;
}

//***************************************************************************************
//**********************              FIFO functions               **********************
//***************************************************************************************
/**
@brief      Reads Newest packet from fifo then on success triggers Callback routine
returns true if packet is captured and stored in gyro[3], accel[3], quat[4] variables 
*/
uint8_t Simple_MPU6050::dmp_read_fifo(uint8_t CheckInterrupt) {
	if (CheckInterrupt && !CheckForInterrupt()) return 0;
	if (!dmp_read_fifo(gyro, accel, quat, &sensor_timestamp)) {
		return 0;
	}
	if (on_FIFO_cb) on_FIFO_cb(gyro, accel, quat);
	return 1;
}

/* Compare the above dmp_read_fifo function to the below dmp_read_fifo function.
 * The above function triggers a callback function only when the data is retrieved 
 * This allows you to define a specific function at the time of setup to be used
 * The result code is a simpler setup
 * you could use the one below like
 * if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp)) { Do Something;}
 * to get similar results 
 */
/**
@brief      Get the Newest packet from the FIFO. FIFO Buffer will be empty awaiting for next packet
*/
uint8_t Simple_MPU6050::dmp_read_fifo(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
	/* Get a packet. */
	uint8_t fifo_data[MAX_PACKET_LENGTH];
	if(GetCurrentFIFOPacket(fifo_data, packet_length)){

	//	FIFO_READ(packet_length, fifo_data);
		*timestamp = micros();
	//	fifo_count -= packet_length;
		/* Parse DMP packet. */
		uint8_t ii = 0;
		quat[0] = ((int32_t)fifo_data[0] << 24) | ((int32_t)fifo_data[1] << 16) | ((int32_t)fifo_data[2] << 8) | fifo_data[3];
		quat[1] = ((int32_t)fifo_data[4] << 24) | ((int32_t)fifo_data[5] << 16) | ((int32_t)fifo_data[6] << 8) | fifo_data[7];
		quat[2] = ((int32_t)fifo_data[8] << 24) | ((int32_t)fifo_data[9] << 16) | ((int32_t)fifo_data[10] << 8) | fifo_data[11];
		quat[3] = ((int32_t)fifo_data[12] << 24) | ((int32_t)fifo_data[13] << 16) | ((int32_t)fifo_data[14] << 8) | fifo_data[15];
		ii += 16;
		accel[0] = ((int16_t)fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
		accel[1] = ((int16_t)fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
		accel[2] = ((int16_t)fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
		ii += 6;
		gyro[0] = ((int16_t)fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
		gyro[1] = ((int16_t)fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
		gyro[2] = ((int16_t)fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
		return 1;
	}
	return 0;
}
 
/**
@brief      Gets FIFO byte count
*/
 int16_t Simple_MPU6050::getFIFOCount(){
	int16_t fifo_count;
	FIFO_COUNTH_READ_FIFO_CNT(&fifo_count);
	return (fifo_count);
 }


/** 
 @brief Get latest byte from FIFO buffer no matter how much time has passed.
*/
/* ================================================================
 * ===                  GetCurrentFIFOPacket                    ===
 * ================================================================
 * Returns 1) when nothing special was done
 *         0) when no valid data is available
 * ================================================================ */
 int8_t Simple_MPU6050::GetCurrentFIFOPacket(uint8_t *data, uint8_t length) { // overflow proof
     int16_t fifoC;
     // This section of code is for when we allowed more than 1 packet to be acquired
     uint32_t BreakTimer = micros();
     do {
         if ((fifoC = getFIFOCount())  > length) {

             if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
                 USER_CTRL_WRITE_FIFO_RST(); // Fixes any overflow corruption
                 fifoC = 0;
                 while (!(fifoC = getFIFOCount()) && ((micros() - BreakTimer) <= (11000))); // Get Next New Packet
             } else { //We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
                 uint8_t Trash[WIRE_BUFFER_LENGTH];
                 while ((fifoC = getFIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
                     fifoC = fifoC - length; // Save the last packet
                     uint16_t  RemoveBytes;
                     while (fifoC) { // fifo count will reach zero so this is safe
                         RemoveBytes = min((int)fifoC, WIRE_BUFFER_LENGTH); // Buffer Length is different than the packet length this will efficiently clear the buffer
//                        getFIFOBytes(Trash, (uint8_t)RemoveBytes);
						 FIFO_READ((uint8_t)RemoveBytes, Trash);
                         fifoC -= RemoveBytes;
                     }
                 }
             }
         }
         if (!fifoC) return 0; // Called too early no data or we timed out after FIFO Reset
         // We have 1 packet
         if ((micros() - BreakTimer) > (11000)) return 0;
     } while (fifoC != length);
	 FIFO_READ((uint8_t)length, data);  //Get 1 packet
//     getFIFOBytes(data, length); //Get 1 packet
     return 1;
}

//***************************************************************************************
//**********************      Firmware Read Write Functions        **********************
//***************************************************************************************

/**
@brief      Read and Write to the DMP FIRMWARE memory. using these functions alters the firmware instance
*/

Simple_MPU6050 & Simple_MPU6050::read_mem(uint16_t mem_addr, uint16_t length, uint8_t *Data) {

	BANK_SEL_WRITE(mem_addr);
	DMP_MEM_READ(length, Data);
	return *this;
	} 

Simple_MPU6050 & Simple_MPU6050::write_mem(uint16_t  mem_addr, uint16_t  length, uint8_t *Data) {
	BANK_SEL_WRITE(mem_addr);
	DMP_MEM_WRITE(length, Data);
	return *this;
}


//***************************************************************************************
//**********************              Setup Functions              **********************
//***************************************************************************************
/**
@brief      ***EVERYTHING!*** needed to get DMP up and running!
*/

/*Simple_MPU6050 & Simple_MPU6050::Set_DMP_Output_Rate(uint8_t byteH, uint8_t byteL){  // 100 HZ Default
	DMP_Output_Rate[0] = byteH;
	DMP_Output_Rate[1] = byteL;
	return *this;
}
*/
Simple_MPU6050 & Simple_MPU6050::Set_DMP_Output_Rate(uint16_t value){  // 100 HZ Default
	DMP_Output_Rate[0] = (uint8_t)((value & 0xFF00) >> 8);
	DMP_Output_Rate[1] = (uint8_t)(value & 0x00FF);
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::Set_DMP_Output_Rate_Hz(float rate){  // 100 HZ Default

    rate = (rate>200) ? 200 : rate;
    rate = (rate<0) ? 0.0032 : rate;
	uint16_t div = (uint16_t)( 200 / rate - 1);
	Set_DMP_Output_Rate(div);
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::Set_DMP_Output_Rate_Seconds(float rate){  // 1Hz Default
    rate = (rate>300) ? 300 : rate;
    rate = (rate<0) ? 0 : rate;
	uint16_t div = (uint16_t)( 200 / (1/rate) - 1);
	Set_DMP_Output_Rate(div);
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::Set_DMP_Output_Rate_Minutes(float rate){  // 1 minute Default
    rate = (rate>5) ? 5 : rate;
    rate = (rate<0) ? 0 : rate;
	Set_DMP_Output_Rate_Seconds(rate/60);
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::load_DMP_Image(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_) {
	sax_ = ax_;
	say_ = ay_;
	saz_ = az_;
	sgx_ = gx_;
	sgy_ = gy_;
	sgz_ = gz_;
	
	TestConnection();
	PWR_MGMT_1_WRITE_DEVICE_RESET();				//PWR_MGMT_1:(0x6B Bit7 true) reset with 100ms delay and full SIGNAL_PATH_RESET:(0x6A Bits 3,2,1,0 True) with another 100ms delay
	WriteByte(0x6B, 0x00);
	WriteByte(0x6C, 0x00);
	WriteByte(0x1A, 0x03);
	//WriteByte(0x1B, 0x00);	//gyro 250 degree/sec
	//WriteByte(0x1B, 0x08);	//gyro 500 degree/sec
	WriteByte(0x1B, 0x18);	//gyro 2000 degree/sec
	WriteByte(0x1C, 0x00);
	WriteByte(0x23, 0x00);
	WriteByte(0x38, 0x00);
	WriteByte(0x6A, 0x04);
	WriteByte(0x19, 0x04);

	load_firmware(DMP_CODE_SIZE, dmp_memory);	// Loads the DMP image into the MPU6050 Memory
	write_mem(D_0_22, 2, DMP_Output_Rate);      // Modify the Firmware Chunk for DMP output Rate  
	WriteInt(0x70,  0x0400);				// DMP Program Start Address

	resetOffset();	

	//PrintActiveOffsets();
	WriteByte(0x6A, 0xC0);					// 1100 1100 USER_CTRL: Enable FIFO and Reset FIFO
	WriteByte(0x38, 0x02);					// 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
	dmp_on = 1;
#ifdef interruptPin
    Interupt_Attach_Function
#endif
	//These are the features the above code initialized for you by default (ToDo Allow removal of one or more Features)
	dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO |  DMP_FEATURE_SEND_CAL_GYRO; // These are Fixed into the DMP_Image and Can't be change easily at this time.
	return *this;
}

/**
@brief      Resets the DMP firmware lockdown to allow firmware to be loaded again.
*/
Simple_MPU6050 &  Simple_MPU6050::Enable_Reload_of_DMP(uint8_t DMPMode) {
    _DMPMode = DMPMode;
	DMP_Loaded = false;
	return *this;	
}
/**
@brief      Loads the DMP firmware.
*/
Simple_MPU6050 & Simple_MPU6050::load_firmware(uint16_t  length, const uint8_t *firmware) {
	
	uint16_t  ii;
	uint16_t  this_write;
	#define LOAD_CHUNK  (16)
	#ifdef DEBUG
	uint8_t cur[LOAD_CHUNK];
	uint16_t bankNum = 0;
	#endif
	/* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
	uint8_t firmware_chunk[LOAD_CHUNK];
	if (DMP_Loaded)return *this; /* DMP should only be loaded once. */
	if (!firmware) return *this;

	for (ii = 0; ii < length; ii += this_write) {
		this_write = min(LOAD_CHUNK, length - ii);
		uint16_t x;
		uint8_t *pFirmware = (uint8_t *)&firmware[ii];
		for ( x = 0; x < this_write; x++ ) firmware_chunk[x] = pgm_read_byte_near(pFirmware + x);
		write_mem(ii, this_write, firmware_chunk);
		#ifdef DEBUG
		// this displays the firmware by retrieving it from the MPU6050 after it was written Then Sending it to the serial port
		read_mem(ii, this_write, cur);
		if ((ii % (16 * 16)) == 0) {
			Serial.print(F("/* bank # "));
			Serial.print(bankNum++);
			Serial.println(F(" */"));
		}
		for (uint16_t c = 0; c < this_write; c++) {
			Serial.print(F(" 0x"));
			Serial.print(cur[c] >> 4, HEX); //Prints 0 instead of nothing when byte is less than 8
			Serial.print(cur[c] & 0X0F, HEX); // Prints the remainder of the hex number
			Serial.print(F(","));
		}
		Serial.println();
		#endif
	}
	#ifdef DEBUG
	Serial.println();
	#endif
	DMP_Loaded = true;
	return *this;
}

/**
@brief      Test to be sure we have communication to the MPU
returns 1 on success
stops or returns 0 on fail
*/
void Simple_MPU6050::TestConnection() {
	Wire.beginTransmission(CheckAddress());
	Wire.endTransmission();
	WHO_AM_I_READ_WHOAMI(&WhoAmI);
}


//***************************************************************************************
//********************** Gather Configuration from working MPU6050 **********************
//***************************************************************************************

// usage after configuration of the MPU6050 to your liking Get these registers to simplify MPU6050 startup

Simple_MPU6050 &  Simple_MPU6050::GetActiveOffsets(int16_t* Data) {
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	if(WhoAmI < 0x38)  A_OFFSET_H_READ_A_OFFS(Data);
	else {
		XA_OFFSET_H_READ_0x77_XA_OFFS(Data);
		YA_OFFSET_H_READ_0x77_YA_OFFS(Data+1);
		ZA_OFFSET_H_READ_0x77_ZA_OFFS(Data+2);
	}
	XG_OFFSET_H_READ_OFFS_USR(Data + 3);
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::PrintActiveOffsets( ) {
	int16_t Data[3];
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	Serial.print(F("\n//              X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro\n#define OFFSETS "));
	if(WhoAmI < 0x38)	A_OFFSET_H_READ_A_OFFS(Data);
	else {
		XA_OFFSET_H_READ_0x77_XA_OFFS(Data);
		YA_OFFSET_H_READ_0x77_YA_OFFS(Data+1);
		ZA_OFFSET_H_READ_0x77_ZA_OFFS(Data+2);
	}
	Serial.printfloatx("", Data[0], 5, 0, ",  ");
	Serial.printfloatx("", Data[1], 5, 0, ",  ");
	Serial.printfloatx("", Data[2], 5, 0, ",  ");

	XG_OFFSET_H_READ_OFFS_USR(Data);
	Serial.printfloatx("", Data[0], 5, 0, ",  ");
	Serial.printfloatx("", Data[1], 5, 0, ",  ");
	Serial.printfloatx("", Data[2], 5, 0, "");

	Serial.println();
	return *this;
}


//***************************************************************************************
//**********************           Calibration Routines            **********************
//***************************************************************************************

/**
@brief      ***EVERYTHING!*** needed to get DMP up and running! With Calibration!!!
*/

Simple_MPU6050 & Simple_MPU6050::resetOffset() {
	setOffset( sax_,  say_,  saz_,  sgx_,  sgy_,  sgz_);
	return *this; // return Pointer to this class
}

Simple_MPU6050 & Simple_MPU6050::setOffset(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_) {
	sax_ = ax_;
	say_ = ay_;
	saz_ = az_;
	sgx_ = gx_;
	sgy_ = gy_;
	sgz_ = gz_;
	
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	if(WhoAmI < 0x38){
		XA_OFFSET_H_WRITE_XA_OFFS(ax_);
		YA_OFFSET_H_WRITE_YA_OFFS(ay_);
		ZA_OFFSET_H_WRITE_ZA_OFFS(az_);
		} else {
		XA_OFFSET_H_WRITE_0x77_XA_OFFS(ax_);
		YA_OFFSET_H_WRITE_0x77_YA_OFFS(ay_);
		ZA_OFFSET_H_WRITE_0x77_ZA_OFFS(az_);
	}

	XG_OFFSET_H_WRITE_X_OFFS_USR(gx_);
	YG_OFFSET_H_WRITE_Y_OFFS_USR(gy_);
	ZG_OFFSET_H_WRITE_Z_OFFS_USR(gz_);
	return *this;
}
//***************************************************************************************
//**********************          Helper Math Functions            **********************
//***************************************************************************************

Simple_MPU6050 &  Simple_MPU6050::SetAccel(VectorInt16 *v, int16_t *accel) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	v -> x = accel[0];
	v -> y = accel[1];
	v -> z = accel[2];
	return *this;
}

Simple_MPU6050 &  Simple_MPU6050::GetQuaternion(Quaternion *q, const int32_t* qI) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	q -> w = (float)(qI[0] >> 16) / 16384.0f;
	q -> x = (float)(qI[1] >> 16) / 16384.0f;
	q -> y = (float)(qI[2] >> 16) / 16384.0f;
	q -> z = (float)(qI[3] >> 16) / 16384.0f;
	return *this;
}

Simple_MPU6050 &  Simple_MPU6050::GetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
	// get rid of the gravity component (+1g = +16384 in standard DMP FIFO packet, sensitivity is +-2g)
	v -> x = vRaw -> x - gravity -> x * 16384;
	v -> y = vRaw -> y - gravity -> y * 16384;
	v -> z = vRaw -> z - gravity -> z * 16384;
	return *this;
}

Simple_MPU6050 &  Simple_MPU6050::GetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
	// rotate measured 3D acceleration vector into original state
	// frame of reference based on orientation quaternion
	memcpy(v, vReal, sizeof(VectorInt16));
	v -> rotate(q);
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::GetGravity(VectorFloat *v, Quaternion *q) {
	v -> x = 2 * (q -> x * q -> z - q -> w * q -> y);
	v -> y = 2 * (q -> w * q -> x + q -> y * q -> z);
	v -> z = q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z;
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::GetEuler(float *data, Quaternion *q) {
	data[0] = atan2(2 * q -> x * q -> y - 2 * q -> w * q -> z, 2 * q -> w * q -> w + 2 * q -> x * q -> x - 1); // psi
	data[1] = -asin(2 * q -> x * q -> z + 2 * q -> w * q -> y);                      // theta
	data[2] = atan2(2 * q -> y * q -> z - 2 * q -> w * q -> x, 2 * q -> w * q -> w + 2 * q -> z * q -> z - 1); // phi
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
	#ifdef USE_OLD_GETYAWPITCHROLL
	// yaw: (about Z axis)
	data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
	#else 
	// yaw: (about Z axis)
	data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan2(gravity -> y , gravity -> z);
	if (gravity -> z < 0) {
		if(data[1] > 0) {
			data[1] = PI - data[1];
			} else {
			data[1] = -PI - data[1];
		}
	}
	#endif
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::GetYawPitchRoll(float *data, Quaternion *q) {
	data[0] = atan2(2.0f * (q -> x*q -> y + q -> w * q -> z), q -> w * q -> w + q -> x * q -> x - q -> y * q -> y - q -> z * q -> z);
	data[1] = -asin(2.0f * (q -> x * q -> z - q -> w * q -> y));
	data[2]  = atan2(2.0f * (q -> w * q -> x + q -> y * q -> z), q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z);
	return *this;
}




Simple_MPU6050 & Simple_MPU6050::ConvertToDegrees(float*ypr, float*xyz) {
	//const float radians_to_degrees = 180.0 / M_PI;
	for (int i = 0; i < 3; i++) {
		xyz[i] = ypr[i] * radians_to_degrees;

	}
	if ( xyz[0] < -180 ) xyz[0] += 360;
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::ConvertToRadians( float*xyz, float*ypr) {
	const float degrees_to_radians = M_PI / 180.0;
	for (int i = 0; i < 3; i++) ypr[i] = xyz[i] * degrees_to_radians;
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::MagneticNorth(float*data, VectorInt16 *v, Quaternion*q ) {
	float ax = v->x, ay = v->y, az = v->z;
	float q1 = q->w, q2 = q->x, q3 = q->y, q4 = q->z;   // short name local variable for readability
	float mx = mag[0], my = mag[1], mz = mag[2];
	float hx, hy, bx, bz,vx,vy,vz,wx,wy,wz,ex,ey,ez;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;


	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	data[0] = wx;
	data[1] = wy;
	data[2] = wz;
	data[3] = ex;
	data[4] = ey;
	data[5] = ez;
	return *this;
}



//***************************************************************************************
//**********************      Helper Magnetometer Functions       **********************
//***************************************************************************************
Simple_MPU6050 & Simple_MPU6050::I2CScanner(){
	for (int x = 1;x < 128;x++){
		if(!(x = FindAddress(x,128))) break;
	}
	return *this;
}




uint8_t Simple_MPU6050::FindAddress(uint8_t Address,uint8_t Limit){
	do {
		Wire.beginTransmission(Address);
		if (Wire.endTransmission() == 0)
		return Address;
	} while (Limit != Address++);// using rollover ate 255 to allow for any number on Limit
	return 0;
}

Simple_MPU6050 & Simple_MPU6050::mpu_set_bypass(unsigned char bypass_on){
	if (bypass_on) {
		USER_CTRL_WRITE_I2C_MST_EN(0);
		delay(3);
		INT_PIN_CFG_WRITE_ACTL(0);				//7
		INT_PIN_CFG_WRITE_OPEN(0);				//6
		INT_PIN_CFG_WRITE_FSYNC_INT_MODE_EN(0);	//2
		INT_PIN_CFG_WRITE_BYPASS_EN(1);			//1
		} else {
		/* Enable I2C master mode if compass is being used. */
		USER_CTRL_WRITE_I2C_MST_EN(akm_addr>0);
		delay(3);
		INT_PIN_CFG_WRITE_ACTL(0);				//7
		INT_PIN_CFG_WRITE_OPEN(0);				//6
		INT_PIN_CFG_WRITE_FSYNC_INT_MODE_EN(0);	//2
		INT_PIN_CFG_WRITE_BYPASS_EN(0);			//1
	}
	return *this;
}


bool Simple_MPU6050::readMagData(){
	return readMagData(mag);
}
bool Simple_MPU6050::readMagData(float *magData){
	//read mag
	static unsigned long XTimer;
	static uint8_t Skip;
	if(!Skip){
		AKM_CNTL_WRITE_SINGLE_MEAS_MODE(0x0C,1);// Single Measurement mode with High res 16Bit output
		Skip = 1;
		XTimer= millis();
	}
	if ( millis() - XTimer < (10)) return 0; // after triggering the Single Measurement mode it takes 10ms max to gather new data
	Skip = 0;// we will need to reset the delay timer before gathering new data
	int16_t RawCompass[3];

	AKM_DATA_READ_RAW_COMPASS_SWAP(akm_addr,RawCompass);
	magData[0] = (float)RawCompass[0];
	magData[1] = (float)RawCompass[1];
	magData[2] = (float)RawCompass[2];
	AKM_CNTL_WRITE_SINGLE_MEAS_MODE(0x0C,1);
   
	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	mRes = (mRes!=0)?mRes:1;
	magData[0] = (float)magData[0]*mRes*mag_sens_adj_F[0] - mag_bias[0];  // get actual magnetometer value, this depends on scale being set
	magData[1] = (float)magData[1]*mRes*mag_sens_adj_F[1] - mag_bias[1];
	magData[2] = (float)magData[2]*mRes*mag_sens_adj_F[2] - mag_bias[2];

	if(mag_scale[0]!=0) magData[0] *= mag_scale[0];
	if(mag_scale[1]!=0) magData[1] *= mag_scale[1];
	if(mag_scale[2]!=0) magData[2] *= mag_scale[2];


	// Normalise magnetometer measurement
	float nmag = sqrt(magData[0] * magData[0] + magData[1] * magData[1] + magData[2] * magData[2]);
	if (nmag == 0.0f) return 0; // handle NaN
	nmag = 1.0f / nmag;        // use reciprocal for division
	magData[0] *= nmag;
	magData[1] *= nmag;
	magData[2] *= nmag;
	
	return true;
}


// I have not proven this to be correct.
Simple_MPU6050 & Simple_MPU6050::magcalMPU(){
//https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
//https://appelsiini.net/2018/calibrate-magnetometer/
	uint8_t DelayCnt = 5;
	uint8_t Ready;
	uint16_t sample_count = 0, PCount = 0;
//	int32_t mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767};
	int16_t mag_min[3] = {32767, 32767, 32767};
	int16_t mag_temp[3] = {0, 0, 0};
	 
	Serial.println(F("Mag Calibration: Wave device in a figure eight until done! @ 2Minutes"));
	delay(1000);
	Serial.println(F("Ready"));
	delay(1000);
	Serial.println(F("Set!"));
	delay(2000);
	Serial.println(F("GO! GO! GO!"));
	//AKM_CNTL_WRITE_CONT_MEAS_MODE2(akm_addr,1); // only works with mpu9250 por mpu9255
	AKM_CNTL_WRITE_SINGLE_MEAS_MODE(akm_addr,1);

	
	static unsigned long _ETimer;
	_ETimer = millis();
	while ( millis() - _ETimer <= (60000 * 2)) {	// shoot for ~fifteen seconds of mag data
		delay(DelayCnt); // Lets wait instead of bugging the MPU
		while(DelayCnt < 135){
			delay(1);
			AKM_ST1_READ_DATA_READY(akm_addr,&Ready); // data ready
			if(Ready)break;
			DelayCnt++;
		}
		sample_count++;  
		AKM_DATA_READ_RAW_COMPASS_SWAP(akm_addr,mag_temp);// get data
		AKM_CNTL_WRITE_SINGLE_MEAS_MODE(akm_addr,1); // Request next reading
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		spamtimer(1000){
			Serial.print((++PCount % 30)?"!":"\n");
			DelayCnt--;
		};
	}
	Serial.print(F("MS Reading Delay = "));
	Serial.println(DelayCnt +1);
	Serial.print(F("sample Count = "));
	Serial.println(sample_count);

	// Get hard iron correction
	mag_bias[0]  = (float)(mag_max[0] + mag_min[0])/2.0;  // get average x mag bias in counts
	mag_bias[1]  = (float)(mag_max[1] + mag_min[1])/2.0;  // get average y mag bias in counts
	mag_bias[2]  = (float)(mag_max[2] + mag_min[2])/2.0;  // get average z mag bias in counts

		
	mag_bias[0] =  mag_bias[0] * mRes * mag_sens_adj_F[0];  // save mag biases in G for main program
	mag_bias[1] =  mag_bias[1] * mRes * mag_sens_adj_F[1];
	mag_bias[2] =  mag_bias[2] * mRes * mag_sens_adj_F[2];
	
	// Get soft iron correction estimate
	mag_scale[0]  = (float)(mag_max[0] - mag_min[0])/2.0;  // get average x mag bias in counts
	mag_scale[1]  = (float)(mag_max[1] - mag_min[1])/2.0;  // get average y mag bias in counts
	mag_scale[2]  = (float)(mag_max[2] - mag_min[2])/2.0;  // get average z mag bias in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	mag_scale[0] = avg_rad/mag_scale[0];
	mag_scale[1] = avg_rad/mag_scale[1];
	mag_scale[2] = avg_rad/mag_scale[2];

	Serial.println("Mag Calibration done!");

	PrintMagOffsets();

	return *this;
}

Simple_MPU6050 & Simple_MPU6050::setMagOffsets(float xMagB,float yMagB,float zMagB, float xMagS,float yMagS,float zMagS){
	mag_bias[0] = (float)xMagB;
	mag_bias[1] = (float)yMagB;
	mag_bias[2] = (float)zMagB;
	mag_scale[0] = (float)xMagS;
	mag_scale[1] = (float)yMagS;
	mag_scale[2] = (float)zMagS;
	return *this; 
}


Simple_MPU6050 & Simple_MPU6050::PrintMagOffsets(){
	Serial.print(F("\n//                  X MagBias  Y MagBias  Z MagBias  X MagScale Y MagScale Z MagScale\n#define MAG_OFFSETS "));
	Serial.printfloatx("", mag_bias[0], 7, 1, ",  ");
	Serial.printfloatx("", mag_bias[1], 7, 1, ",  ");
	Serial.printfloatx("", mag_bias[2], 7, 1, ",  ");
	Serial.printfloatx("", mag_scale[0], 7, 3,",  ");
	Serial.printfloatx("", mag_scale[1], 7, 3,",  ");
	Serial.printfloatx("", mag_scale[2], 7, 3,"");
	Serial.println();
	return *this;
}


//Print the Mag Configuration registers
Simple_MPU6050 & Simple_MPU6050::viewMagRegisters(){
	uint8_t D;
	ReadByte(akm_addr,0,&D);
	Serial.print((ReadCount())? "R ":"X ");
	Serial.print(" Device ID 0x");
	DPRINTHEX(D);
	Serial.print(" 0B");
	DPRINTBIN(D);
	Serial.println();
	D = 0;

	while(!D){
		ReadByte(akm_addr,0x02,&D);
		Serial.print((ReadCount())? "R ":"X ");
		Serial.print(" Status 1 = 0x");
		DPRINTHEX(D);
		Serial.print(" 0B");
		DPRINTBIN(D);
		Serial.println();
		delay(1000);
	}
	Serial.println("****************");
	for(int i = 0X03;i<=0x08;i++){
		D = 0;
		if((i != 0x0B) && (i != 0x0D) && (i != 0x0E))  ReadByte(akm_addr,i,&D);
		Serial.print((ReadCount())? "R ":"X ");
		Serial.print("Register = 0x");
		DPRINTHEX(i);
		switch(i){
			case 0x00:
			Serial.print(" Device ID ");
			break;
			case 0x01:
			Serial.print(" Information ");
			break;
			case 0x02:
			Serial.print(" Status 1 ");
			break;
			case 0x03:
			Serial.print(" Measurement data ");
			break;
			case 0x04:
			Serial.print(" Measurement data ");
			break;
			case 0x05:
			Serial.print(" Measurement data ");
			break;
			case 0x06:
			Serial.print(" Measurement data ");
			break;
			case 0x07:
			Serial.print(" Measurement data ");
			break;
			case 0x08:
			Serial.print(" Measurement data ");
			break;
			case 0x09:
			Serial.print(" Status 2");
			break;
			case 0x0A:
			Serial.print(" Control ");
			break;
			case 0x0C:
			Serial.print(" Self-test");
			break;
			case 0x0F:
			Serial.print(" I2C disable");
			break;
			case 0x10:
			Serial.print(" X-axis sensitivity adjustment value");
			break;
			case 0x11:
			Serial.print(" Y-axis sensitivity adjustment value");
			break;
			case 0x12:
			Serial.print(" Z-axis sensitivity adjustment value");
			break;


		}
		Serial.print(" Value = 0x");
		DPRINTHEX(D);
		Serial.print(" 0B");
		DPRINTBIN(D);
		Serial.println();
	}
	Serial.println("\n");
	WriteByte(0x0C,0x0A ,  1 << 4 | 0x01 );// 16bit single measurement mode

	//	AKM_CNTL_WRITE_CONT_MEAS_MODE1(akm_addr,1);
	//Serial.println((WriteStatus())? "W-AKM_CNTL_WRITE_CONT_MEAS_MODE1":"X-AKM_CNTL_WRITE_CONT_MEAS_MODE1");
	return *this;
}


