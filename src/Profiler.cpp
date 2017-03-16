//============================================================================
// Name        : Profiler.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sys/wait.h>
#include "BlackI2C.h"
#include "BlackUART.h"
#include <ctime>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <queue>

union semun {
    int val;
    struct semid_ds *buf;
    ushort *array;
};

using namespace std;

class BatteryManager
{
	private:
		BlackLib::BlackUART Uart4;
	public:
		BatteryManager():Uart4(BlackLib::UART4, BlackLib::Baud19200, BlackLib::ParityNo, BlackLib::StopOne, BlackLib::Char8)
		{
			bool isOpened=this->Uart4.open( BlackLib::ReadWrite | BlackLib::NonBlock );
			if( !isOpened )
			{
				std::cout << "UART DEVICE CAN\'T OPEN.;" << std::endl;
				exit(1);
			}

		}

		unsigned int timeToEmpty()
		{

			string uart,aux,minToEnd;
			Uart4 >> uart;

			unsigned int a;
			if(uart=="UartReadError")
			{
				return 0;
			}

			std::stringstream UartStream(uart);
			while(std::getline(UartStream,aux,'\n')){
				a=aux.find("S");
				if(a!=string::npos)
				{
					minToEnd=aux.substr(a+5, 4);
					return (unsigned int)strtol(minToEnd.c_str(), NULL, 16);
				}
			}
			return 0;
		}
		unsigned int percentage()
		{
			string uart,aux,percentage;
			Uart4 >> uart;
			unsigned int a;
			if(uart=="UartReadError")
				return 0;

			std::stringstream UartStream(uart);
			while(std::getline(UartStream,aux,'\n')){
				a=aux.find("S");
				if(a!=string::npos)
				{
					percentage=aux.substr(a+19, 2);
					return (unsigned int)strtol(percentage.c_str(), NULL, 16);
				}
			}
			return 0;
		}


		virtual ~BatteryManager(){
			this->Uart4.close();
		}
};

class PWM
{
	private:
			int id1;
			int id2;
			bool state=false;//true if PWM enabled
			std::string ocpPath;
			std::string pwmPath;
			double currentPeriod=0;//nanoseconds
			double currentDutyCycle=0;//nanoseconds
	public:
			//PWM constructor-initializes PWM on Pid1_id2
			PWM(int id1, int id2){
				this->id1=id1;
				this->id2=id2;
				std::string pwmSelect;
				std::string pwmChip;
				if((this->id1==9 && this->id2==21) || (this->id1==9 && this->id2==22))
				{
					this->ocpPath="/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/";
				}
				else if((this->id1==9 && this->id2==14) || (this->id1==9 && this->id2==16)){
					this->ocpPath="/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/";
				}
				else if((this->id1==8 && this->id2==13) || (this->id1==8 && this->id2==19)){
					this->ocpPath="/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/";
				}
				pwmChip=executeCommand("sudo ls " + this->ocpPath + "| grep pwm*");
				pwmChip.erase(pwmChip.size()-1);
				if((this->id1==9 && this->id2==21) || (this->id1==9 && this->id2==22))
				{
					//cout<<"export ehrpwm0=" << this->ocpPath+pwmChip;
					//executeCommand("export ehrpwm0=" +this->ocpPath+pwmChip);
					if(this->id2==21)
					{
						executeCommand("config-pin P9.21 pwm");
						pwmSelect="1";
						this->pwmPath="/sys/class/pwm/"+pwmChip+"/pwm1/";
					}
					else if(this->id2==22)
					{
						executeCommand("config-pin P9.22 pwm");
						pwmSelect="0";
						this->pwmPath="/sys/class/pwm/"+pwmChip+"/pwm0/";
					}
				}
				else if((this->id1==9 && this->id2==14) || (this->id1==9 && this->id2==16))
				{
					//executeCommand("export ehrpwm1=" +this->ocpPath+pwmChip);
					if(this->id2==14)
					{
						executeCommand("config-pin P9.14 pwm");
						pwmSelect="0";
						this->pwmPath="/sys/class/pwm/"+pwmChip+"/pwm0/";
					}
					else if(this->id2==16)
					{
						executeCommand("config-pin P9.16 pwm");
						pwmSelect="1";
						this->pwmPath="/sys/class/pwm/"+pwmChip+"/pwm1/";
					}
				}
				else if((this->id1==8 && this->id2==13) || (this->id1==8 && this->id2==19))
				{
					//executeCommand("export ehrpwm2=" +this->ocpPath+pwmChip);
					if(this->id2==13)
					{
						executeCommand("config-pin P8.13 pwm");
						pwmSelect="1";
						this->pwmPath="/sys/class/pwm/"+pwmChip+"/pwm1/";
					}
					else if(this->id2==19)
					{
						executeCommand("config-pin P8.19 pwm");
						pwmSelect="0";
						this->pwmPath="/sys/class/pwm/"+pwmChip+"/pwm0/";
					}
				}
				std::string file= this->ocpPath + pwmChip + string("/export");
				//cout<<file;
				std::ofstream pwmExportFile;
				pwmExportFile.open(file.c_str(), std::ios::out);
				if(pwmExportFile.fail())
				{
					pwmExportFile.close();
					return;
				}
				else
				{
					pwmExportFile << pwmSelect;
					pwmExportFile.close();
					//cout<<this->pwmPath;
					return;
				}
				return;
			}
			//function to set period in nanoseconds
			bool setPeriod(double period){
				if( period > 1000000000)
					return false; //out of range
				else{
					std::ofstream periodFile;
					std::string file=pwmPath+"period";
					periodFile.open(file.c_str(),std::ios::out);
					if(periodFile.fail())
					{
						periodFile.close();
						return false;
					}
					else
					{
						periodFile << period;
						this->currentPeriod=period;
						periodFile.close();
						return true;
					}
				}
				return true;
			}
			//function to set duty cycle in nanoseconds
			bool setDutyCycle(double dutyCycle){
				if( dutyCycle > this->currentPeriod)
					return false; //out of range
				else if(dutyCycle<0){
					return false;
				}
				else{
					std::ofstream dutyFile;
					std::string file=pwmPath+"duty_cycle";
					dutyFile.open(file.c_str(),std::ios::out);
					if(dutyFile.fail())
					{
						dutyFile.close();
						return false;
					}
					else
					{
						dutyFile << dutyCycle;
						this->currentDutyCycle=dutyCycle;
						dutyFile.close();
						return true;
					}
				}
				return true;
			}
			//function to enable pwm
			bool enablePWM(){
				std::ofstream enableFile;
				std::string file=pwmPath+"enable";
				enableFile.open(file.c_str(),std::ios::out);
				if(enableFile.fail())
				{
					enableFile.close();
					return false;
				}
				else
				{
					enableFile << "1";
					this->state=true;
					enableFile.close();
					return true;
				}
				return true;
			}
			//function to disable pwm
			bool disablePWM(){
				std::ofstream enableFile;
				std::string file=pwmPath+"enable";
				enableFile.open(file.c_str(),std::ios::out);
				if(enableFile.fail())
				{
					enableFile.close();
					return false;
				}
				else
				{
					enableFile << "0";
					this->state=false;
					enableFile.close();
					return true;
				}
				return true;
			}
			//function to get dutycycle
			double getDutyCycle(){
				return this->currentDutyCycle;
			}
			//function to get Perido
			double getPeriod(){
				return this->currentPeriod;
			}
			std::string searchDirectory(std::string searchIn, std::string searchThis)
			{
				std::string str;
				DIR *path;
				dirent *entry;
				path = opendir(searchIn.c_str());
				if( path != NULL  )
				{
					while( (entry = readdir(path)) != NULL)
					{
						if( entry->d_name[0] == '.')
						{
							continue;
						}

						str = entry->d_name;
						if(strstr(entry->d_name,searchThis.c_str()) != NULL )
						{
							closedir(path);
							return str;
						}
					}
				}
				closedir(path);
				return "ERROR";
			}
			std::string executeCommand(std::string command)
			{
				FILE* pipe = popen(command.c_str(), "r");
				if ( pipe==NULL )
				{
					return "ERROR";
				}

				char buffer[128];
				std::string result = "";

				while( !feof(pipe) )
				{
					if( fgets(buffer, 128, pipe) != NULL )
					{
						result += buffer;
					}
				}
				pclose(pipe);
				return result;
			}
			virtual ~PWM(){
			}
};

class TemperaturaI2C
{
	private:
		uint16_t K4;
		uint16_t K3;
		uint16_t K2;
		uint16_t K1;
		uint16_t K0;
		BlackLib::BlackI2C TempSensor;
	public:
		//Temperatura I2C constructor-initializes calibration parameters
		TemperaturaI2C():TempSensor(BlackLib::I2C_2,0x77)
		{

			bool isOpened = this->TempSensor.open( BlackLib::ReadWrite | BlackLib::NonBlock );
			if( !isOpened )
			{
				std::cout << "I2C DEVICE CAN\'T OPEN.;" << std::endl;
				exit(1);
			}
			TempSensor.readByte(0x1E);
			usleep(10000);

			this->K4=TempSensor.readWord(0xA2);
			this->K3=TempSensor.readWord(0xA4);
			this->K2=TempSensor.readWord(0xA6);
			this->K1=TempSensor.readWord(0xA8);
			this->K0=TempSensor.readWord(0xAA);
			this->K4=(this->K4>>8) | (this->K4<<8);
			this->K3=(this->K3>>8) | (this->K3<<8);
			this->K2=(this->K2>>8) | (this->K2<<8);
			this->K1=(this->K1>>8) | (this->K1<<8);
			this->K0=(this->K0>>8) | (this->K0<<8);
			return;
		}
		float readTemperature()
		{
			this->TempSensor.readByte(0x48);
			usleep(10000);
			uint8_t adcBytes[3]={0,0,0};
			uint32_t adcRead;
			this->TempSensor.readBlock(0x00, adcBytes, 3);
			adcRead =adcBytes[0];
			adcRead = (adcRead << 8) | adcBytes[1];
			adcRead = (adcRead << 8) | adcBytes[2];
			adcRead=adcRead/256;
			//float Temperatura=(-2)*(K4)*pow(10,-21)*pow(adcRead,4)+4*(K3)*pow(10,-16)*pow(adcRead,3)+(-2)*(K2)*pow(10,-11)*pow(adcRead,2)+1*(K1)*pow(10,-6)*adcRead+(-1.5)*(K0)*pow(10,-2);
			float Temperatura = (-2) * float(K4) / 1000000000000000000000.0f * pow(adcRead,4) + 4 * float(K3) / 10000000000000000.0f * pow(adcRead,3) + (-2) * float(K2) / 100000000000.0f * pow(adcRead,2) +   1 * float(K1) / 1000000.0f * adcRead + (-1.5) * float(K0) / 100 ;
			return Temperatura;
		}
		virtual ~TemperaturaI2C(){
			this->TempSensor.close();
		}
};

class PressureI2C
{
	private:
		uint8_t crcRead;
		uint16_t n_prom[8];
		uint16_t fluidDensity=1029;//defined as sea water density
		BlackLib::BlackI2C PressureSensor;
	public:
		//Pressure I2C constructor-initializes calibration parameters
		PressureI2C():PressureSensor(BlackLib::I2C_2,0x76)
		{
			bool isOpened = this->PressureSensor.open( BlackLib::ReadWrite | BlackLib::NonBlock );
			if( !isOpened )
			{
				std::cout << "I2C DEVICE CAN\'T OPEN.;" << std::endl;
				exit(1);
			}
			PressureSensor.readByte(0x1E);
			usleep(10000);

			this->n_prom[0]=(PressureSensor.readWord(0xA0)>>8)|(PressureSensor.readWord(0xA0)<<8);
			//this->n_prom[0]=PressureSensor.readWord(0xA0);
			this->n_prom[1]=(PressureSensor.readWord(0xA2)>>8)|(PressureSensor.readWord(0xA2)<<8);
			//this->n_prom[1]=PressureSensor.readWord(0xA2);
			this->n_prom[2]=(PressureSensor.readWord(0xA4)>>8)|(PressureSensor.readWord(0xA4)<<8);
			//this->n_prom[2]=(PressureSensor.readWord(0xA4));
			this->n_prom[3]=(PressureSensor.readWord(0xA6)>>8)|(PressureSensor.readWord(0xA6)<<8);
			//this->n_prom[3]=(PressureSensor.readWord(0xA6));
			this->n_prom[4]=(PressureSensor.readWord(0xA8)>>8)|(PressureSensor.readWord(0xA8)<<8);
			//this->n_prom[4]=(PressureSensor.readWord(0xA8));
			this->n_prom[5]=(PressureSensor.readWord(0xAA)>>8)|(PressureSensor.readWord(0xAA)<<8);
			//this->n_prom[5]=(PressureSensor.readWord(0xAA));
			this->n_prom[6]=(PressureSensor.readWord(0xAC)>>8)|(PressureSensor.readWord(0xAC)<<8);
			//this->n_prom[6]=(PressureSensor.readWord(0xAC));
			this->n_prom[7]=(PressureSensor.readWord(0xAE)>>8)|(PressureSensor.readWord(0xAE)<<8);
			//this->n_prom[7]=(PressureSensor.readWord(0xAE));
			this->crcRead=this->n_prom[0]>>12;
			if(this->crcRead==this->crc4(this->n_prom))
				return;
			else
			{
				perror("CRC Error");
				cout<<endl;
				exit(1);
			}
			//cout<<"C1 "<<C1<<endl<<"C2 "<<C2<<endl<<"C3 "<<C3<<endl<<"C4 "<<C4<<endl<<"C5 "<<C5<<endl<<"C6 "<<C6<<endl<<"C7 "<<C7<<endl<<"C8 "<<C8<<endl;
			return;
		}
		unsigned char crc4(uint16_t prom[]) // n_prom defined as 8x unsigned int (n_prom[8])
		{
			int cnt; // simple counter
			unsigned int n_rem=0; // crc remainder
			unsigned char n_bit;

			prom[0]=((prom[0]) & 0x0FFF); // CRC byte is replaced by 0
			prom[7]=0; // Subsidiary value, set to 0
			for (cnt =0; cnt < 16; cnt++) // operation is performed on bytes
			{ // choose LSB or MSB
				if (cnt%2==1) n_rem ^= (unsigned short) ((prom[cnt>>1]) &0x00FF);
				else n_rem ^= (unsigned short) (prom[cnt>>1]>>8);
				for (n_bit = 8; n_bit > 0; n_bit--)
				{
					if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
					else n_rem = (n_rem << 1);
				}
			}
			n_rem= ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
			return (n_rem ^ 0x00);
		}
		float readPressure()
		{
			uint8_t adcBytesD1[3]={0,0,0};
			uint8_t adcBytesD2[3]={0,0,0};
			uint32_t D1;//Digital pressure value
			uint32_t D2;//Digital temperature value
			this->PressureSensor.readByte(0x4A);//start convertion D1 OSR 8192
			usleep(20000);//pode ser menor usando um OSR menor (ver datasheet)
			this->PressureSensor.readBlock(0x00, adcBytesD1, 3);
			D1=adcBytesD1[0];
			D1=(D1<<8)|adcBytesD1[1];
			D1=(D1<<8)|adcBytesD1[2];

			this->PressureSensor.readByte(0x5A);//start convertion D2 OSR 8192
			usleep(20000);//pode ser menor usando um OSR menor (ver datasheet)
			this->PressureSensor.readBlock(0x00, adcBytesD2, 3);
			D2=adcBytesD2[0];
			D2=(D2<<8)|adcBytesD2[1];
			D2=(D2<<8)|adcBytesD2[2];

			int32_t dT=D2-this->n_prom[5]*pow(2,8);
			int32_t temp=2000+dT*this->n_prom[6]/pow(2,23);
			int64_t off=this->n_prom[2]*pow(2,16)+(this->n_prom[4]*dT)/pow(2,7);
			int64_t sens=this->n_prom[1]*pow(2,15)+(this->n_prom[3]*dT)/pow(2,8);
			//int32_t pressure=(D1*sens/pow(2,21)-off)/pow(2,13);

			int64_t offi=0,sensi=0;
			//int64_t Ti=0;

			if(temp/100<20)
			{
				//Ti=3*pow(dT,2)/pow(2,33);
				offi=3*pow((temp-2000),2)/pow(2,1);
				sensi=5*pow((temp-2000),2)/pow(2,3);
				if(temp/100<-15)
				{
					offi=offi+7*pow((temp+1500),2);
					sensi=sensi+4*pow((temp+1500),2);
				}
 			}
			else
			{
				//Ti=2*pow(dT,2)/pow(2,37);
				offi=1*pow((temp-2000),2)/pow(2,4);
				sensi=0;
			}
			int64_t off2=off-offi;
			int64_t sens2=sens-sensi;
			//float temp2=((float)temp-(float)Ti)/100;
			//cout<<"Temperatura "<<temp2<<endl;
			float p2=(((D1*sens2)/pow(2,21)-off2)/pow(2,13))/10;
			return p2;
		}
		float calcDepth(float pressure)
		{
			return ((pressure-1013.25)*100)/(this->fluidDensity*9.80665);
		}
		uint16_t setFluidDensity(uint16_t fluidDensity)
		{
			this->fluidDensity=fluidDensity;
			return this->fluidDensity;
		}
		virtual ~PressureI2C(){
			this->PressureSensor.close();
		}
};

//class to log data
class DataTime
{
	public:
		DataTime() : data(), timestamp() {}
		DataTime(float data, time_t timestamp)
		: data(data), timestamp(timestamp) {}
		float data;
		time_t timestamp;
};

int main() {

	system("config-pin overlay cape-universal");
	sleep(5);
	system("config-pin P9.13 uart");
	system("config-pin P9.11 uart");
	cout<<endl;

	key_t keyData, keyTime;
	int shmidData, shmidTime;
	float *data;
	time_t *timestamp;
	//int mode;


///////////////Shared Memory Initialization
	/* make the key for data: */
	if ((keyData = ftok("/root/Joao/helloBBB", 'R')) == -1) {
		perror("ftok. Error generating key");
		exit(1);
	}
	/* make the key for data: */
	if ((keyTime = ftok("/root/Joao/helloBBB", 'W')) == -1) {
		perror("ftok. Error generating key");
		exit(1);
	}
	/* connect to (and possibly create) the segment: */
	//Create shared memory for data
	if ((shmidData = shmget(keyData, 4*sizeof(float), 0644 | IPC_CREAT)) == -1) {
		perror("shmget. Error creating memory segment");
		exit(1);
	}
	//Create shared memory for timestamps
	if ((shmidTime = shmget(keyTime, 4*sizeof(time_t), 0644 | IPC_CREAT)) == -1) {
		perror("shmget. Error creating memory segment");
		exit(1);
	}
//////////////

//////////////Semaphores
	union semun arg;
	//struct semid_ds buf;
	struct sembuf sb;
	int semid;
	semid = semget(keyData, 1, IPC_CREAT | IPC_EXCL | 0666);
	if (semid >= 0) {
		sb.sem_op = 1; sb.sem_flg = 0;sb.sem_num=0;
		arg.val = 1;
		/* do a semop() to "free" the semaphores. */
		/* this sets the sem_otime field, as needed below. */
		if (semop(semid, &sb, 1) == -1) {
			semctl(semid, 0, IPC_RMID); /* clean up */
			perror("Error cleaning the semaphore");
			exit(1); /* error, check errno */
		}
	}
	else
	{
		perror("Error creating semaphore");
		exit(1);
	}

/////////////
	pid_t PID1=fork();
	if(PID1<0)
	{
		perror("error creating process");
		exit(1);
	}
//////////////////////////////////////////PROCESS 1
	if(PID1==0)
	{
		struct sembuf sb;
		cout<<"Child 1 Process number "<<getpid()<<endl<<endl;
		//////////Shared Memory
		/* attach to the segment to get a pointer to it: */
		data = (float*) shmat(shmidData, (void *)0, 0);
		if (data == (float *)(-1)) {
			perror("shmat. Error attaching to memory segment");
			exit(1);
		}
		/* attach to the segment to get a pointer to it: */
		timestamp = (time_t*) shmat(shmidTime, (void *)0, 0);
		if (data == (float *)(-1)) {
			perror("shmat. Error attaching to memory segment");
			exit(1);
		}

		timestamp[0]=0;
		timestamp[1]=0;
		timestamp[2]=0;
		timestamp[3]=0;
		//////////
		TemperaturaI2C Temp;
		float Temperatura=Temp.readTemperature();
		PressureI2C Pre;
		float pressure=Pre.readPressure();
		float depth=Pre.calcDepth(pressure);
		BatteryManager Battery;

		float timeToEnd, percentage;

		do{
			timeToEnd=(float)Battery.timeToEmpty();
			percentage=(float)Battery.percentage();
		}
		while(timeToEnd==0);
		do{
			percentage=(float)Battery.percentage();
		}
		while(percentage==0);
		//cout<<"Pressao "<<pressure<<endl<<endl;
		//cout<<"Profundidade "<<depth<<endl<<endl;
		//cout<<"Temperatura "<<Temperatura<<endl<<endl;


		//shared memory lock
		sb.sem_num = 0;
		sb.sem_op = -1;  /* set to allocate resource */
		sb.sem_flg = SEM_UNDO;
		if (semop(semid, &sb, 1) == -1) {
			perror("semop");
			exit(1);
		}
		//
		data[0]=Temperatura;
		data[1]=pressure;
		data[2]=depth;
		data[3]=timeToEnd;
		timestamp[0]=std::time(0);
		timestamp[1]=std::time(0);
		timestamp[2]=std::time(0);
		timestamp[3]=std::time(0);


		sb.sem_op = 1; /* free resource */
		if (semop(semid, &sb, 1) == -1) {
			perror("semop");
			exit(1);
		}
		//
		//free shared memory
		if (shmdt(data) == -1) {
			perror("shmdt");
			exit(1);
		}
		exit(42);
	}
/////////////


	pid_t PID2=fork();
	if(PID2<0)
	{
		perror("error creating process");
		exit(1);
	}
/////////////////////////////////////////PROCESS 2
	if(PID2==0)
	{
		std::queue<DataTime> temperatura, pressao, profundidade,bateria;

		cout<<"Child 2 Process number "<<getpid()<<endl<<endl;
		ofstream outputFile("Teste.txt");
		/* attach to the segment to get a pointer to it: */
		data = (float*) shmat(shmidData, (void *)0, 0);
		if (data == (float *)(-1)) {
			perror("shmat. Error attaching to memory segment");
			exit(1);
		}
		timestamp = (time_t*) shmat(shmidTime, (void *)0, 0);
		if (data == (float *)(-1)) {
			perror("shmat. Error attaching to memory segment");
			exit(1);
		}

		while(timestamp[0]==0){}


		outputFile << "writing to file"<<endl<<endl;
		//shared memory lock
		sb.sem_num = 0;
		sb.sem_op = -1;  /* set to allocate resource */
		sb.sem_flg = SEM_UNDO;
		if (semop(semid, &sb, 1) == -1) {
			perror("semop");
			exit(1);
		}
		//
		temperatura.push(DataTime(data[0],timestamp[0]));
		pressao.push(DataTime(data[1],timestamp[1]));
		profundidade.push(DataTime(data[2],timestamp[2]));
		bateria.push(DataTime(data[3],timestamp[3]));
		timestamp[0]=0;
		timestamp[1]=0;
		timestamp[2]=0;
		timestamp[3]=0;

		sb.sem_op = 1; /* free resource */
		if (semop(semid, &sb, 1) == -1) {
			perror("semop");
			exit(1);
		}
		//
		outputFile <<"Temperatura by TSYS "<<temperatura.front().data<<" at time "<<ctime(&temperatura.front().timestamp)<<endl;
		outputFile <<"Pressao "<<pressao.front().data<<" at time "<<ctime(&pressao.front().timestamp)<<endl;
		outputFile <<"Profundidade "<<profundidade.front().data<<" at time "<<ctime(&profundidade.front().timestamp)<<endl;
		outputFile <<"Time to end "<<bateria.front().data<<" at time "<<ctime(&bateria.front().timestamp)<<endl;
		temperatura.pop();
		pressao.pop();
		profundidade.pop();
		bateria.pop();
		outputFile.close();

		cout << "writing done"<<endl<<endl;
		//free shared memory
		if (shmdt(data) == -1) {
			perror("shmdt");
			exit(1);
		}
		//cout<<"Done Writing"<<endl;
		exit(42);
	}
/////////////

/////////////PROCESS 0
	cout<<"Father Process number "<< getpid()<<endl<<endl;

	PWM P9_22(9, 22);
	PWM P9_21(9, 21);
	PWM P9_14(9, 14);
	PWM P9_16(9, 16);
	PWM P8_13(8, 13);
	PWM P8_19(8, 19);
	P8_19.setPeriod(50000);//nanoseconds
	P8_19.setDutyCycle(40000);//nanoseconds
	P8_19.disablePWM();
	cout<<"PWM period is "<<P8_19.getPeriod()<<endl;
	cout<<"PWM duty is "<<P8_19.getDutyCycle()<<endl;

	waitpid(PID2,NULL,0);
	waitpid(PID1,NULL,0);

	//remove semaphore
	if (semctl(semid, 0, IPC_RMID, arg) == -1) {
		perror("semctl");
		exit(1);
	}

	shmctl(shmidData, IPC_RMID, NULL);
	shmctl(shmidTime, IPC_RMID, NULL);

	return 0;
}
