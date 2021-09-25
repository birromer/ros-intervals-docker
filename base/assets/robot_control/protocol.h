#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__
#include <stdio.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <vector>
#include <unistd.h>
#include <cstring>


struct headerMsg
{
	uint8_t seq[3] = {0xab,0xcd,0xef};
	uint8_t typeOfMsg; // 0x01 pos 0x02 attitude 0x03 overGround 0x04 mission 0x05 speed 0x06 order 0x07 currentWp 0x08 distToLine 0x09 heading 0x10 mode 0x11 custom
	uint16_t lengthOfMsg; // 6+sizeof(data)
};

struct objectiveWpOrLine
{
	double lat;
	double lon;
	bool isLine;
};

struct missionMsg
{
	headerMsg header;
	uint8_t override;
	uint16_t numberOfObjectives;
	uint16_t totalObjectives;
	std::vector<objectiveWpOrLine> objectives;
	uint32_t checksum;
};

struct posMsg
{
	headerMsg header;
	double lat;
	double lon;
	double alt;
	double error_xy;
	double error_h;
	uint32_t checksum;
};

struct attitudeMsg
{
	headerMsg header;
	double heading;
	double roll;
	double pitch;
	uint32_t checksum;
};

struct overGroundMsg
{
	headerMsg header;
	double cog;
	double sog;
	double cog_acc;
	double sog_acc;
	uint32_t checksum;
};

struct customMsg
{
	headerMsg header;
	uint8_t isValid[4];
	double data[4];
	uint32_t checksum;
};

struct speedMsg
{
	headerMsg header;
	double speed;
	uint32_t checksum;
};

struct distToLineMsg
{
	headerMsg header;
	double distToLine;
	uint32_t checksum;
};

struct headingMsg
{
	headerMsg header;
	double heading;
	uint32_t checksum;
};

struct currentWpMsg
{
	headerMsg header;
	double lat;
	double lon;
	double dist;
	double distTot;
	double timeMission;
	uint32_t checksum;
};

struct orderMsg
{
	headerMsg header;
	uint8_t order; // 0x01 Get Speed 0x02 Get missions 0x03 Start robot 0x04 Stop robot 0x05 WP ok
	uint32_t checksum;
};

struct modeMsg
{
	headerMsg header;
	int8_t mode; //0 disarmed 1 armed +waypoints 3 armed + heading following 2 armed +line
	uint32_t checksum;
};

class Protocol
{
	public:
		Protocol();
		~Protocol();
		
		bool unpackPosMsg(std::vector<uint8_t> data, posMsg &pos);
		bool unpackAttitudeMsg(std::vector<uint8_t> data, attitudeMsg &attitude);
		bool unpackOverGroundMsg(std::vector<uint8_t> data, overGroundMsg &overGround);
		bool unpackMissionMsg(std::vector<uint8_t> data, missionMsg &mission);
		bool unpackSpeedMsg(std::vector<uint8_t> data, speedMsg &speed);
		bool unpackDistToLineMsg(std::vector<uint8_t> data, distToLineMsg &distToLine);
		bool unpackHeadingMsg(std::vector<uint8_t> data, headingMsg &heading);
		bool unpackCurrentWpMsg(std::vector<uint8_t> data, currentWpMsg &currentWp);
		bool unpackOrderMsg(std::vector<uint8_t> data, orderMsg &order);
		bool unpackModeMsg(std::vector<uint8_t> data, modeMsg &mode);
		bool unpackCustomMsg(std::vector<uint8_t> data, customMsg &custom);

		std::vector<uint8_t> packPosMsg(double lat, double lon, double alt, double error_xy, double error_h);
		std::vector<uint8_t> packAttitudeMsg(double heading, double roll, double pitch);
		std::vector<uint8_t> packOverGroundMsg(double cog, double sog, double cog_acc, double sog_acc);
		std::vector<uint8_t> packCustomMsg(uint8_t valid[4], double custom[4]);
		std::vector<uint8_t> packSpeedMsg(double speed);
		std::vector<uint8_t> packDistToLineMsg(double distToLine);
		std::vector<uint8_t> packHeadingMsg(double heading);
		std::vector<uint8_t> packCurrentWpMsg(double lat, double lon, double dist, double distTot, double timeMission);
		std::vector<uint8_t> packOrderMsg(uint8_t order);
		std::vector<uint8_t> packModeMsg(int8_t mode);
		std::vector<uint8_t> packMissionMsg(std::vector<objectiveWpOrLine> obj,uint16_t totalObjectives, uint8_t override);

	private:
		uint32_t calcChecksum(uint8_t *buffer,int length);


};
#endif