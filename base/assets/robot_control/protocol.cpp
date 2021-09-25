#include "protocol.h"

uint32_t Protocol::calcChecksum(uint8_t *buffer,int length)
{
	uint32_t checksum = 0;
	for(int i = 0; i < length; i++)
	{
		checksum += buffer[i];
	}
	return checksum;
}

bool Protocol::unpackPosMsg(std::vector<uint8_t> data, posMsg &pos)
{
	posMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.lat, &buffer[6],8);
	memcpy(&msg.lon, &buffer[14],8);
	memcpy(&msg.alt, &buffer[22],8);
	memcpy(&msg.error_xy, &buffer[30],8);
	memcpy(&msg.error_h, &buffer[38],8);
	memcpy(&msg.checksum, &buffer[46],4);

	uint32_t checksum = calcChecksum(buffer,46);

	if(checksum == msg.checksum)
	{
		pos = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackAttitudeMsg(std::vector<uint8_t> data, attitudeMsg &attitude)
{
	attitudeMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.heading, &buffer[6],8);
	memcpy(&msg.roll, &buffer[14],8);
	memcpy(&msg.pitch, &buffer[22],8);
	memcpy(&msg.checksum, &buffer[30],4);

	uint32_t checksum = calcChecksum(buffer,30);

	if(checksum == msg.checksum)
	{
		attitude = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackOverGroundMsg(std::vector<uint8_t> data, overGroundMsg &overGround)
{
	overGroundMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.cog, &buffer[6],8);
	memcpy(&msg.sog, &buffer[14],8);
	memcpy(&msg.cog_acc, &buffer[22],8);
	memcpy(&msg.sog_acc, &buffer[30],8);
	memcpy(&msg.checksum, &buffer[38],4);

	uint32_t checksum = calcChecksum(buffer,38);

	if(checksum == msg.checksum)
	{
		overGround = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackCustomMsg(std::vector<uint8_t> data, customMsg &custom)
{
	customMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.isValid, &buffer[6],4);
	memcpy(&msg.data[0], &buffer[10],8);
	memcpy(&msg.data[1], &buffer[18],8);
	memcpy(&msg.data[2], &buffer[26],8);
	memcpy(&msg.data[3], &buffer[34],8);
	memcpy(&msg.checksum, &buffer[42],4);

	uint32_t checksum = calcChecksum(buffer,42);

	if(checksum == msg.checksum)
	{
		custom = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackSpeedMsg(std::vector<uint8_t> data, speedMsg &speed)
{
	speedMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.speed, &buffer[6],8);
	memcpy(&msg.checksum, &buffer[14],4);

	uint32_t checksum = calcChecksum(buffer,14);

	if(checksum == msg.checksum)
	{
		speed = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackDistToLineMsg(std::vector<uint8_t> data, distToLineMsg &distToLine)
{
	distToLineMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.distToLine, &buffer[6],8);
	memcpy(&msg.checksum, &buffer[14],4);

	uint32_t checksum = calcChecksum(buffer,14);

	if(checksum == msg.checksum)
	{
		distToLine = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackHeadingMsg(std::vector<uint8_t> data, headingMsg &heading)
{
	headingMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.heading, &buffer[6],8);
	memcpy(&msg.checksum, &buffer[14],4);

	uint32_t checksum = calcChecksum(buffer,14);

	if(checksum == msg.checksum)
	{
		heading = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackCurrentWpMsg(std::vector<uint8_t> data, currentWpMsg &currentWp)
{
	currentWpMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.lat, &buffer[6],8);
	memcpy(&msg.lon, &buffer[14],8);
	memcpy(&msg.dist, &buffer[22],8);
	memcpy(&msg.distTot, &buffer[30],8);
	memcpy(&msg.timeMission, &buffer[38],8);
	memcpy(&msg.checksum, &buffer[46],4);

	uint32_t checksum = calcChecksum(buffer,46);

	if(checksum == msg.checksum)
	{
		currentWp = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackOrderMsg(std::vector<uint8_t> data, orderMsg &order)
{
	orderMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.order, &buffer[6],1);
	memcpy(&msg.checksum, &buffer[7],4);
	uint32_t checksum = calcChecksum(buffer,7);

	if(checksum == msg.checksum)
	{
		order = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackMissionMsg(std::vector<uint8_t> data, missionMsg &mission)
{
	missionMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.override, &buffer[6],1);
	memcpy(&msg.numberOfObjectives, &buffer[7],2);
	memcpy(&msg.totalObjectives, &buffer[9],2);

	for(int i = 0; i < msg.numberOfObjectives; i++)
	{
		objectiveWpOrLine o;
		memcpy(&o.lat,&buffer[11+17*i],8);
		memcpy(&o.lon,&buffer[19+17*i],8);
		memcpy(&o.isLine,&buffer[27+17*i],1);
		msg.objectives.push_back(o);
	}
	memcpy(&msg.checksum, &buffer[11+msg.numberOfObjectives*17],4);
	uint32_t checksum = calcChecksum(buffer,11+msg.numberOfObjectives*17);
	if(checksum == msg.checksum)
	{
		mission = msg;
		return true;
	}
	else
	{
		return false;
	}
}

bool Protocol::unpackModeMsg(std::vector<uint8_t> data, modeMsg &mode)
{
	modeMsg msg;
	uint8_t *buffer = &data[0];
	memcpy(&msg.header.seq, &buffer[0],3);
	memcpy(&msg.header.typeOfMsg, &buffer[3],1);
	memcpy(&msg.header.lengthOfMsg, &buffer[4],2);
	memcpy(&msg.mode, &buffer[6],1);
	memcpy(&msg.checksum, &buffer[7],4);
	uint32_t checksum = calcChecksum(buffer,7);

	if(checksum == msg.checksum)
	{
		mode = msg;
		return true;
	}
	else
	{
		return false;
	}
}

std::vector<uint8_t> Protocol::packPosMsg(double lat, double lon, double alt, double error_xy, double error_h)
{
	uint16_t length = 50;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x01;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&lat), 8);
	memcpy(&buffer[14], (uint8_t *)(&lon), 8);
	memcpy(&buffer[22], (uint8_t *)(&alt), 8);
	memcpy(&buffer[30], (uint8_t *)(&error_xy), 8);
	memcpy(&buffer[38], (uint8_t *)(&error_h), 8);

	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[46], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packAttitudeMsg(double heading, double roll, double pitch)
{
	uint16_t length = 34;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x02;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&heading), 8);
	memcpy(&buffer[14], (uint8_t *)(&roll), 8);
	memcpy(&buffer[22], (uint8_t *)(&pitch), 8);

	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[30], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packOverGroundMsg(double cog, double sog, double cog_acc, double sog_acc)
{
	uint16_t length = 42;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x03;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&cog), 8);
	memcpy(&buffer[14], (uint8_t *)(&sog), 8);
	memcpy(&buffer[22], (uint8_t *)(&cog_acc), 8);
	memcpy(&buffer[30], (uint8_t *)(&sog_acc), 8);

	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[38], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packCustomMsg(uint8_t valid[4], double custom[4])
{
	uint16_t length = 46;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x11;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&valid[0]), 4);
	memcpy(&buffer[10], (uint8_t *)(&custom[0]), 8);
	memcpy(&buffer[18], (uint8_t *)(&custom[1]), 8);
	memcpy(&buffer[26], (uint8_t *)(&custom[2]), 8);
	memcpy(&buffer[34], (uint8_t *)(&custom[3]), 8);

	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[42], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packSpeedMsg(double speed)
{
	uint16_t length = 18;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x05;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&speed), 8);

	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[14], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packDistToLineMsg(double distToLine)
{
	uint16_t length = 18;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x08;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&distToLine), 8);

	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[14], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packHeadingMsg(double heading)
{
	uint16_t length = 18;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x09;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&heading), 8);

	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[14], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packCurrentWpMsg(double lat, double lon, double dist, double distTot, double timeMission)
{
	uint16_t length = 50;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x07;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&lat), 8);
	memcpy(&buffer[14], (uint8_t *)(&lon), 8);
	memcpy(&buffer[22], (uint8_t *)(&dist), 8);
	memcpy(&buffer[30], (uint8_t *)(&distTot), 8);
	memcpy(&buffer[38], (uint8_t *)(&timeMission), 8);

	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[46], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packOrderMsg(uint8_t order)
{
	uint16_t length = 11;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x06;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&order), 1);
	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[7], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packModeMsg(int8_t mode)
{
	uint16_t length = 11;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x10;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&mode), 1);
	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[7], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

std::vector<uint8_t> Protocol::packMissionMsg(std::vector<objectiveWpOrLine> obj,uint16_t totalObjectives, uint8_t override)
{
	int nb_obj = obj.size();
	uint16_t length = 11 + nb_obj*17 + 4;
	uint8_t buffer[length];
	buffer[0] = 0xab;
	buffer[1] = 0xcd;
	buffer[2] = 0xef;
	buffer[3] = 0x04;
	memcpy(&buffer[4], (uint8_t *)(&length), 2);
	memcpy(&buffer[6], (uint8_t *)(&override), 1);
	memcpy(&buffer[7], (uint8_t *)(&nb_obj), 2);
	memcpy(&buffer[9], (uint8_t *)(&totalObjectives), 2);
	for(int i = 0; i < nb_obj; i++)
	{
		memcpy(&buffer[11+17*i], (uint8_t *)(&obj[i].lat), 8);
		memcpy(&buffer[19+17*i], (uint8_t *)(&obj[i].lon), 8);
		memcpy(&buffer[27+17*i], (uint8_t *)(&obj[i].isLine), 1);
	}

	uint32_t checksum = calcChecksum(buffer,length-4);
	memcpy(&buffer[length-4], (uint8_t *)(&checksum), 4);
	std::vector<uint8_t> output(buffer,buffer+length);
	return output;
}

Protocol::Protocol()
{
		/*
	currentWpMsg tmp;
	bool a9 = unpackCurrentWpMsg(packCurrentWpMsg(1.,2.,3.),tmp);
	std::cout << tmp.lat << " " << tmp.lon << " " << tmp.dist << std::endl;

	speedMsg tmp;
	bool a6 = unpackSpeedMsg(packSpeedMsg(86.),tmp);
	if(!a6)
	{
		std::cout << "failure" << std::endl;
	}
	std::cout << tmp.speed << std::endl;
	*/
/*
	orderMsg tmp;
	bool a7 = unpackOrderMsg(packOrderMsg(0x05),tmp);
	if(!a7)
	{
		std::cout << "failure" << std::endl;
	}
	std::cout << std::hex << (int)tmp.order << std::endl;
*/
	/*posMsg tmp1;
	attitudeMsg tmp2;
	overGroundMsg tmp3;
	bool a1 = unpackPosMsg(packPosMsg(1.586,55.555,3.255,1.5,6.),tmp1);
	bool a2 = unpackAttitudeMsg(packAttitudeMsg(1.555,2.22,1.75),tmp2);
	bool a3 = unpackOverGroundMsg(packOverGroundMsg(1.2,6.6),tmp3);


	objectiveWpOrLine o1;
	o1.lat_pt0 = 1.1;
	o1.lon_pt0 = 8.1;
	o1.lat_pt1 = 5.1;
	o1.lon_pt1 = 6.1;
	o1.isLine = false;

	objectiveWpOrLine o2;
	o2.lat_pt0 = 12.1;
	o2.lon_pt0 = 84.1;
	o2.lat_pt1 = 45.1;
	o2.lon_pt1 = 66.1;
	o2.isLine = true;

	objectiveWpOrLine o3;
	o3.lat_pt0 = 14.1;
	o3.lon_pt0 = 48.1;
	o3.lat_pt1 = 95.1;
	o3.lon_pt1 = 96.1;
	o3.isLine = false;

	std::vector<objectiveWpOrLine> objs = {o1,o2,o3};
	missionMsg tmp4;
	bool a4 = unpackMissionMsg(packMissionMsg(objs),tmp4);
	for(int i = 0; i < tmp4.numberOfObjectives; i++)
	{
		std::cout << tmp4.header.lengthOfMsg << std::endl;
		std::cout << tmp4.objectives.size() << std::endl;
		std::cout << tmp4.objectives[i].lat_pt0 << std::endl;
		std::cout << tmp4.objectives[i].lon_pt0 << std::endl;
		std::cout << tmp4.objectives[i].lat_pt1 << std::endl;
		std::cout << tmp4.objectives[i].lon_pt1 << std::endl;
	}
	*/
}

Protocol::~Protocol()
{

}