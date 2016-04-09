#define BOOST_ALL_DYN_LINK

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alnavigationproxy.h>
//... /* set up ROBOT_IP, ROBOT_PORT */ ...
static const char ROBOT_IP[] = "192.168.11.17";
static const int ROBOT_PORT  = 9559;

AL::ALMotionProxy proxy_motion(ROBOT_IP,ROBOT_PORT);
AL::ALRobotPostureProxy proxy_posture(ROBOT_IP,ROBOT_PORT);


int export_motion(AL::ALMotionProxy &proxy_motion){
	std::vector<std::string> names;
	AL::ALValue times, keys;
	names.reserve(12);
	times.arraySetSize(12);
	keys.arraySetSize(12);

	names.push_back("LElbowRoll");
	times[0].arraySetSize(3);
	keys[0].arraySetSize(3);

	times[0][0] = 0.96;
	keys[0][0] = -1.14586;
	times[0][1] = 1.44;
	keys[0][1] = -0.923426;
	times[0][2] = 1.92;
	keys[0][2] = -0.857464;

	names.push_back("LElbowYaw");
	times[1].arraySetSize(3);
	keys[1].arraySetSize(3);

	times[1][0] = 0.96;
	keys[1][0] = -0.434164;
	times[1][1] = 1.44;
	keys[1][1] = -0.615176;
	times[1][2] = 1.92;
	keys[1][2] = -0.615176;

	names.push_back("LHand");
	times[2].arraySetSize(3);
	keys[2].arraySetSize(3);

	times[2][0] = 0.96;
	keys[2][0] = 0.2896;
	times[2][1] = 1.44;
	keys[2][1] = 0.6676;
	times[2][2] = 1.92;
	keys[2][2] = 0.6676;

	names.push_back("LShoulderPitch");
	times[3].arraySetSize(3);
	keys[3].arraySetSize(3);

	times[3][0] = 0.96;
	keys[3][0] = 0.92496;
	times[3][1] = 1.44;
	keys[3][1] = 0.742414;
	times[3][2] = 1.92;
	keys[3][2] = 0.742414;

	names.push_back("LShoulderRoll");
	times[4].arraySetSize(3);
	keys[4].arraySetSize(3);

	times[4][0] = 0.96;
	keys[4][0] = 0.262272;
	times[4][1] = 1.44;
	keys[4][1] = 0.401866;
	times[4][2] = 1.92;
	keys[4][2] = 0.401866;

	names.push_back("LWristYaw");
	times[5].arraySetSize(3);
	keys[5].arraySetSize(3);

	times[5][0] = 0.96;
	keys[5][0] = 0.0843279;
	times[5][1] = 1.44;
	keys[5][1] = 0.099668;
	times[5][2] = 1.92;
	keys[5][2] = 0.099668;

	names.push_back("RElbowRoll");
	times[6].arraySetSize(3);
	keys[6].arraySetSize(3);

	times[6][0] = 0.96;
	keys[6][0] = 1.19043;
	times[6][1] = 1.44;
	keys[6][1] = 0.593412;
	times[6][2] = 1.92;
	keys[6][2] = 0.116937;

	names.push_back("RElbowYaw");
	times[7].arraySetSize(3);
	keys[7].arraySetSize(3);

	times[7][0] = 0.96;
	keys[7][0] = 0.516916;
	times[7][1] = 1.44;
	keys[7][1] = 0.49544;
	times[7][2] = 1.92;
	keys[7][2] = 0.492372;

	names.push_back("RHand");
	times[8].arraySetSize(3);
	keys[8].arraySetSize(3);

	times[8][0] = 0.96;
	keys[8][0] = 0.2968;
	times[8][1] = 1.44;
	keys[8][1] = 0.2968;
	times[8][2] = 1.92;
	keys[8][2] = 0.2968;

	names.push_back("RShoulderPitch");
	times[9].arraySetSize(3);
	keys[9].arraySetSize(3);

	times[9][0] = 0.96;
	keys[9][0] = -1.36485;
	times[9][1] = 1.44;
	keys[9][1] = -0.195477;
	times[9][2] = 1.92;
	keys[9][2] = -0.176368;

	names.push_back("RShoulderRoll");
	times[10].arraySetSize(3);
	keys[10].arraySetSize(3);

	times[10][0] = 0.96;
	keys[10][0] = -0.319114;
	times[10][1] = 1.44;
	keys[10][1] = -0.352862;
	times[10][2] = 1.92;
	keys[10][2] = -1.02974;

	names.push_back("RWristYaw");
	times[11].arraySetSize(3);
	keys[11].arraySetSize(3);

	times[11][0] = 0.96;
	keys[11][0] = 0.0628521;
	times[11][1] = 1.44;
	keys[11][1] = 0.0413761;
	times[11][2] = 1.92;
	keys[11][2] = 0.0413761;

	proxy_motion.angleInterpolation(names, keys, times, true);
	return 0;
}

std::string getPosture(){
	return proxy_posture.getPosture();
}

int moveTo(float x,float y,float threat)
{
	/*
	proxy_motion.stiffnessInterpolation("Body", 1.0, 1.0);
	Sleep(500);
	*/
	if(std::string(getPosture()) != std::string("Stand")){
		proxy_posture.goToPosture("Stand",1.0);
	}
	proxy_motion.moveTo(x, y, threat);
	proxy_motion.stopMove();
	return 0;
}

int rest()
{
	proxy_motion.rest();
	return 0;
}