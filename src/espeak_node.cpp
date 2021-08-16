/*
 * espeak_ros_node.cpp
 *
 * Main node for the ROS espeak package
 * Subscribes to a single ROS topic of type std_msgs/String and sends the contents
 * of that topic through the Espeak voice synthesizer for audio output
 *
 */

#include <ros/ros.h>
#include <espeak/speak_lib.h>
#include <espeak_node.h>
#include <std_msgs/String.h>
#include <pthread.h>

EspeakNode::EspeakNode()
{
	nh_ = ros::NodeHandle("~");

	messageQueue_ = std::list<std::string>();

	espeakPositionType_ = POS_CHARACTER;
	espeakOutput_ = AUDIO_OUTPUT_PLAYBACK;
	espeakFlags_ = espeakCHARS_AUTO;
	espeakPlaybackBuffLength_ = 500;
	espeakPlaybackOptions_ = 0x00;

	loadParams();
	initializeEspeak();
}

EspeakNode::~EspeakNode()
{
	// nothing specific needed here
}

void EspeakNode::loadParams()
{
	if (nh_.hasParam("voice"))
		nh_.getParam("voice", espeakVoice_);
	else
		espeakVoice_ = "default";

	if (nh_.hasParam("maxQueue"))
		nh_.getParam("maxQueue", maxQueue_);
	else
		maxQueue_ = 1;
}

void EspeakNode::initializeEspeak()
{
	espeak_SetSynthCallback(&espeakSynthCallback);
	espeak_Initialize(espeakOutput_, espeakPlaybackBuffLength_, NULL, espeakPlaybackOptions_);
	espeak_SetVoiceByName(espeakVoice_.c_str());
}

void EspeakNode::run()
{
	pthread_mutex_init(&messageQueueLock_, NULL);
	pthread_create(&speakThread_, NULL, speakThreadFn, this);

	speakInputTopic_ = nh_.subscribe("voice_input", 1, &EspeakNode::speakCallback, this);

	ros::spin();
	pthread_join(speakThread_, NULL);
}

void EspeakNode::speakCallback(const std_msgs::String::ConstPtr &msg)
{
	// appends new data to the back of the queue, removes data from
	// the front of the queue if we exceed our maximum queue length
	pthread_mutex_lock(&messageQueueLock_);
	messageQueue_.push_back(msg->data);
	while (messageQueue_.size() > maxQueue_)
		messageQueue_.pop_front();
	pthread_mutex_unlock(&messageQueueLock_);
}

void *EspeakNode::speakThreadFn(void *arg)
{
	EspeakNode *node = (EspeakNode*)arg;
	ros::Rate sleepRate(10);
	std::string msg;
	bool doSpeak = false;

	while (ros::ok())
	{
		// check if there's anything in the queue. if there is,
		// remove it from the front of the queue and process it.
		// if there isn't, just wait 0.1s and check again.
		// yes, we could just use a condition variable to trigger
		// the worker thread, but this is easier, and for a 10Hz
		// task the CPU overhead is minimal anyway
		pthread_mutex_lock(&node->messageQueueLock_);
		if (node->messageQueue_.size() > 0)
		{
			msg = node->messageQueue_.front();
			node->messageQueue_.pop_front();
			doSpeak = true;
		}
		pthread_mutex_unlock(&node->messageQueueLock_);

		if (doSpeak)
		{
			node->speak(msg);
			doSpeak = false;
		}
		else
			sleepRate.sleep();
	}

	pthread_exit(NULL);
	return NULL;
}

void EspeakNode::speak(const std::string &msg)
{
	const char *cstr = msg.c_str();
	int size = strlen(cstr)+1;
	int position = 0;
	int end_position = 0;
	unsigned int* unique_id = NULL;
	void *user_data = NULL;

	espeak_Synth( cstr, size, position, espeakPositionType_, end_position, espeakFlags_, unique_id, user_data );
	espeak_Synchronize( );
}

int EspeakNode::espeakSynthCallback(short *wav, int numsamples, espeak_EVENT *events)
{
  return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "espeak_node");
	EspeakNode node = EspeakNode();
	node.run();
	ros::shutdown();
	return 0;
}
