/***************************************************************************
 *  include/speech_realizer/speech_realizer_node.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Dylan Foster (turtlecannon@gmail.com)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of sbl nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/


#ifndef SBL_SPEECHREALIZER_SPEECHREALIZER
#define SBL_SPEECHREALIZER_SPEECHREALIZER

/// getenv
#include <cstdlib>

// ROS
#include <ros/ros.h>

/// festival
#include <festival.h>

/// edinburgh sound tools
#include <EST.h>
#include <ling_class/EST_Item.h>
#include <ling_class/EST_Relation_list.h>

#include <std_msgs/String.h>
#include <speech_realizer/SayText.h>
#include <speech_realizer/GetWordTimings.h>

typedef std_msgs::String _StringMsg;

typedef speech_realizer::SayText _SayTextService;
typedef speech_realizer::GetWordTimings _GetWordTimingsService;

class SpeechRealizerNode
{
private:
  
  const std::string node_name_;
  double loop_rate_hz_;
  bool running_;

  ros::NodeHandle nh_rel_;
  ros::ServiceServer say_text_server_, get_word_timings_servers_;
  
  bool speak_locally_;

  EST_Val fest_default_val_float_;
  int fest_heap_size_;
  
  
public:
  SpeechRealizerNode(): node_name_("SpeechRealizer"), running_(false), nh_rel_("~"), 
			fest_default_val_float_(0.0f), fest_heap_size_(210000)
  {
  }

public:

  void spin()

  {
    ROS_INFO( "Spinning up %s...", node_name_.c_str() );
    
    nh_rel_.param<double>( "loop_rate", loop_rate_hz_, double(10) );

    ros::Rate loop_rate( loop_rate_hz_ );

    spinFirst();

    ROS_INFO( "%s is spinning at %.2f Hz.", node_name_.c_str(), loop_rate_hz_ ); 
    
    running_ = true;

    while( ros::ok() )
      {
	spinOnce();
	ros::spinOnce();
	loop_rate.sleep();
      }
    
    return;
  }

private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    nh_rel_.param<bool>( "speak_locally", speak_locally_, true);

    say_text_server_ = nh_rel_.advertiseService("say_text", &SpeechRealizerNode::textServiceCallback, this );

    get_word_timings_servers_ = nh_rel_.advertiseService("get_word_timings", &SpeechRealizerNode::getWordTimingsServiceCallback, this );

    festival_initialize(true, fest_heap_size_);

  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {

  }

  bool textServiceCallback( _SayTextService::Request & request, _SayTextService::Response & response)
  {
    ROS_INFO_STREAM("Saying text: \"" << request.text << "\"");

    EST_String est_text( request.text.c_str() );

    EST_String command1 = "(set! utt1 (utt.synth(Utterance Text \"" +
      est_text + "\")))";

    if (!festival_eval_command(command1))
      return false;

    if( speak_locally_ )
      return festival_say_text(est_text);

    return true;
  }

  bool getWordTimingsServiceCallback( _GetWordTimingsService::Request & request,
				      _GetWordTimingsService::Response & response )
  {
    typedef speech_realizer::TimedWord _TimedWordMsg;

    EST_String est_text( request.text.c_str() );

    EST_String command1 = "(set! utt1 (utt.synth(Utterance Text \"" +
      est_text + "\")))";

    char * home = getenv("HOME");
    
    if( home == NULL )
      {
	ROS_ERROR("Could not find home directory");
	return false;
      }

    std::stringstream utt_path;
    utt_path << home << "/.ros/speech_realizer.utt";
    EST_String utt_path_est( utt_path.str().c_str() );

    EST_String command2 = "(utt.save utt1 \"" + utt_path_est + "\")";

    if (!festival_eval_command(command1))
      return false;
    if( !festival_eval_command(command2))
      return false;
    
    EST_Utterance myUtt;
    EST_read_status status = myUtt.load(utt_path_est);
    if( status != read_ok )
      {
	ROS_ERROR("Failed to load utterance file.");
	return false;
      }

    EST_Item* s = NULL;
    for (s = myUtt.relation("Word")->head(); s != 0; s = next(s))
      {
	ROS_INFO_STREAM("Word: "   << s->S("name") <<
			"\tstart: "<< ff_word_start(s) <<
			"\tend: "  << ff_word_end(s) );

	_TimedWordMsg word;
	word.word = s->S("name");
	word.begin = ff_word_start(s);
	word.end = ff_word_end(s);

	response.words.push_back(word);
	
	//print out features of the word: name, start time, end time
	// cout << "Word: "   << s->S("name"); 
	// cout << "\tstart: "<< ff_word_start(s);
	// cout << "\tend: "  << ff_word_end(s) << endl;	

	// print out the "word_end" feature of the word  <-- does not work
	//cout << s->A("word_end") << endl;

	
	
      }

    return true;
  }

private:

  // returns segment start time		
  EST_Val ff_seg_start(EST_Item* s)
  {
    EST_Item* n = as(s, "Segment");
    if (prev(n) == 0) return fest_default_val_float_;
    return prev(n)->F("end", 0);
  } // ff_seg_start(EST_Item*)



  // returns segment end time
  EST_Val ff_seg_end(EST_Item* s)
  {
    return s->F("end", 0);
  } // ff_seg_end(EST_Item*)



  // returns syllable end time
  EST_Val ff_syl_end(EST_Item* s)
  {
    EST_Item* n = as(s, "SylStructure");
    if (daughtern(n) == 0) return fest_default_val_float_;
    return ff_seg_end(daughtern(n));
  } // ff_syl_end(EST_Item*)



  // returns word start time
  EST_Val ff_word_start(EST_Item* s)
  {
    EST_Item* n = as(s, "SylStructure");
    if (daughter1(daughter1(n)) == 0) return fest_default_val_float_;
    return ff_seg_start(daughter1(daughter1(n)));
  } // ff_word_start(EST_Item*)



  // returns word end time
  EST_Val ff_word_end(EST_Item* s)
  {
    EST_Item* n = as(s, "SylStructure");
    if (daughtern(n) == 0) return fest_default_val_float_;
    return ff_syl_end(daughtern(n));
  } // ff_word_end(EST_Item*)

};

#endif // SBL_SPEECHREALIZER_SPEECHREALIZER
