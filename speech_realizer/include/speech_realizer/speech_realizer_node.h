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

// ROS
#include <ros/ros.h>

/// festival
#include <festival.h>

/// edinburgh sound tools
#include <EST.h>
#include <ling_class/EST_Item.h>
#include <ling_class/EST_Relation_list.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/param_loader.h>

#include <std_msgs/String.h>
#include <speech_realizer/SayText.h>

typedef std_msgs::String _StringMsg;

typedef speech_realizer::SayText _SayTextService;

class SpeechRealizerNode: public BaseNode
{
private:
  ros::NodeHandle nh_rel_;
  ros::ServiceServer say_text_server_;
  
  bool speak_locally_;

  EST_Val fest_default_val_float_;
  int fest_heap_size_;
  
  
public:
  SpeechRealizerNode(): BaseNode("SpeechRealizer"), nh_rel_("~"), 
			fest_default_val_float_(0.0f), fest_heap_size_(210000)
  {
  }

private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    speak_locally_ = uscauv::param::load<bool>(nh_rel_, "speak_locally", true);

    say_text_server_ = nh_rel_.advertiseService("say_text", &SpeechRealizerNode::textServiceCallback, this );

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

    bool success1 = festival_eval_command(command1);

    festival_say_text(est_text);

    return success1;
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
