/***************************************************************************
 *  include/bml_realizer/realizer_node.h
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


#ifndef SBL_BMLREALIZER_REALIZER
#define SBL_BMLREALIZER_REALIZER

// ROS
#include <ros/ros.h>

#include <std_msgs/String.h>

// uscauv
#include <uscauv_common/base_node.h>

#include <nvbg/bml_generation.h>

/// realizer
#include <bml_realizer/bml_processing.h>

class RealizerNode: public BaseNode
{
private:
  ros::NodeHandle nh_rel_;
  ros::Subscriber bml_sub_;
  
  
public:
  RealizerNode(): BaseNode("Realizer"), nh_rel_("~")
  {
  }

private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    bml_sub_ = nh_rel_.subscribe("bml", 1, &RealizerNode::bmlCallback, this);
   
    nvbg::initializeXMLPlatform();
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {

  }

  void bmlCallback( std_msgs::String::ConstPtr const & msg)
  {
    std::shared_ptr<xercesc::DOMDocument> doc = realizer::parseBML( msg->data );

    if( !doc )
      {
	ROS_ERROR("Failed to parse BML document.");
	return;
      }

    std::map<std::string, std::string> named_speeches = realizer::extractSpeech(doc);

  }

};

#endif // SBL_BMLREALIZER_REALIZER
