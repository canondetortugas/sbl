/***************************************************************************
 *  include/nvbg/NVBG_server_node.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster (turtlecannon@gmail.com)
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
 *  * Neither the name of SBL nor the names of its
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


#ifndef SBL_NVBG_NVBGSERVER
#define SBL_NVBG_NVBGSERVER

#include <map>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/macros.h>
#include <uscauv_common/param_loader.h>

/// sbl
#include <sbl_msgs/SimpleNVBGRequest.h>
#include <bml_cpp/bml-1.0.h>
#include <nvbg/types.h>
#include <nvbg/param_conversions.h>
#include <nvbg/bml_generation.h>

typedef sbl_msgs::SimpleNVBGRequest _SimpleNVBGRequest;
typedef XmlRpc::XmlRpcValue _XmlVal;

using namespace nvbg;

class NVBGServerNode: public BaseNode, public MultiReconfigure
{
 private:
  
  ros::Subscriber simple_request_sub_;
  ros::Publisher bml_pub_;
  ros::NodeHandle nh_base_, nh_rel_;

  rules::RuleClassMap rules_;
  behavior::BehaviorMap behaviors_;
  
  size_t request_idx;


 public:
  NVBGServerNode(): BaseNode("NVBGServer"), nh_rel_("~"), request_idx(0)
   {
   }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       simple_request_sub_ = nh_rel_.subscribe<_SimpleNVBGRequest>("simple_requests", 10,
								   &NVBGServerNode::simpleRequestCallback, this);

       bml_pub_ = nh_rel_.advertise<std_msgs::String>("bml", 10);
       
       rules_ = uscauv::param::load<rules::RuleClassMap>(nh_base_, "nvbg/rules");
       
       behaviors_ = uscauv::param::load<behavior::BehaviorMap>(nh_base_, "nvbg/behaviors");
       postProcessRules(rules_, behaviors_);

       /// Various preprocessing things that should be done
       /// TODO: Verify that all behaviors referenced in the rules are actually loaded
       /// TODO: Check that scope arguments for timings are valid
       /// TODO: Check for duplicate syncpoint assignments in each timing
       /// (this is currently done in bml_gen.cpp - change that to an assert)
       
       /// Must be run before any BML functionality where we use xerces directly
       nvbg::initializeXMLPlatform();
     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {
       
     }

  void simpleRequestCallback( _SimpleNVBGRequest::ConstPtr const & msg )
  {
    ROS_INFO_STREAM("Got request for ECA " << brk( msg->eca ) << " with text " << brk( quote(msg->text) ) );

    try
      {
	std::stringstream id;
	id << "request" << request_idx;

	std::string output_str = nvbg::generateBML(msg->text, msg->eca, 
							   behaviors_, rules_, id.str() );
	
	std_msgs::String output;
	output.data = output_str;
	
	bml_pub_.publish(output);
		
	ROS_INFO_STREAM(output_str);		
      }
    catch (const xml_schema::exception& e)
      {
	ROS_ERROR_STREAM( e );
	return;
      }

    ++request_idx;
  }

};

#endif // SBL_NVBG_NVBGSERVER
