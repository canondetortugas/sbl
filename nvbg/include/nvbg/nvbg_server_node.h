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

// ROS
#include <ros/ros.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/macros.h>
#include <uscauv_common/param_loader.h>

#include <sbl_msgs/SimpleNVBGRequest.h>

typedef sbl_msgs::SimpleNVBGRequest _SimpleNVBGRequest;


class NVBGServerNode: public BaseNode, public MultiReconfigure
{
 private:
  ros::Subscriber simple_request_sub_;
  ros::NodeHandle nh_rel_;
  

 public:
 NVBGServerNode(): BaseNode("NVBGServer"), nh_rel_("~")
   {
   }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
     {
       simple_request_sub_ = nh_rel_.subscribe<_SimpleNVBGRequest>("simple_requests", 10,
								   &NVBGServerNode::simpleRequestCallback, this);
     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {

     }

  void simpleRequestCallback( _SimpleNVBGRequest::ConstPtr const & msg )
  {
    ROS_INFO_STREAM("Got request for ECA " << brk( msg->eca ) << " with text " << brk( msg->text) );
  }

};

#endif // SBL_NVBG_NVBGSERVER
