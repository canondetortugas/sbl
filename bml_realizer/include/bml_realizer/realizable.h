/***************************************************************************
 *  include/bml_realizer/realizable.h
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


#ifndef SBL_BMLREALIZER_REALIZABLE
#define SBL_BMLREALIZER_REALIZABLE

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace realizer
{

  /**
   * Realize a behavior of some sort as a function of time index into the behavior.
   * 
   */
   class Realizable
   {
 
   public:
     typedef std::map<std::string, double> SyncPointMap;
    
    public:
      Realizable()
       {}

     /// Play back the behavior at time
     virtual void realize(double const & time) = 0;

     /** 
      * @return length of the behavior in seconds
      */
     virtual double getLength() = 0;
     
     /// Map from syncpoints (eg strokeStart) to times at which they occurr
     virtual SyncPointMap getSyncPoints() = 0;
     
    };

  /**
   * Realize a gesture by publishing tf frames describing it.
   * 
   */
  class RealizableGesture: public Realizable
  {
  protected:
    std::shared_ptr<tf::TransformBroadcaster> br_;

  public:
    RealizableGesture():
    {
      br_ = std::make_shared<tf::TransformBroadcaster>();
    }
    
    RealizableGesture(std::shared_ptr<tf::TransformBroadcaster> const & parent): 
      br_(parent)
    {}

    virtual std::string getMode() = 0;
    virtual void setMode(std::string const &) = 0;
    
    // virtual void realize(double const & time) = 0;
    // virtual double getLength() = 0;
  }

    
} // realizer

#endif // SBL_BMLREALIZER_REALIZABLE
