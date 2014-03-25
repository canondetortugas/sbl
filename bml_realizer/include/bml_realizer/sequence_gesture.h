/***************************************************************************
 *  include/bml_realizer/sequence_gesture.h
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


#ifndef SBL_BMLREALIZER_SEQUENCEGESTURE
#define SBL_BMLREALIZER_SEQUENCEGESTURE

// ROS
#include <ros/ros.h>

#include <nvbg/types.h>

#include <bml_realizer/realizable.h>

#include <uscauv_common/param_loader.h>
#include <uscauv_common/macros.h>

namespace realizer
{

  class SequenceGesture: public RealizableGesture
  {
  public:
    struct GestureFrames
    {
      std::vector<tf::StampedTransform> right_;
      std::vector<tf::StampedTransform> left_;
    };
          
    /// a list of times
    typedef std::set<double> TimeSet;
    /// Mapping from time to gesture frames
    typedef std::map<double, GestureFrames> FrameSequence;

    struct Storage
    {
      bool force_mode_;
      std::string mode_;
      double length_;
      /// Map from syncpoint names to times when they occur
      SyncPointMap syncpoints_;
      FrameSequence sequence_;
      TimeSet times_;
    };
     
  private:
    std::shared_ptr<Storage> storage_;

  public:
    SequenceGesture(std::shared_ptr<Storage> const & storage): RealizableGesture(), storage_(storage)
    {}
    SequenceGesture(std::shared_ptr<Storage> const & storage,
		    std::shared_ptr<tf::TransformBroadcaster> const & parent_br): 
      RealizableGesture(parent_br), storage_(storage)
      {}
    
    virtual double getLength()
    {
      return storage_->length_;
    }
  
    virtual SyncPointMap getSyncPoints()
    {
      return storage_->syncpoints_;
    }

    virtual std::string getMode()
    {
      return storage_->mode_;
    }
    virtual void setMode(std::string const & mode)
    {
      if( !storage_->force_mode_ )
	storage_->mode_ = mode;
    }

    virtual void realize(double const & time)
    {
      if ( time < 0 )
	throw std::invalid_argument("Times must be non-negative.");

      TimeSet::iterator time_it = storage_->times_.lower_bound(time);
      
      if( time_it == storage_->times_.end() )
	throw std::invalid_argument("Time exceeds gesture length.");

      GestureFrames & frames = storage_->sequence_.at( *time_it );

      std::vector<tf::StampedTransform> output;
      ros::Time now = ros::Time::now();

      for( std::vector<tf::StampedTransform>::value_type & transform : frames.left_)
	{
	  transform.stamp_ = now;
	}
      for( std::vector<tf::StampedTransform>::value_type & transform : frames.right_)
	{
	  transform.stamp_ = now;
	}
      
      br_->sendTransform(frames.left_);
      br_->sendTransform(frames.right_);
    }
    
  };

} // realizer
    
USCAUV_DECLARE_PARAM_LOADER_CONVERSION(tf::Transform, param,
				       double const x = uscauv::param::lookup<double>(param, "x");
				       double const y = uscauv::param::lookup<double>(param, "y");
				       double const z = uscauv::param::lookup<double>(param, "z");
				       double const roll = uscauv::param::lookup<double>(param, "roll");
				       double const yaw = uscauv::param::lookup<double>(param, "yaw");
				       double const pitch = uscauv::param::lookup<double>(param, "pitch");

				       tf::Vector3 vec(x,y,z);
				       tf::Quaternion quat; quat.setRPY(roll, pitch, yaw);
				       
				       return tf::Transform(quat, vec);
				       )

/// Doesn't bother loading timestamp
USCAUV_DECLARE_PARAM_LOADER_CONVERSION(tf::StampedTransform, param,
				       
				       tf::Transform const transform = uscauv::param::lookup<tf::Transform>(param, "transform");
				       std::string const child_frame = uscauv::param::lookup<std::string>(param, "child_frame");
				       std::string const parent_frame = uscauv::param::lookup<std::string>(param, "parent_frame");
				       /// Arbitrary - this will be changed later
				       ros::Time now = ros::Time::now();
				       
				       return tf::StampedTransform(transform, now, parent_frame, child_frame);
				       )

USCAUV_DECLARE_PARAM_LOADER_CONVERSION(realizer::SequenceGesture::GestureFrames, param,
				       realizer::SequenceGesture::GestureFrames frames;
				       frames.left_ = uscauv::param::lookup<std::vector<tf::StampedTransform> >
				       (param, "left", std::vector<tf::StampedTransform>(), true);
				       frames.right_ = uscauv::param::lookup<std::vector<tf::StampedTransform> >
				       (param, "right", std::vector<tf::StampedTransform>(), true);
				       
				       return frames;
				       )				       

/// Assumes that param is a vector of frames with times and syncpoints
USCAUV_DECLARE_PARAM_LOADER_CONVERSION(realizer::SequenceGesture::Storage, param,
				       typedef XmlRpc::XmlRpcValue _XmlVal;
				       
				       realizer::SequenceGesture::Storage storage;
				       
				       bool use_left = false;
				       bool use_right = false;
				       double last_time = -1;
				       for(size_t idx = 0; idx < param.size(); ++idx)
					 {
					   /// Get the tf frames for the gesture at this point in time.
					   realizer::SequenceGesture::GestureFrames frames = 
					     uscauv::param::lookup<realizer::SequenceGesture::GestureFrames>
					     (param, "frames");

					   double const time = uscauv::param::lookup<double>(param, "time");
					   if( idx == 0 && time != 0.0)
					     {
					       throw std::invalid_argument("Gesture sequence must start at time zero");
					     }
					   
					   if( !time > last_time )
					     {
					       ROS_WARN("Got frame with invalid time.");
					       continue;
					     }
					   else
					     last_time = time;

					   storage.times_.insert(time);
					   storage.sequence_.insert( std::make_pair(time, frames) );

					   std::string syncpoint = uscauv::param::lookup<std::string>(param, "syncpoint", "", true);
					   
					   if( !nvbg::behavior::GESTURE_SYNC_TYPES.count(syncpoint))
					     {
					       ROS_WARN_STREAM("Got invalid syncpoint type " << brk(syncpoint) << ".");
					     }
					   else
					     {
					       if( storage.syncpoints_.count( syncpoint ) )
						 {
						   ROS_WARN_STREAM("Syncpoint of type " << brk(syncpoint) << 
								   " already exists. Ignoring...");
						 }
					       else
						 {
						   storage.syncpoints_.insert( std::make_pair(syncpoint, time ));
						 }
					     }
					   
					   /// Figure out whether or not this gesture actually uses both arms
					   if( frames.right_.size() )
					     use_right = true;
					   if( frames.left_.size() )
					     use_left = true;
					 }

				       if( last_time < 0)
					 {
					   throw std::invalid_argument("No sequence frames were loaded successfully.");
					 }
				       else
					 {
					   storage.length_ = last_time;
					 }
				       
				       if( use_left && !use_right )
					 {
					   storage.mode_ = "LEFT_HAND";
					   storage.force_mode_ = true;
					 }
				       else if( !use_left && use_right )
					 {
					   storage.mode_ = "RIGHT_HAND";
					   storage.force_mode_ = true;
					 }
				       else
					 {
					   storage.mode_ = "BOTH_HANDS";
					   storage.force_mode_ = false;
					 }

				       
				       return storage;
				       )

#endif // SBL_BMLREALIZER_SEQUENCEGESTURE
