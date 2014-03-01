/***************************************************************************
 *  include/nvbg/constrained.h
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


#ifndef SBL_NVBG_CONSTRAINED
#define SBL_NVBG_CONSTRAINED

// ROS
#include <ros/ros.h>

namespace nvbg
{
  
  struct ConstrainedBehavior
  {
    /// Indices of start and end times
    size_t start, end;
    unsigned int priority;
    std::string id;


    /** 
     * @return True of behaviors collide, false otherwise
     */
    template<class __BehaviorType>
    static bool checkCollision(__BehaviorType const & c1, __BehaviorType const & c2)
    {
      /// Check whether or not timings overlap
      bool const valid_timing = ((c1.start < c2.start && c1.end <= c2.start) ||
				 (c2.start < c1.start && c2.end <= c1.start));
	
      return !valid_timing;
    }
    
  };
    
  struct ConstrainedGesture: public ConstrainedBehavior
  {
    std::string mode;
    

    static bool checkCollision(ConstrainedGesture const & c1, ConstrainedGesture const & c2)
    {
      bool const same_arm = (c1.mode == "BOTH_HANDS" || c2.mode == "BOTH_HANDS" ||
			     (c1.mode == "RIGHT_HAND" && c2.mode == "RIGHT_HAND") ||
			     (c1.mode == "LEFT_HAND" && c2.mode == "RIGHT_HAND"));
    
      /// Gestures collide if they try to use the same arm at the same time
      return same_arm && ConstrainedBehavior::checkCollision(c1, c2);
    }
    
    static std::string const getTagName(){ return "gesture";}

  };

  struct ConstrainedHead: public ConstrainedBehavior
  {
    static bool checkCollision(ConstrainedHead const & c1, ConstrainedHead const & c2)
    {
      return ConstrainedBehavior::checkCollision(c1, c2);
    }

    static std::string const getTagName(){ return "head";}
  };

  struct ConstrainedFace: public ConstrainedBehavior
  {
    static bool checkCollision(ConstrainedFace const & c1, ConstrainedFace const & c2)
    {
      return ConstrainedBehavior::checkCollision(c1, c2);
    }

    static std::string const getTagName(){ return "faceLexeme";}
  };
    
} // nvbg

#endif // SBL_NVBG_CONSTRAINED
