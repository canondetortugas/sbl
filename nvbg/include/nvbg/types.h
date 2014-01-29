/***************************************************************************
 *  include/nvbg/types.h
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


#ifndef SBL_NVBG_TYPES
#define SBL_NVBG_TYPES

// ROS
#include <ros/ros.h>

namespace nvbg
{
  namespace timing
  {
    struct Timing
    {
      std::string scope;
      std::string sync;
      std::string arg_str;
      size_t arg_idx;
    };
  }

  namespace rules
  {
    /// Mapping from behavior name to timing
    // typedef std::map<std::string, std::vector<timing::Timing> > Rule;
    typedef std::map<std::string, std::vector<timing::Timing> > Rule;

    /// Mapping from strings to behaviors and rules
    typedef std::map<std::string, Rule> RuleMap;
    

    struct RuleClass
    {
      size_t priority;

      RuleMap rules;
    };

    typedef std::map<std::string, RuleClass> RuleClassMap;
  }


  namespace behavior
  {
    struct Behavior
    {
      std::string type;
      std::string lexeme;
      std::string mode;
      double amount;
      size_t repetition;
    };

    typedef std::map<std::string, Behavior> BehaviorMap;
  }

    
} // nvbg

namespace nvbg
{

  namespace behavior
  {
    extern std::set<std::string> const TYPES;

    extern std::set<std::string> const GESTURE_MODES;
  }
  
  namespace timing
  {
    extern std::set<std::string> const SCOPE_TYPES;
    extern std::set<std::string> const VALID_STRING_ARGS;
  }
}

#endif // SBL_NVBG_TYPES
