/***************************************************************************
 *  include/nvbg/param_conversions.h
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


#ifndef SBL_NVBG_PARAMCONVERSIONS
#define SBL_NVBG_PARAMCONVERSIONS

// ROS
#include <ros/ros.h>

#include <nvbg/types.h>

#include <uscauv_common/param_loader.h>

using namespace uscauv;
using namespace nvbg;

USCAUV_DECLARE_PARAM_LOADER_CONVERSION(timing::Timing, param,

				       timing::Timing time;
				       time.scope = param::lookup<std::string>(param, "scope");
				       
				       if( !timing::SCOPE_TYPES.count( time.scope ))
					 {
					   throw std::invalid_argument("Invalid scope type");
					 }
				       

				       /// We will check if this a valid sync type later when we have context as to the behavior type later.
				       time.sync = param::lookup<std::string>(param, "sync");
				       
				       if( time.scope == "speech" || time.scope == "sentence")
					 {
					   std::string arg = param::lookup<std::string>(param, "arg");
					   
					   if( !timing::VALID_STRING_ARGS.count( arg) )
					     {
					       throw std::invalid_argument("Invalid timing argument");
					     }
					   
					   time.arg_str = arg;
					 }
				       else
					 {
					   time.arg_idx = param::lookup<int>(param, "arg");
					 }
				       
				       return time;
				       )

USCAUV_DECLARE_PARAM_LOADER_CONVERSION(rules::RuleClass, param,

				       rules::RuleClass rule_class;

				       rule_class.priority = param::lookup<int>(param, "priority");

				       rule_class.rules = param::lookup<rules::RuleMap>(param, "rules");

				       return rule_class;
				       )

USCAUV_DECLARE_PARAM_LOADER_CONVERSION(behavior::Behavior, param,
				       
				       behavior::Behavior bh;

				       bh.type = param::lookup<std::string>(param, "type");

				       if( !behavior::TYPES.count( bh.type ) )
					 {
					   throw std::invalid_argument("Invalid behavior type");
					 }

				       bh.lexeme = param::lookup<std::string>(param, "lexeme");
				       
				       if( bh.type == "gesture")
					 {
					   bh.mode = param::lookup<std::string>(param, "mode");
					   if (!behavior::GESTURE_MODES.count(bh.mode))
					     {
					       throw std::invalid_argument("Invalid gesture mode");
					     }

					 }
				       
				       if( bh.type == "face" || bh.type == "head" )
					 {
					   bh.amount = param::lookup<double>(param, "amount", 0.5);
					 }
				       
				       if( bh.type == "head" )
					 bh.repetition = param::lookup<int>(param, "repetition", 1);

				       return bh;
				       )
				       


#endif // SBL_NVBG_PARAMCONVERSIONS
 
