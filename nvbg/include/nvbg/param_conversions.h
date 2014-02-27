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
#include <nvbg/parsing.h>

/// TODO: Figure out why these only work if I define them in CMakeLists
// #define USCAUV_PARAM_LOADER_DISALLOW_EMPTY_VECTORS
// #define USCAUV_PARAM_LOADER_DISALLOW_EMPTY_MAPS
#include <uscauv_common/param_loader.h>
#include <uscauv_common/macros.h>

using namespace uscauv;
using namespace nvbg;

/// Perform any validation that can't be performed during autoload
void postProcessRules( rules::RuleClassMap & rcm, behavior::BehaviorMap & bh );
bool validateRuleTimings( std::vector<timing::Timing> const & timings, std::set<std::string> sync_types);

USCAUV_DECLARE_PARAM_LOADER_CONVERSION(timing::Timing, param,

				       timing::Timing time;
				       time.type = param::lookup<std::string>(param, "type");
				       
				       if( !timing::TYPES.count( time.type ))
					 {
					   throw std::invalid_argument("Invalid timing type");
					 }
				       
				       /// We will check if this a valid sync type later when we have context as to the behavior type later.
				       time.sync = param::lookup<std::string>(param, "sync");
				       
				       time.pos = param::lookup<std::string>(param, "pos");
				       
				       if( !timing::POS_ARGS.count( time.pos ))
					 {
					   throw std::invalid_argument("Invalid timing position argument");
					 }

				       
				       if( time.type == "word")
					 {
					   time.word_idx = param::lookup<int>(param, "word_index");
					   if( time.word_idx < 0 )
					     throw std::invalid_argument("Word index must be non-negative");
					 }
				       
				       /// Quiet
				       time.offset = param::lookup<double>(param, "offset", 0.0, true);
				       
				       return time;
				       )

USCAUV_DECLARE_PARAM_LOADER_CONVERSION(rules::RuleClass, param,

				       rules::RuleClass rule_class;

				       rule_class.priority = param::lookup<int>(param, "priority");

				       rule_class.rules = param::lookup<rules::RuleMap>(param, "rules");

				       /// Preprocess all of the rule strings
				       rules::RuleMap lrules = rule_class.rules;
				       for( rules::RuleMap::const_iterator rule_it = rule_class.rules.begin();
					    rule_it != rule_class.rules.begin(); ++rule_it )
					 {
					   lrules.insert( std::make_pair( parse::toLower( rule_it->first),
									  rule_it->second ) );
					 }
				       
				       rule_class.rules = lrules;

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
