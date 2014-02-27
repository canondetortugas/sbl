/***************************************************************************
 *  src/param_conversions.cpp
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


#include <nvbg/param_conversions.h>

void postProcessRules( rules::RuleClassMap & rcm, behavior::BehaviorMap & bh )
{
  /// A great case for why not to do this sort of thing in C++

  /// Iterate over map from rule class name to rules class info
  for( rules::RuleClassMap::value_type & rc_it : rcm )
    {
      /// Iterate over map from phrase to rules for phrase
      for( rules::RuleMap::value_type & rm_it : rc_it.second.rules )
	{
	  /// Iterate over map from behavior names to timings
	  rules::Rule::iterator rule_it = rm_it.second.begin();
	  while( rule_it != rm_it.second.end())
	    {
	      behavior::BehaviorMap::const_iterator behavior_it = bh.find( rule_it->first);
	      std::set<std::string> sync_set;
	      
	      if( behavior_it == bh.end())
		{
		  ROS_WARN_STREAM("Rule references missing behavior " << brk(rule_it->first) << ".");
		  rule_it = rm_it.second.erase( rule_it );
		  continue;
		}
	      else
		{
		  if( behavior_it->second.type == "gesture" )
		    sync_set = behavior::GESTURE_SYNC_TYPES;
		  else if( behavior_it->second.type == "head" )
		    sync_set = behavior::HEAD_SYNC_TYPES;
		  else if( behavior_it->second.type == "face" )
		    sync_set = behavior::FACE_SYNC_TYPES;
		  else
		    ROS_ASSERT_MSG(false, "Invalid behavior type");
		}
	      
	      
	      if( !validateRuleTimings(rule_it->second, sync_set))
		{
		  ROS_WARN_STREAM("Behavior " << brk(rule_it->first) <<
				  " in rule " << brk(rm_it.first) <<
				  " has invalid timings. Removing...");
		  /// Remove offending behavior
		  rule_it = rm_it.second.erase( rule_it );
		  continue;
		}
	      
		++rule_it;
	    }
	}
    }
}

/** 
 * Count occurrences of each sync point type. Make sure there are no duplicates and that start/end exist
 * 
 * @param timings Timings to validate
 * 
 * @return true if valid, false otherwise
 */
bool validateRuleTimings( std::vector<timing::Timing> const & timings, std::set<std::string> sync_types)
{
  
  std::map<std::string, size_t> timing_counts;
  
  for( std::set<std::string>::const_iterator type_it = sync_types.begin();
       type_it != sync_types.end(); ++type_it)
    {
      timing_counts.insert( std::make_pair( *type_it, 0));
    }
  
  /// Iterate over timings for behavior
  for( std::vector<timing::Timing>::value_type const & timing : timings )
    {
      /// Should have been validated already
      if( !sync_types.count( timing.sync ) )
	return false;
      
      ++timing_counts[ timing.sync ];
    }

  for( std::map<std::string, size_t>::iterator count_it = timing_counts.begin();
       count_it != timing_counts.end(); ++count_it)
    {
      if( count_it->second > 1 )
	{
	  ROS_WARN("Duplicate syncpoint types in single behavior");
	  return false;
	}
    }

  if( timing_counts["start"] != 1)
    {
      ROS_WARN("Timing contains no start syncpoint");
      return false;
    }
  if( timing_counts["end"] != 1)
    {
      ROS_WARN("Timing contains no end syncpoint");
      return false;
    }


  return true;
}
