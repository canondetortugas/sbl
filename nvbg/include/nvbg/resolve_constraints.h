/***************************************************************************
 *  include/nvbg/resolve_constraints.h
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


#ifndef SBL_NVBG_RESOLVECONSTRAINTS
#define SBL_NVBG_RESOLVECONSTRAINTS

// ROS
#include <ros/ros.h>

#include <nvbg/constrained.h>

/// xerces and pals
#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMWriter.hpp>

#include <uscauv_common/macros.h>

namespace nvbg
{

  /// Remove behaviors that violate timing constraints from the DOM Document
  template<class __ConstrainedBehavior>
  void resolveConstraints(std::shared_ptr<xercesc::DOMDocument> const& doc, 
							   std::vector<std::shared_ptr<__ConstrainedBehavior> > const & cb)
  {
    size_t const BUFFER_LEN = 1000;
    XMLCh text_buffer[BUFFER_LEN];
    XMLCh id_buffer[BUFFER_LEN];
    
    typedef std::shared_ptr<xercesc::DOMDocument> _DOMPtr;
    typedef std::vector<__ConstrainedBehavior> _ConstrainedBehaviorVec;
    typedef std::set<std::string> _BehaviorNameSet;
    
    _BehaviorNameSet offending_behaviors = chooseBehaviorCombination(cb);
    std::stringstream of;
    std::copy(offending_behaviors.begin(), offending_behaviors.end(), std::ostream_iterator<std::string>(of, ", "));
    ROS_INFO_STREAM("Offending behaviors: " << of.str());

    // std::shared_ptr<xercesc::DOMDocument> output( doc->cloneNode(true) );
    
    xercesc::DOMElement* root = doc->getDocumentElement();

    xercesc::XMLString::transcode( __ConstrainedBehavior::getTagName().c_str(), text_buffer, BUFFER_LEN-1);
    xercesc::DOMNodeList* nodes = root->getElementsByTagName(text_buffer);
    
    xercesc::XMLString::transcode( "id", text_buffer, BUFFER_LEN-1);
    for(size_t node_idx = 0; node_idx < nodes->getLength(); ++node_idx)
      {
	/// TODO: Verify that the node is actually an element
	xercesc::DOMElement* node = dynamic_cast<xercesc::DOMElement*>( nodes->item(node_idx) );
	XMLCh const * xml_id = node->getAttribute(text_buffer);
	char * id = xercesc::XMLString::transcode(xml_id);
	std::string id_str(id);
	
	ROS_ASSERT_MSG( id_str.size() != 0, "Behavior element has no id attribute");
	
	ROS_INFO_STREAM("Processed bml node " << id_str);

	if( offending_behaviors.count(id_str))
	  {
	    root->removeChild( node );
	    ROS_INFO_STREAM("Removing behavior " << brk(id_str));
	  }
	
	xercesc::XMLString::release(&id);
      }
    
    // return output;
    return;
  }
  
  /** 
   * Return binary combination string with the highest priority
   * 
   * @param int 
   * @param scores 
   * 
   * @return 
   */
  static long unsigned int chooseBehaviorCombinationHighestPriority( std::map<long unsigned int, unsigned int>  & scores )
  {
    typedef std::map<long unsigned int, unsigned int> _CombinationScores;
    _CombinationScores::iterator it = 
      std::min_element( scores.begin(), scores.end(), [](std::pair<long unsigned int, unsigned int> const & a,
							 std::pair<long unsigned int, unsigned int> const & b)->bool
			{return a.second < b.second; });
			return it->first;
			}

      /** 
   * Choose a behavior combination, and return a list of behaviors that must be removed to make it so.
   *x 
   * @param cb 
   * 
   * @return 
   */
  template<class __ConstrainedBehavior>
  std::set<std::string> chooseBehaviorCombination(std::vector<std::shared_ptr<__ConstrainedBehavior> > const & cb)
  {
    typedef std::map<long unsigned int, unsigned int> _CombinationScores;

    _CombinationScores scores = scoreBehaviorCombinationsExhaustive(cb);
    
    
    
    long unsigned int combination = chooseBehaviorCombinationHighestPriority(scores);

    ROS_INFO_STREAM("Chose combination " << combination);

    return getRemovedBehaviorIDs(cb, combination);
  }

  /** 
   * Get a map from a binary string describing behavior combination to the score of that combination
   * combinations that 
   * 
   * @param cb 
   * 
   * @return 
   */
  template<class __ConstrainedBehavior>
  std::map<long unsigned int, unsigned int> scoreBehaviorCombinationsExhaustive(std::vector<std::shared_ptr<__ConstrainedBehavior> > const & cb)
  {
    typedef std::map<long unsigned int, unsigned int> _CombinationScores;
    typedef std::vector<std::shared_ptr<__ConstrainedBehavior> > _CBVec;

    if( cb.size() >= 20)
      ROS_WARN("Large number of constraints. Resolving might take some time.");

    _CombinationScores scores;
    
    long unsigned int const  n_combinations = static_cast<long unsigned int>(1) << cb.size();

    for(long unsigned int combination = 1; combination < n_combinations; ++combination)
      {
	unsigned int priority = 0;
	
	std::vector<std::shared_ptr<__ConstrainedBehavior> > candidates = getBehaviorCombination(cb, combination);
	
	/// Check every pair of behaviors in this combination for collision
	for(typename _CBVec::const_iterator cb1_it = candidates.begin(); cb1_it != candidates.end(); ++cb1_it)
	  {
	    for(typename _CBVec::const_iterator cb2_it = cb1_it + 1; cb2_it != candidates.end(); ++cb2_it)
	      {
		/// this behavior pair collides, so this combination doesn't get added to the list of valid combinations
		if( __ConstrainedBehavior::checkCollision( **cb1_it, **cb2_it ))
		  goto bailout;
	      }
	    priority += (*cb1_it)->priority;
	  }

	scores.insert( std::make_pair( combination, priority ));
	
      bailout:
	;
      }

    return scores;
  }
  
  /** 
   * Get the behaviors that a binary combination string includes
   * 
   * @param cb 
   * @param combination 
   * 
   * @return 
   */
  template<class __ConstrainedBehavior>
  std::vector<std::shared_ptr<__ConstrainedBehavior> > getBehaviorCombination(std::vector<std::shared_ptr<__ConstrainedBehavior> > const & cb, long unsigned int const & combination)
  {
    std::vector<std::shared_ptr<__ConstrainedBehavior> > candidates;

    /// Get the behaviors that the combination we are testing includes
    for(size_t behavior_idx = 0; behavior_idx < cb.size(); ++behavior_idx)
      {
	bool const included = combination & (static_cast<long unsigned int>(1) << behavior_idx);

	if(included)
	  candidates.push_back( cb[behavior_idx] );
      }
    
    return candidates;
  }

  /** 
   * Get the IDs of behaviors that a binary combination string would remove
   * 
   * @param cb 
   * @param combination 
   * 
   * @return 
   */
template<class __ConstrainedBehavior>
std::set<std::string > getRemovedBehaviorIDs(std::vector<std::shared_ptr<__ConstrainedBehavior> > const & cb, long unsigned int const & combination)
  {
    std::set<std::string> output;
    
    /// Get the behaviors that the combination we are testing includes
    for(size_t behavior_idx = 0; behavior_idx < cb.size(); ++behavior_idx)
      {
	bool const included = combination & (static_cast<long unsigned int>(1) << behavior_idx);

	if(!included)
	  output.insert( cb[behavior_idx]->id );
      }
    
    return output;
  }

} // nvbg

#endif // SBL_NVBG_RESOLVECONSTRAINTS
