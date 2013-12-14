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

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/macros.h>
#include <uscauv_common/param_loader.h>

/// sbl
#include <sbl_msgs/SimpleNVBGRequest.h>
#include <bml_cpp/bml-1.0.h>

/// boost
#include <boost/tokenizer.hpp>

typedef sbl_msgs::SimpleNVBGRequest _SimpleNVBGRequest;
typedef XmlRpc::XmlRpcValue _XmlVal;

struct BehaviorType
{
  unsigned int priority_;
  std::vector<std::string> behaviors_;
};

/// Param loader will deal with exceptions automatically so we don't have to check success directly
USCAUV_DECLARE_PARAM_LOADER_CONVERSION( BehaviorType, param,
					BehaviorType behavior_type;
					behavior_type.priority_ = uscauv::param::lookup<int>(param, "priority");
					behavior_type.behaviors_ = uscauv::param::lookup<std::vector<std::string> >(param, "behaviors");
					return behavior_type; )

typedef std::map<std::string, BehaviorType> _NamedBehaviorMap;
typedef std::multimap<std::string, std::string> _MultiStringMap;



/// TODO: Make sure behavior clases actually have behaviors in them
class NVBGServerNode: public BaseNode, public MultiReconfigure
{
 private:
  ros::Subscriber simple_request_sub_;
  ros::NodeHandle nh_base_, nh_rel_;

  _NamedBehaviorMap behaviors_;
  _MultiStringMap word_behavior_map_;


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

       /// Load parameters
       _XmlVal words = uscauv::param::load<_XmlVal>(nh_base_, "nvbg/words");

       /// Create mapping from words to behavior types
       for( _XmlVal::ValueStruct::value_type & elem : words )
	 {
	   std::vector<std::string> behavior_names = 
	     uscauv::param::XmlRpcValueConverter<std::vector<std::string> >::convert( elem.second );
	   for( std::string const & behavior_name : behavior_names )
	     {
	       word_behavior_map_.insert( std::make_pair( elem.first, behavior_name ));
	     }
	 }
       
       /// This is possible because we declared a conversion for the value type earlier
       behaviors_ = uscauv::param::load<_NamedBehaviorMap>( nh_base_, "nvbg/behaviors");
       
     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {
       
     }

  void simpleRequestCallback( _SimpleNVBGRequest::ConstPtr const & msg )
  {
    ROS_INFO_STREAM("Got request for ECA " << brk( msg->eca ) << " with text " << brk( msg->text) );

    try
      {
	std::auto_ptr<bml::bml> tree( new bml::bml("server_request") );

	tree->characterId( msg->eca );

	/// Very simple parsing
	/* std::vector<std::string> tokens; */
	/* std::istringstream iss( msg->text ); */
	/* std::copy(std::istream_iterator<std::string>(iss), */
	/* 	  std::istream_iterator<std::string>(), */
	/* 	  std::back_inserter<std::vector<std::string> >(tokens) ); */
	
	boost::tokenizer<> tokenizer(msg->text);
	
	unsigned id_idx = 0;
	for( boost::tokenizer<>::iterator token = tokenizer.begin(); token != tokenizer.end(); ++token )
	  {
	    std::string match_string = *token;
	    std::transform( match_string.begin(), match_string.end(), 
			    match_string.begin(), ::tolower );
	    
	    std::cout << match_string << std::endl;
	    
	    std::pair<_MultiStringMap::iterator, _MultiStringMap::iterator> match_range = 
	      word_behavior_map_.equal_range( match_string );

	    if( match_range.first != match_range.second )
	      {
		BehaviorType matched_type = (behaviors_.find(match_range.first->second ))->second;
		std::stringstream id;
		id << "gesture" << id_idx;
		bml::gestureType gesture ( id.str() );
		bml::openSetItem item (1,  matched_type.behaviors_[0] );
		gesture.lexeme(item);

		tree->gesture().push_back( gesture );

		++id_idx;
	      }

	  }
	
	/// Wrap everything up and send it off
	xml_schema::namespace_infomap map;
	map[""].name = "";
	map[""].schema = "bml-1.0.xsd";

	bml::bml_( std::cout, *tree, map);
		
      }
    catch (const xml_schema::exception& e)
      {
	ROS_ERROR_STREAM( e );
	return;
      }

  }

};

#endif // SBL_NVBG_NVBGSERVER
