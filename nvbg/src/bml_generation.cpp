/***************************************************************************
 *  src/bml_generation.cpp
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

#include <ros/ros.h>
#include <nvbg/bml_generation.h>
#include <uscauv_common/macros.h>

/// xerces and pals
#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMWriter.hpp>

namespace nvbg
{
  std::string const PRIMARY_SPEECH_ID = "speech0";

  /** 
   * This must be run before any other BML functions are called
   * or else anything using the xerces API will fail
   * 
   * @return Zero if successful, non-zero otherwise
   */
  int initializeXMLPlatform()
  {
    try {
      xercesc::XMLPlatformUtils::Initialize();
    }
    catch (const xercesc::XMLException& toCatch) {
      char* message = xercesc::XMLString::transcode(toCatch.getMessage());
      ROS_ERROR_STREAM("Error during initialization! :\n"
		       << message );
      xercesc::XMLString::release(&message);
      return 1;
    }
    return 0;
  }

  xml_schema::namespace_infomap getBMLInfoMap()
  {
    xml_schema::namespace_infomap map;
    map[""].name = "http://www.bml-initiative.org/bml/bml-1.0";
    map[""].schema = "bml-1.0.xsd";
    return map;
  }
  
  std::string serializeXMLDocument( xercesc::DOMDocument & doc )
  {
    using namespace xercesc;
    
    /// LS - Load/Save
    XMLCh tempStr[100];
    XMLString::transcode("LS", tempStr, 99);
    DOMImplementation *impl          = DOMImplementationRegistry::getDOMImplementation(tempStr);
    DOMWriter         *theSerializer = ((DOMImplementationLS*)impl)->createDOMWriter();

    /// Pretty print
    if (theSerializer->canSetFeature(XMLUni::fgDOMWRTFormatPrettyPrint, true))
      theSerializer->setFeature(XMLUni::fgDOMWRTFormatPrettyPrint, true);

    /// Print document type definition
    if (theSerializer->canSetFeature(XMLUni::fgDocTypeString, true))
      theSerializer->setFeature(XMLUni::fgDocTypeString, true);

    DOMElement* root = doc.getDocumentElement();

    XMLCh * output_ch = theSerializer->writeToString( *root );
    std::string output ( XMLString::transcode( output_ch ) );
    
    return output;
  }

  std::string serializeXMLDocument( bml::bml & tree )
  {
    ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > doc = 
      bml::bml_( tree, getBMLInfoMap());

    return serializeXMLDocument( *doc );
  }

  /** 
   * Hack around XSD's poor support for mixed text XML elements
   * 
   * @param tree BML document data structure to add speech element to
   * @param text Speech to add
   * 
   * @return 
   */
  std::shared_ptr<xercesc::DOMDocument> addSpeech(std::shared_ptr<bml::bml> tree, 
						  parse::ParsedSpeech const & ps)
  {
    size_t const BUFFER_LEN = 1000;

    /// All xerces APIs use this c-strings 16-bit XMLCh type, so we use this buffer to convert all
    /// noncompatible strings that we want to pass to the API. XMLCh text_buffer[BUFFER_LEN];
    XMLCh text_buffer[BUFFER_LEN];
   
    /// Serialize tree structure to Xerces DOM Document object
    ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > doc = 
      bml::bml_( *tree, getBMLInfoMap());

    xercesc::DOMElement* root = doc->getDocumentElement();
    
    //////////////////////////////////////////////////////////////////////////////////////////////////
    bml::speech speech( PRIMARY_SPEECH_ID );
    bml::textType speech_text;
    
    /// element names aren't preserved when we serialize xsd::bml types
    xercesc::XMLString::transcode("speech", text_buffer, BUFFER_LEN-1);
    // May be doing the createElement part wrong
    xercesc::DOMElement* speech_element = doc->createElement(text_buffer);
    *speech_element << speech;
    root->appendChild( speech_element );

    xercesc::XMLString::transcode("text", text_buffer, BUFFER_LEN-1);
    xercesc::DOMElement* text_element = doc->createElement(text_buffer);
    *text_element << speech_text;
    speech_element->appendChild(text_element);

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /// Test out xerces text nodes
    // xercesc::XMLString::transcode("This is a test string", text_buffer, BUFFER_LEN-1);
    // xercesc::DOMText* str_element = doc->createTextNode(text_buffer);
    // text_element->appendChild(str_element);

    /// Add sync point for sentence beginning
    bml::syncType begin_sync, end_sync, s0begin_sync;
    begin_sync.id("begin");
    end_sync.id("end");
    s0begin_sync.id("s0_begin");

    xercesc::XMLString::transcode("sync", text_buffer, BUFFER_LEN-1);
    xercesc::DOMElement* begin_element = doc->createElement(text_buffer);
    xercesc::DOMElement* end_element = doc->createElement(text_buffer);
    xercesc::DOMElement* s0begin_element = doc->createElement(text_buffer);
    *begin_element << begin_sync;
    *end_element << end_sync;
    *s0begin_element << s0begin_sync;
    text_element->appendChild(begin_element);
    text_element->appendChild(s0begin_element);

    /// added_word_idx tracks words for which we've added syncpoints
    size_t word_idx = 0, added_word_idx = 0, sentence_idx = 0;
    bool closed = false;
    for( std::vector<std::string>::const_iterator token_it = ps.all_tokens_.begin();
	 token_it != ps.all_tokens_.end(); ++token_it )
      {
	std::string const & token = *token_it;

	ROS_ASSERT( token.size() <= BUFFER_LEN -1);
	xercesc::XMLString::transcode(token.c_str(), text_buffer, BUFFER_LEN-1);
	xercesc::DOMText* str_element = doc->createTextNode(text_buffer);

	/// We don't give word tags to spaces etc.
	if( !parse::isIgnored(token) )
	  {
	    /// sync point for the current word
	    std::stringstream ws;
	    ws << "w" << added_word_idx;
	    bml::syncType word_sync;
	    word_sync.id( ws.str() );
	    
	    xercesc::XMLString::transcode("sync", text_buffer, BUFFER_LEN-1);
	    xercesc::DOMElement* ws_element = doc->createElement(text_buffer);
	    *ws_element << word_sync;

	    /// word syncpoint
	    text_element->appendChild(ws_element);
	    /// word
	    text_element->appendChild(str_element);	

	    /// Add sentence tags for the current sentence
	    if( token == parse::SENTENCE_END)
	      {
		std::stringstream bstream, estream;
		estream << "s" << sentence_idx << "_end";
		++sentence_idx;
		bstream << "s" << sentence_idx << "_begin";
		
		bml::syncType sbegin_sync, send_sync;
		send_sync.id( estream.str() );
		sbegin_sync.id( bstream.str() );

		xercesc::XMLString::transcode("sync", text_buffer, BUFFER_LEN-1);
		xercesc::DOMElement* sbegin_element = doc->createElement(text_buffer);
		xercesc::DOMElement* send_element = doc->createElement(text_buffer);

		*sbegin_element << sbegin_sync;
		*send_element << send_sync;

		text_element->appendChild(send_element);
		
		/// Don't start a new sentence if this is the last token
		if( token_it + 1 == ps.all_tokens_.end() )
		  {
		    closed = true;
		  }
		else
		  {
		    text_element->appendChild(sbegin_element);
		  }
	      }

	    ++added_word_idx;
	  }
	else
	  {
	    text_element->appendChild(str_element);
	  }
	
	++word_idx;
      }

    /// Add end tag for final sentence if we haven't already
    if( !closed )
      {
	/// final sentence sync
	bml::syncType sf_end_sync;
	std::stringstream id;
	id << "s" << sentence_idx << "_end";
	sf_end_sync.id( id.str() );
	
	xercesc::XMLString::transcode("sync", text_buffer, BUFFER_LEN-1);
	xercesc::DOMElement* sf_element = doc->createElement(text_buffer);
	*sf_element << sf_end_sync;
	
	text_element->appendChild(sf_element);
      }

    /// Add end-of-sentence sync point
    text_element->appendChild(end_element);

    std::shared_ptr<xercesc::DOMDocument> result( doc.release() );

    return result;
  }

/** 
 * For now this function does a simplified version of BML generation. It doesn't insert any timing
 * constraints or speech. Instead, it simply checks to see if phrases have been
 * matched (in a very primitive / brittle way) and insert their corresponding behaviors into the
 * document if they have.
 * 
 * @param text Speech to process
 * @param eca Embodied conversational agent to enact the speech
 * @param behaviors Library of behaviors
 * @param rule_classes Rules for mapping phrases to behaviors
 * 
 * @return 
 */
  std::string generateBML(std::string const & text, std::string const & eca,
				      nvbg::behavior::BehaviorMap const &behaviors,
				      nvbg::rules::RuleClassMap const &rule_classes,
				      std::string request_id)
{
  /// Map from a sync point type (e.g. stokeStart) to a sync point
  typedef std::map<std::string, std::string> _SyncMap;
  
  /// Stores the number of occurrences of each behavior type
  std::map<std::string, size_t> behavior_idx;
  /// Initialize the count to zero for each behavior type
  for(std::set<std::string>::const_iterator behavior_type_it = behavior::TYPES.begin();
      behavior_type_it != behavior::TYPES.end(); ++behavior_type_it )
    {
      behavior_idx.insert( std::make_pair( *behavior_type_it, 0 ) );
    }

  /// Top-level BML element ID
  std::shared_ptr<bml::bml> tree( new bml::bml(request_id) );
  
  tree->characterId( eca );


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Add speech block////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  
  parse::ParsedSpeech ps = parse::parseSpeech( text );
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Match rules//////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  for( rules::RuleClassMap::const_iterator rule_class_it = rule_classes.begin();
       rule_class_it != rule_classes.end(); ++rule_class_it )
    {
      std::string const & class_name = rule_class_it->first;
      rules::RuleClass const & rule_class = rule_class_it->second;
      rules::RuleMap const & rules = rule_class.rules;

      for( rules::RuleMap::const_iterator rule_it = rules.begin();
	   rule_it != rules.end(); ++rule_it )
	{
	  std::string const & rule_phrase = parse::toLower(rule_it->first);
	  /// Mapping from behaviors to timings
	  rules::Rule const & rule = rule_it->second;
	  
	  //////////////////////////////////////////////////////////////////////////////////////////////////
	  // Try to match phrase in the speech /////////////////////////////////////////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////////////////
	  size_t phrase_idx = ps.processed_speech_.find(rule_phrase);
	  
	  if(phrase_idx != std::string::npos )
	    {
	      /// Iterate over all of the behaviors that this rule activates
	      for( rules::Rule::const_iterator rule_behavior_it = rule.begin(); 
		   rule_behavior_it != rule.end(); ++rule_behavior_it )
		{
		  std::string const & behavior_name = rule_behavior_it->first;
		  /// The timing constraints for this behavior
		  std::vector<timing::Timing> timings = rule_behavior_it->second;
		  
		  behavior::BehaviorMap::const_iterator behavior_it = behaviors.find( behavior_name );
		  /// We should have already pruned behaviors without definitions
		  ROS_ASSERT(behavior_it != behaviors.end());
		  
		  behavior::Behavior const & behavior = behavior_it->second;
		  
		  /////////////////////////////////////////////////////////////////////////////////////////
		  // Resolve behavior syncpoints //////////////////////////////////////////////////////////
		  /////////////////////////////////////////////////////////////////////////////////////////
		  _SyncMap sync;
		  
		  for( std::vector<timing::Timing>::const_iterator timing_it = timings.begin(); 
		       timing_it != timings.end(); ++timing_it )
		    {
		      timing::Timing const & timing = *timing_it;
		      
		      /// Can only have one syncpoint of each type
		      if( sync.find( timing.sync ) != sync.end() )
			{
			  ROS_WARN_STREAM("Duplicated syncpoint of type " << brk(timing.sync)
				   << " in rule " << brk(rule_phrase) << ", behavior "
				   << brk(behavior_name) );
			  continue;
			}
		      
		      if ( timing.scope == "speech" )
			{
			  std::stringstream ts;
			  /// arg_str is either 'begin' or 'end'
			  ts << PRIMARY_SPEECH_ID << ":" << timing.arg_str;
			  if( timing.offset )
			    ts << " + " << timing.offset;
			  
			  sync.insert (std::make_pair( timing.sync, ts.str() ) );
			}
		      else if( timing.scope == "sentence" )
			{
			  std::stringstream ts;
			  
			  /// Find which sentence our phrase is in
			  parse::IndexMap::iterator index_it = ps.char_to_sentence_.find(phrase_idx);
			  /// this map should have an entry for every character in the speech
			  /// so this shouldn't happen ever
			  ROS_ASSERT( index_it != ps.char_to_sentence_.end() );
			  size_t sentence_idx = index_it->second;
			  
			  ts << PRIMARY_SPEECH_ID << ":s" << sentence_idx 
			     << "_" << timing.arg_str;
			  if( timing.offset )
			    ts << " + " << timing.offset;
			  
			  sync.insert (std::make_pair( timing.sync, ts.str() ) );
			}
		      else if( timing.scope == "phrase" )
			{
			  /// Get which sentence we're in
			  parse::IndexMap::iterator sentence_it = ps.char_to_sentence_.find(phrase_idx);
			  /// these maps should have an entry for every character in the speech
			  /// so this shouldn't happen ever
			  ROS_ASSERT( sentence_it != ps.char_to_sentence_.end() );
			  size_t sentence_idx = sentence_it->second;

			  /// Get which non-ignored word the phrase starts with
			  parse::IndexMap::iterator word_it = ps.char_to_word_.find(phrase_idx);
			  ROS_ASSERT( word_it != ps.char_to_word_.end() );

			  size_t ref_word_idx = word_it->second + timing.arg_idx;

			  if( ref_word_idx >= ps.tokens_.size() )
			    {
			      ROS_ERROR_STREAM("Invalid word offset for syncpoint" << brk(timing.sync)
				   << " in rule " << brk(rule_phrase) << ", behavior "
				   << brk(behavior_name) );
			      continue;
			    }

			  std::stringstream ts;
			  ts << PRIMARY_SPEECH_ID << ":w" << ref_word_idx;
			  if( timing.offset )
			    ts << " + " << timing.offset;

			  sync.insert( std::make_pair( timing.sync, ts.str() ) );
			}
		      else
			{
			  /// This should have already been validated
			  ROS_ASSERT_MSG(false, "Invalid scope argument");
			}

		    }
		  
		  /////////////////////////////////////////////////////////////////////////////////////////
		  // Add behavior to BML document //////////////////////////////////////////////////////////
		  /////////////////////////////////////////////////////////////////////////////////////////
		  if( behavior.type == "gesture" )
		    {
		      size_t & idx = behavior_idx.at("gesture");
		      std::stringstream id;
		      id << "gesture" << idx;
		      
		      bml::gestureType gesture( id.str() );
		      
		      bml::openSetItem lexeme(1, behavior.lexeme );
		      gesture.lexeme( lexeme );

		      bml::closedSetItem mode(1, behavior.mode );
		      gesture.mode( mode );

		      for( _SyncMap::const_iterator sync_it = sync.begin(); sync_it != sync.end();
			   ++sync_it)
			{
			  std::string const & sync_type = sync_it->first;
			  std::string const & sync_ref = sync_it->second;

			  /// Gross
			  if( sync_type == "start" )
			    gesture.start( sync_ref );
			  else if( sync_type == "ready" )
			    gesture.ready( sync_ref );
			  else if( sync_type == "strokeStart" )
			    gesture.strokeStart( sync_ref );
			  else if( sync_type == "stroke" )
			    gesture.stroke( sync_ref );
			  else if( sync_type == "strokeEnd" )
			    gesture.strokeEnd( sync_ref );
			  else if( sync_type == "relax" )
			    gesture.relax( sync_ref );
			  else if( sync_type == "end" )
			    gesture.end( sync_ref );
			  else
			    ROS_ASSERT_MSG(false, "Invalid gesture sync type.");
			}

		      tree->gesture().push_back( gesture );
		      ++idx;
		    }
		  else if ( behavior.type == "face" )
		    {
		      size_t & idx = behavior_idx.at("face");
		      std::stringstream id;
		      id << "face" << idx;
		      
		      bml::openSetItem lexeme(1, behavior.lexeme);
		      bml::faceLexemeType face(id.str(), lexeme );
		      
		      face.amount( behavior.amount );

		      for( _SyncMap::const_iterator sync_it = sync.begin(); sync_it != sync.end();
			   ++sync_it)
			{
			  std::string const & sync_type = sync_it->first;
			  std::string const & sync_ref = sync_it->second;

			  /// Gross
			  if( sync_type == "start" )
			    face.start( sync_ref );
			  else if( sync_type == "attackPeak" )
			    face.attackPeak( sync_ref );
			  else if( sync_type == "relax" )
			    face.relax( sync_ref );
			  else if( sync_type == "end" )
			    face.end( sync_ref );
			  else
			    ROS_ASSERT_MSG(false, "Invalid face sync type.");
			}

		      tree->faceLexeme().push_back( face );

		      ++idx;
		    }
		  else if ( behavior.type == "head" )
		    {
		      size_t & idx = behavior_idx.at("head");
		      std::stringstream id;
		      id << "head" << idx;

		      bml::headType head( id.str() );

		      head.lexeme( behavior.lexeme );
		      
		      head.repetition( behavior.repetition );
		      head.amount( behavior.amount );

		      for( _SyncMap::const_iterator sync_it = sync.begin(); sync_it != sync.end();
			   ++sync_it)
			{
			  std::string const & sync_type = sync_it->first;
			  std::string const & sync_ref = sync_it->second;

			  /// Gross
			  if( sync_type == "start" )
			    head.start( sync_ref );
			  else if( sync_type == "ready" )
			    head.ready( sync_ref );
			  else if( sync_type == "strokeStart" )
			    head.strokeStart( sync_ref );
			  else if( sync_type == "stroke" )
			    head.stroke( sync_ref );
			  else if( sync_type == "strokeEnd" )
			    head.strokeEnd( sync_ref );
			  else if( sync_type == "relax" )
			    head.relax( sync_ref );
			  else if( sync_type == "end" )
			    head.end( sync_ref );
			  else
			    ROS_ASSERT_MSG(false, "Invalid head sync type.");
			}

		      tree->head().push_back( head );
		      ++idx;
		    }
		  else
		    {
		      ROS_ASSERT_MSG(false, "Invalid behavior type");
		    }
		  
		}
	    }
	}
    }
  
  std::shared_ptr<xercesc::DOMDocument> doc = addSpeech(tree, ps);
  std::string output = serializeXMLDocument( *doc );
  
  return output;
}

}
