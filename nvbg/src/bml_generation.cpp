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

/// xerces and pals
#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMWriter.hpp>

namespace nvbg
{
  namespace parse
  {
    
  }

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

  parse::ParsedSpeech parseSpeech(std::string const & speech)
  {
    
    
    
  }

  /** 
   * Hack around XSD's poor support for mixed text XML elements
   * 
   * @param tree BML document data structure to add speech element to
   * @param text Speech to add
   * 
   * @return 
   */
  std::shared_ptr<xercesc::DOMDocument> addSpeech(std::shared_ptr<bml::bml> tree, std::string const & text)
  {
    /// All xerces APIs use this c-strings 16-bit XMLCh type, so we use this buffer to convert all
    /// noncompatible strings that we want to pass to the API.
    XMLCh text_buffer[1000];
   
    /// Serialize tree structure to Xerces DOM Document object
    ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > doc = 
      bml::bml_( *tree, getBMLInfoMap());

    xercesc::DOMElement* root = doc->getDocumentElement();
    
    //////////////////////////////////////////////////////////////////////////////////////////////////
    bml::speech speech( "primary" );
    bml::textType speech_text;
    
    /// element names aren't preserved when we serialize xsd::bml types
    xercesc::XMLString::transcode("speech", text_buffer, 999);
    // May be doing the createElement part wrong
    xercesc::DOMElement* speech_element = doc->createElement(text_buffer);
    *speech_element << speech;
    root->appendChild( speech_element );

    xercesc::XMLString::transcode("text", text_buffer, 999);
    xercesc::DOMElement* text_element = doc->createElement(text_buffer);
    *text_element << speech_text;
    speech_element->appendChild(text_element);

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /// Test out xerces text nodes
    xercesc::XMLString::transcode("This is a test string", text_buffer, 999);
    xercesc::DOMText* str_element = doc->createTextNode(text_buffer);
    text_element->appendChild(str_element);



    /// Try out BML doc
    // std::string dom_str = serializeXMLDocument( *doc );
    // ROS_INFO_STREAM( dom_str );
    /// Convert from stupid auto_ptr to nice shared_ptr
    // std::shared_ptr<bml::bml> bml_res( bml::bml_( doc ).release() );
    // ROS_INFO_STREAM( serializeXMLDocument(*bml_res));

    std::shared_ptr<xercesc::DOMDocument> result( doc.release() );

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: Return real results//////////////////////////////////////////////////////////////////////
    // Author: Dylan Foster, Date: 2014-02-12////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
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
  
  // How to add plaintext to textType? Alternatively, how to cast 'syncType's into plaintext
  
  // bml::speech speech ( "primary" ); /// arbitrary id

  // bml::textType speech_text;
  // // speech_text.string( text );

  // // the 'textType' has a vector of 'syncType's. 
  // bml::syncType speech_sync;
  // speech_sync.id( text );
  // speech_text.sync().push_back( speech_sync );
  
  // speech.text().push_back( speech_text );
  
  // tree->speech().push_back( speech );

  // tree = addSpeech(tree, text);

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
	  std::string const & rule_phrase = rule_it->first;
	  /// Mapping from behaviors to timings
	  rules::Rule const & rule = rule_it->second;
	  

	  //////////////////////////////////////////////////////////////////////////////////////////////////
	  // Try to match phrase in the speech /////////////////////////////////////////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////////////////
	  // TODO: More sophisticated matching - need to at least be able to get point where match occurred
	  // Author: Dylan Foster, Date: 2014-01-29/////////////////////////////////////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////////////////
	  if( text.find(rule_phrase) != std::string::npos )
	    {
	      /// Iterate over all of the behaviors that this rule activates
	      for( rules::Rule::const_iterator rule_behavior_it = rule.begin(); 
		   rule_behavior_it != rule.end(); ++rule_behavior_it )
		{
		  /// TODO: Incorporate timings
		  std::string const & behavior_name = rule_behavior_it->first;
		  
		  behavior::BehaviorMap::const_iterator behavior_it = behaviors.find( behavior_name );
		  /// We should have already pruned behaviors without definitions
		  ROS_ASSERT(behavior_it != behaviors.end());
		  
		  behavior::Behavior const & behavior = behavior_it->second;
		  
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
  
  /// TODO: Add anything to the tree that's not directly related to rules
  std::shared_ptr<xercesc::DOMDocument> doc = addSpeech(tree, text);
  std::string output = serializeXMLDocument( *doc );
  
  return output;
}

}
