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

#include <nvbg/bml_generation.h>

namespace nvbg
{
  std::string const PROCESSED_SPEECH_ID = "processed";
  std::string const RAW_SPEECH_ID = "raw";

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
    bml::speech speech( PROCESSED_SPEECH_ID ), raw( RAW_SPEECH_ID );
    bml::textType speech_text, raw_text;
    
    /// element names aren't preserved when we serialize xsd::bml types
    xercesc::XMLString::transcode("speech", text_buffer, BUFFER_LEN-1);
    // May be doing the createElement part wrong
    xercesc::DOMElement* speech_element = doc->createElement(text_buffer);
    *speech_element << speech;
    root->appendChild( speech_element );

    xercesc::DOMElement* rspeech_element = doc->createElement(text_buffer);
    *rspeech_element << raw;
    root->appendChild( rspeech_element );

    xercesc::XMLString::transcode("text", text_buffer, BUFFER_LEN-1);
    xercesc::DOMElement* text_element = doc->createElement(text_buffer);
    *text_element << speech_text;
    speech_element->appendChild(text_element);

    /// populate a speech element with the raw input text
    xercesc::DOMElement* rtext_element = doc->createElement(text_buffer);
    *rtext_element << raw_text;
    rspeech_element->appendChild(rtext_element);
    
    ROS_ASSERT( ps.speech_.size() <= BUFFER_LEN -1 );
    xercesc::XMLString::transcode(ps.speech_.c_str(), text_buffer, BUFFER_LEN-1);
    xercesc::DOMText* stext_element = doc->createTextNode(text_buffer);
    rtext_element->appendChild(stext_element);

    //////////////////////////////////////////////////////////////////////////////////////////////////
    std::stringstream begin_stream, sbegin_stream, tbegin_stream;
    begin_stream << PROCESSED_SPEECH_ID << ":begin";
    sbegin_stream << PROCESSED_SPEECH_ID << ":s0:begin";
    tbegin_stream << PROCESSED_SPEECH_ID << ":t0";

    nvbg::addConstraint( doc.get(), {begin_stream.str(), sbegin_stream.str(), tbegin_stream.str()});

    /// added_word_idx tracks words for which we've added syncpoints
    size_t word_idx = 0, added_word_idx = 0, sentence_idx = 0;
    bool closed = false;
    for( std::vector<std::string>::const_iterator token_it = ps.all_tokens_.begin();
	 token_it != ps.all_tokens_.end(); ++token_it )
      {
	std::string const & token = *token_it;

	ROS_ASSERT( token.size() <= BUFFER_LEN -1);

	/// We don't give word tags to spaces etc.
	if( !parse::isIgnored(token) )
	  {
	    xercesc::XMLString::transcode(token.c_str(), text_buffer, BUFFER_LEN-1);
	    xercesc::DOMText* str_element = doc->createTextNode(text_buffer);
	    
	    /// sync point for the current word
	    std::stringstream wsb, wse;
	    wsb << "t" << parse::wordToStartTime(added_word_idx);
	    wse << "t" << parse::wordToEndTime(added_word_idx);

	    bml::syncType word_begin_sync, word_end_sync;
	    word_begin_sync.id( wsb.str() );
	    word_end_sync.id( wse.str() );
	    
	    xercesc::XMLString::transcode("sync", text_buffer, BUFFER_LEN-1);
	    xercesc::DOMElement* wsb_element = doc->createElement(text_buffer);
	    xercesc::DOMElement* wse_element = doc->createElement(text_buffer);
	    *wsb_element << word_begin_sync;
	    *wse_element << word_end_sync;

	    /// word begin syncpoint
	    text_element->appendChild(wsb_element);
	    /// word text
	    text_element->appendChild(str_element);
	    /// word end syncpoint
	    text_element->appendChild(wse_element);

	    ++added_word_idx;
	  }

	/// Add sentence tags for the current sentence
	if(parse::isSentenceEnding(token))
	  {
	    std::stringstream sstream, tstream;
	    sstream << PROCESSED_SPEECH_ID <<":" << "s" << sentence_idx << ":end";
	    tstream << PROCESSED_SPEECH_ID << ":" << "t" 
		    << parse::wordToEndTime(added_word_idx -1); /// the previous word
	    ++sentence_idx;

	    std::vector<std::string> end_refs = {sstream.str(), tstream.str()};

	    /// Don't start a new sentence if this is the last token
	    if( token_it + 1 == ps.all_tokens_.end() )
	      {
		closed = true;
		
		/// Add a sync reference for the end of the entire speech
		std::stringstream end_stream;
		end_stream << PROCESSED_SPEECH_ID << ":end";
		end_refs.push_back(end_stream.str() );
	      }
	    else
	      {
		/// A new sentence is coming, so we make a constraint block for the beginning of it
		std::stringstream sstream, tstream;
		sstream << PROCESSED_SPEECH_ID <<":" << "s" << sentence_idx << ":begin";
		tstream << PROCESSED_SPEECH_ID << ":" << "t" 
			<< parse::wordToStartTime(added_word_idx); /// the next word

		nvbg::addConstraint(doc.get(), {sstream.str(), tstream.str() });

	      }

	    nvbg::addConstraint(doc.get(), end_refs);
	  }
	
	++word_idx;
      }
    
    /// Add end tag for final sentence if we haven't already
    if( !closed )
      {
	/// final sentence sync
	// bml::syncRefType sf_end_sync, end_sync, tend_sync;
	std::stringstream sf_end_stream, end_stream, tend_stream;
	tend_stream << PROCESSED_SPEECH_ID << ":" << "t" << parse::wordToEndTime(added_word_idx-1);
	sf_end_stream << PROCESSED_SPEECH_ID << ":" << "s" << sentence_idx << ":end";
	end_stream << PROCESSED_SPEECH_ID << ":end";

	nvbg::addConstraint(doc.get(), {sf_end_stream.str(), end_stream.str(), tend_stream.str()});
      }

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
  void generateBML(std::string & processed_bml, std::string & raw_bml, 
		   std::string const & text, std::string const & eca,
			  nvbg::behavior::BehaviorMap const &behaviors,
			  nvbg::rules::RuleClassMap const &rule_classes,
			  std::string request_id)
  {
    using namespace nvbg;
    
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

    std::vector<std::shared_ptr<ConstrainedGesture> > constrained_gestures;
    std::vector<std::shared_ptr<ConstrainedFace> > constrained_faces;
    std::vector<std::shared_ptr<ConstrainedHead> > constrained_heads;
  
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

		    size_t start_idx, end_idx;
		  
		    for( std::vector<timing::Timing>::const_iterator timing_it = timings.begin(); 
			 timing_it != timings.end(); ++timing_it )
		      {
			timing::Timing const & timing = *timing_it;

			size_t word_idx;
		      
			/// Can only have one syncpoint of each type
			if( sync.find( timing.sync ) != sync.end() )
			  {
			    ROS_WARN_STREAM("Duplicated syncpoint of type " << brk(timing.sync)
					    << " in rule " << brk(rule_phrase) << ", behavior "
					    << brk(behavior_name) );
			    continue;
			  }
		      
			if ( timing.type == "speech" )
			  {
			    std::stringstream ts;
			    /// pos is either 'start' or 'end'
			    ts << PROCESSED_SPEECH_ID << ":" << timing.pos;
			    if( timing.offset )
			      ts << " + " << timing.offset;
			  
			    sync.insert (std::make_pair( timing.sync, ts.str() ) );

			    if( timing.pos == "start")
			      word_idx = parse::wordToStartTime(0);
			    else if( timing.pos =="end")
			      word_idx = parse::wordToEndTime(ps.char_to_word_.at( rule_phrase.size()-1));
			  }
			else if( timing.type == "sentence" )
			  {
			    std::stringstream ts;
			  
			    /// Find which sentence our phrase is in
			    parse::IndexMap::iterator index_it = ps.char_to_sentence_.find(phrase_idx);
			    /// this map should have an entry for every character in the speech
			    /// so this shouldn't happen ever
			    ROS_ASSERT( index_it != ps.char_to_sentence_.end() );
			    size_t sentence_idx = index_it->second;
			  
			    ts << PROCESSED_SPEECH_ID << ":s" << sentence_idx 
			       << "_" << timing.pos;
			    if( timing.offset )
			      ts << " + " << timing.offset;
			  
			    sync.insert (std::make_pair( timing.sync, ts.str() ) );

			    if( timing.pos == "start" )
			      {
				parse::IndexMap::iterator start_it = ps.sentence_to_start_word_.find(sentence_idx);
				ROS_ASSERT( start_it != ps.sentence_to_start_word_.end() );
				word_idx = parse::wordToStartTime(start_it->second);
			      }
			    else if( timing.pos == "end" )
			      {
				parse::IndexMap::iterator end_it = ps.sentence_to_end_word_.find(sentence_idx);
				ROS_ASSERT( end_it != ps.sentence_to_end_word_.end() );
				word_idx = parse::wordToEndTime(end_it->second);
			      }
			    
			  }
			/// TODO: Add begin/end support
			else if( timing.type == "word" )
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

			    size_t ref_word_idx = word_it->second + timing.word_idx;
			    
			    if( ref_word_idx >= ps.tokens_.size() )
			      {
				ROS_ERROR_STREAM("Invalid word offset for syncpoint" << brk(timing.sync)
						 << " in rule " << brk(rule_phrase) << ", behavior "
						 << brk(behavior_name) );
				continue;
			      }

			    size_t final_idx;
			    if( timing.pos == "start")
			      final_idx = parse::wordToStartTime(ref_word_idx);
			    else if( timing.pos == "end")
			      final_idx = parse::wordToEndTime(ref_word_idx);
			    else
			      ROS_ASSERT_MSG(false, "Invalid timing pos type");

			    word_idx = final_idx;
			  
			    std::stringstream ts;
			    ts << PROCESSED_SPEECH_ID << ":t" << final_idx;
			    if( timing.offset )
			      ts << " + " << timing.offset;

			    sync.insert( std::make_pair( timing.sync, ts.str() ) );
			  }
			else if(timing.type == "phrase" )
			  {
			    if(timing.pos == "start")
			      {
				parse::IndexMap::iterator word_it = ps.char_to_word_.find(phrase_idx);
				size_t ref_word_idx = parse::wordToStartTime(word_it->second);
				std::stringstream ts;
				ts << PROCESSED_SPEECH_ID << ":t" << ref_word_idx;
				if( timing.offset )
				  ts << " + " << timing.offset;
			      
				sync.insert( std::make_pair( timing.sync, ts.str() ) );
				
				word_idx =ref_word_idx;
			      }
			    else if(timing.pos == "end")
			      {
				/// Get the last character in the phrase, then get the index of the word containing it.
				parse::IndexMap::iterator word_it = ps.char_to_word_.find(phrase_idx + rule_phrase.size()-1);
				size_t ref_word_idx = parse::wordToEndTime(word_it->second);
				std::stringstream ts;
				ts << PROCESSED_SPEECH_ID << ":t" << ref_word_idx;
				if( timing.offset )
				  ts << " + " << timing.offset;
			      
				sync.insert( std::make_pair( timing.sync, ts.str() ) );
				word_idx =ref_word_idx;
			      }
			    else
			      {
				ROS_ASSERT_MSG(false, "Invalid timing pos type");
			      }
			  }
			else
			  {
			    /// This should have already been validated
			    ROS_ASSERT_MSG(false, "Invalid type argument");
			  }

			if( timing.sync == "start")
			  start_idx = word_idx;
			else if( timing.sync == "end")
			  end_idx = word_idx;
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

			std::shared_ptr<ConstrainedGesture> cg = std::make_shared<ConstrainedGesture>();

			cg->priority = rule_class.priority;
			cg->mode = behavior.mode;
			cg->id = id.str();
			cg->start = start_idx;
			cg->end = end_idx;
			constrained_gestures.push_back(cg);
			
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

			std::shared_ptr<ConstrainedFace> cf = std::make_shared<ConstrainedFace>();

			cf->priority = rule_class.priority;
			cf->id = id.str();
			cf->start = start_idx;
			cf->end = end_idx;
			constrained_faces.push_back(cf);

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

			std::shared_ptr<ConstrainedHead> ch = std::make_shared<ConstrainedHead>();

			ch->priority = rule_class.priority;
			ch->id = id.str();
			ch->start = start_idx;
			ch->end = end_idx;
			constrained_heads.push_back(ch);

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
    
    raw_bml = serializeXMLDocument( *doc );

    nvbg::resolveConstraints(doc, constrained_gestures);
    nvbg::resolveConstraints(doc, constrained_faces);
    nvbg::resolveConstraints(doc, constrained_heads);

    processed_bml = serializeXMLDocument( *doc );
  }

  void addConstraint( xercesc::DOMDocument* doc, std::vector<std::string> const & refs)
  {
    using namespace xercesc;

    size_t const BUFFER_LEN = 1000;

    /// All xerces APIs use this c-strings 16-bit XMLCh type, so we use this buffer to convert all
    /// noncompatible strings that we want to pass to the API. XMLCh text_buffer[BUFFER_LEN];
    XMLCh text_buffer[BUFFER_LEN];

    DOMElement* root = doc->getDocumentElement();

    XMLString::transcode("constraint", text_buffer, BUFFER_LEN-1);
    DOMElement* constraint_element = doc->createElement(text_buffer);
    root->appendChild(constraint_element);

    XMLString::transcode("synchronize", text_buffer, BUFFER_LEN-1);
    DOMElement* synchronize_element = doc->createElement(text_buffer);    
    constraint_element->appendChild(synchronize_element);

    XMLString::transcode("sync", text_buffer, BUFFER_LEN-1);
    for( std::string const & ref: refs )
      {
	bml::syncRefType sr;
	sr.ref( ref );
	DOMElement* ref_element = doc->createElement(text_buffer);
	*ref_element << sr;
	synchronize_element->appendChild( ref_element );
      }
    
    return;
  }

}
