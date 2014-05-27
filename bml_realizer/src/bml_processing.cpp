/***************************************************************************
 *  src/bml_processing.cpp
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


#include <bml_realizer/bml_processing.h>



namespace realizer
{
  bool validSpeechId(std::string const & id)
  {
    return id == "processed" || id == "raw";
  }

  std::string stripSpaces(std::string const & input)
  {
    std::stringstream ss;
    for(size_t idx = 0; idx < input.size(); ++idx)
      {
	char const & c = input[idx];

	if( !(c == ' ' || c == '\n' || c == '\r' || c == '\t') )
	  ss << c;
      }
    return ss.str();
  }

  std::shared_ptr<xercesc::DOMDocument> parseBML(std::string const & doc_string)
  {
    size_t const BUFFER_LEN = 2500;
    char * text_buffer;
    
    std::shared_ptr<xercesc::XercesDOMParser> parser = std::make_shared<xercesc::XercesDOMParser>();
    std::shared_ptr<xercesc::ErrorHandler> error_handler( (xercesc::ErrorHandler*) new xercesc::HandlerBase);
    
    parser->setValidationScheme(xercesc::XercesDOMParser::Val_Auto);
    parser->setDoNamespaces(false);
    parser->setDoSchema(false);
    parser->setValidationSchemaFullChecking(false);
    parser->setCreateEntityReferenceNodes(false);

    parser->setErrorHandler( error_handler.get() );

    xercesc::MemBufInputSource input( (const unsigned char *)doc_string.c_str(), doc_string.size(), "BML Document (in memory)");
    
    bool errorsOccured = false;
    try
      {
	parser->parse( input );
      }
    catch (const xercesc::OutOfMemoryException&)
      {
        // XERCES_STD_QUALIFIER cerr << "OutOfMemoryException" << XERCES_STD_QUALIFIER endl;
	ROS_ERROR_STREAM("Parse error: Out of memory");
	errorsOccured = true;
      }
    catch (const xercesc::XMLException& e)
      {
	text_buffer = xercesc::XMLString::transcode(e.getMessage());
	  
	ROS_ERROR_STREAM( "An error occurred during parsing\n   Message: "
			  << text_buffer);

	xercesc::XMLString::release(&text_buffer);
	errorsOccured = true;
      }
    
    catch (const xercesc::DOMException& e)
      {
	XMLCh errText[BUFFER_LEN + 1];
	ROS_ERROR_STREAM("DOM Error during parsing.\n"
			 << "DOMException code is:  " << e.code );
      
	if (xercesc::DOMImplementation::loadDOMExceptionMsg(e.code, errText, BUFFER_LEN))
	  {	
	    text_buffer = xercesc::XMLString::transcode(errText);
	    ROS_ERROR_STREAM("Message is: " << text_buffer );
	    xercesc::XMLString::release(&text_buffer);
	  }

	errorsOccured = true;
      }
    catch (const xercesc::SAXException& e)
      {
	text_buffer = xercesc::XMLString::transcode(e.getMessage());
	ROS_ERROR_STREAM("SAX exception: " << text_buffer );

	xercesc::XMLString::release(&text_buffer);

	errorsOccured = true;
      }
    catch (...)
      {
    	ROS_ERROR_STREAM( "An error occurred during parsing" );
    	errorsOccured = true;
      }

    if(!errorsOccured)
      {
	ROS_INFO("Parsed BML document successfully.");
	return std::shared_ptr<xercesc::DOMDocument> ( parser->adoptDocument() );
      }
    else
      return std::shared_ptr<xercesc::DOMDocument> ();
  }

  bool extractSpeech( std::shared_ptr<xercesc::DOMDocument> const & doc, Speech & speech )
  {
    speech = Speech();
    
    bool raw_found = false, processed_found = false;
    
    size_t const BUFFER_LEN = 1000;
    XMLCh text_buffer[BUFFER_LEN+1];

    xercesc::DOMElement* root = doc->getDocumentElement();
    
    xercesc::XMLString::transcode( "speech", text_buffer, BUFFER_LEN);
    xercesc::DOMNodeList* nodes = root->getElementsByTagName(text_buffer);

    for(size_t node_idx = 0; node_idx < nodes->getLength(); ++node_idx)
      {
	xercesc::DOMNode* node = nodes->item(node_idx);
	
	if( node->getNodeType() != xercesc::DOMNode::ELEMENT_NODE)
	  continue;

	
	xercesc::DOMElement* element = dynamic_cast<xercesc::DOMElement*>( node );
	
	/// We are going to inspect the id of speech block 
	xercesc::XMLString::transcode( "id", text_buffer, BUFFER_LEN);

	/// Convert from stupid XMLCh to c++ string
	XMLCh const * xml_id = element->getAttribute(text_buffer);
	char * id = xercesc::XMLString::transcode(xml_id);
	std::string id_str(id);
	xercesc::XMLString::release(&id);

	if( !validSpeechId( id_str ) )
	  {
	    ROS_WARN_STREAM("Encountered speech element with invalid ID. Ignoring... " << brk(id_str) );
	    continue;
	  }

	else if ( id_str == "processed" )
	  {
	    if( processed_found )
	      {
		ROS_ERROR("Duplicate processed speech element found");
		return true;
	      }
	    else
	      {
		processed_found = true;
	      }
	  }
	else if ( id_str == "raw" )
	  {
	    if( raw_found )
	      {
		ROS_ERROR("Duplicate raw speech element found");
		return true;
	      }
	    else
	      {
		raw_found = true;
	      }
	  }

	/// Get child text elements (BML text type, not XML text, although BML text has XML text children)
	xercesc::XMLString::transcode( "text", text_buffer, BUFFER_LEN);
	xercesc::DOMNodeList* text_nodes = element->getElementsByTagName(text_buffer);

	if( text_nodes->getLength() != 1)
	  {
	    ROS_WARN_STREAM("Speech elements may only contain a single text block. Ignoring... " << brk(id_str));
	    continue;
	  }

	xercesc::DOMNode* text_node = text_nodes->item(0);
	if( node->getNodeType() != xercesc::DOMNode::ELEMENT_NODE)
	  {
	    ROS_WARN_STREAM("Invalid speech element content " << brk(id_str) );
	    continue;
	  }

	if( id_str == "processed" )
	  {

	    ////////////////////////////////////////////////////////////
	    // TODO: Change to a real method of filtering whitespace.
	    // Either include a raw text string in all BML documents
	    // or turn on parser validation and use a schema that
	    // specifies ignorable whitespace (and use parser.setIncludeIgnorableWhitespace)
	    // Author: Dylan Foster, Date: 2014-05-26////////////////////
	    ////////////////////////////////////////////////////////////

	    bool take_next = false;

	    /// Extract raw text by iterating over children of BML text element and skipping non-XML-text elements
	    for(xercesc::DOMNode* text_child = text_node->getFirstChild(); 
		text_child != NULL; text_child = text_child->getNextSibling() )
	      {
		if( text_child->getNodeType() == xercesc::DOMNode::TEXT_NODE)
		  {
		    xercesc::DOMText* text_element = dynamic_cast<xercesc::DOMText*>( text_child );
	    
		    XMLCh const * text = text_element->getData();
		    char * text_cstr = xercesc::XMLString::transcode(text);
		    std::string text_str( text_cstr );
		    xercesc::XMLString::release(&text_cstr);

		    std::string stripped = stripSpaces(text_str);

		    if( stripped.size() )
		      speech.words_.push_back(stripped);
		  }
	      }


	  }
	else if (id_str == "raw" )
	  {
	    std::stringstream text_ss;
	    /// Extract raw text by iterating over children of BML text element and skipping non-XML-text elements
	    for(xercesc::DOMNode* text_child = text_node->getFirstChild(); 
		text_child != NULL; text_child = text_child->getNextSibling() )
	      {
		if( text_child->getNodeType() != xercesc::DOMNode::TEXT_NODE)
		  continue;

		xercesc::DOMText* text_element = dynamic_cast<xercesc::DOMText*>( text_child );
	    
		XMLCh const * text = text_element->getData();
		char * text_cstr = xercesc::XMLString::transcode(text);
		std::string text_str( text_cstr );

		text_ss << text_str;

		xercesc::XMLString::release(&text_cstr);
	    
	      }

	    speech.raw_text_ = text_ss.str();
	  }
	else
	  {
	    ROS_ASSERT(false);
	  }

	// std::stringstream text_ss;

	// ////////////////////////////////////////////////////////////
	// // TODO: Change to a real method of filtering whitespace.
	// // Either include a raw text string in all BML documents
	// // or turn on parser validation and use a schema that
	// // specifies ignorable whitespace (and use parser.setIncludeIgnorableWhitespace)
	// // Author: Dylan Foster, Date: 2014-05-26////////////////////
	// ////////////////////////////////////////////////////////////
	
	// /// Extract raw text by iterating over children of BML text element and skipping non-XML-text elements
	// for(xercesc::DOMNode* text_child = text_node->getFirstChild(); 
	//     text_child != NULL; text_child = text_child->getNextSibling() )
	//   {
	//     if( text_child->getNodeType() != xercesc::DOMNode::TEXT_NODE)
	//       continue;
	    
	//     xercesc::DOMText* text_element = dynamic_cast<xercesc::DOMText*>( text_child );
	    
	//     XMLCh const * text = text_element->getData();
	//     char * text_cstr = xercesc::XMLString::transcode(text);
	//     std::string text_str( text_cstr );
	    
	//     /// Space between xml tags seems to be included as text nodes - we filter these out
	//     /// they are length 7 due to one newline and 6 spaces
	//     if( text_str.size() < 7 )
	//       continue;

	//     text_ss << text_str.substr(0, text_str.size() -7);
	//     xercesc::XMLString::release(&text_cstr);
	    
	//   }
	// std::pair<std::map<std::string, std::string>::iterator, bool> result = named_speeches.insert( std::make_pair(id_str, text_ss.str() ) );
	
	// if( !result.second )
	//   ROS_WARN_STREAM("Unable to add speech " << brk(id_str) << ". Duplicated ID not allowed.");
	
	// ROS_INFO_STREAM("Loaded speech " << brk(id_str) << " with content " << brk(text_ss.str() ) );
      }

    if( !(raw_found && processed_found) )
      {
	ROS_ERROR("Couldn't find all required speech blocks");
	return true;
      }
    
    return false;
  }

} /// realizer
 
