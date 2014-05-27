/***************************************************************************
 *  include/bml_realizer/bml_processing.h
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


#ifndef SBL_BMLREALIZER_BMLPROCESSING
#define SBL_BMLREALIZER_BMLPROCESSING

// ROS
#include <ros/ros.h>

#include <uscauv_common/macros.h>

#include <nvbg/bml_generation.h>

#include <boost/algorithm/string.hpp>

/// xerces nonsense
#include <xercesc/util/PlatformUtils.hpp>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMWriter.hpp>

#include <xercesc/framework/MemBufInputSource.hpp>

#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLUni.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>

namespace realizer
{
  extern size_t const BUFFER_LEN; 

  struct Speech
  {
    std::string raw_text_;
    std::vector<std::string> words_;
  };

  /// XML platform must be initialized before this is run
  std::shared_ptr<xercesc::DOMDocument> parseBML(std::string const & doc_string);
  
  /** 
   * 
   * @param doc BML Document
   * @param speech Speech content
   * 
   * @return Zero if successful, non-zero otherwise
   */
  bool extractSpeech( std::shared_ptr<xercesc::DOMDocument> const & doc, Speech & speech );

  /** 
   * 
   * @param doc BML document
   * @param string 
   * @param syncref_to_time Map from sync reference to time index
   * 
   * @return Zero if successful, non-zero otherwise
   */
  bool extractConstraints( std::shared_ptr<xercesc::DOMDocument> const & doc, 
			   std::map<std::string, std::string> & syncref_to_time );

  bool validSpeechId(std::string const & id);

  std::string stripSpaces(std::string const & input);

  xercesc::DOMNodeList* getNodes(xercesc::DOMElement* doc, std::string const & type);
  
}


#endif // SBL_BMLREALIZER_BMLPROCESSING
