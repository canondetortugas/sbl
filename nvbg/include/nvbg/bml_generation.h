/***************************************************************************
 *  include/nvbg/bml_generation.h
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


#ifndef SBL_NVBG_BMLGENERATION
#define SBL_NVBG_BMLGENERATION

// ROS
#include <ros/ros.h>

/// BML parsing
#include <bml_cpp/bml-1.0.h>

#include <nvbg/types.h>

namespace nvbg
{
  int initializeXMLPlatform();
  xml_schema::namespace_infomap getBMLInfoMap();
  std::shared_ptr<xercesc::DOMDocument> addSpeech(std::shared_ptr<bml::bml> tree, std::string const & text);

  std::string generateBML(std::string const & text, std::string const & eca,
					nvbg::behavior::BehaviorMap const &behaviors,
					nvbg::rules::RuleClassMap const &rule_classes,
					std::string request_id = "request");

  std::string serializeXMLDocument( xercesc::DOMDocument & doc );
  std::string serializeXMLDocument( bml::bml & tree );

  parse::ParsedSpeech parseSpeech(std::string const & speech);

  namespace parse
  {
    extern std::string const DELIMITERS;
    
    /// Map string index to the token containing this index
    typedef std::map<size_t, size_t> IndexTokenMap;

    /// Need some way to store indices for sentences
    struct ParsedSpeech
    {
      IndexTokenMap word_indices_;
      IndexTokenMap sentence_indices_;
      /// Map from the index of a word to the index of the sentence
      /// starting with the word, if such a sentence exists.
      IndexTokenMap word_to_sentence_;
      std::vector<std::string> tokens_;
    };
    
  }
  
}

#endif // SBL_NVBG_BMLGENERATION
