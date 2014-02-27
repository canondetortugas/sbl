/***************************************************************************
 *  include/nvbg/parsing.h
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


#ifndef SBL_NVBG_PARSING
#define SBL_NVBG_PARSING

// ROS
#include <ros/ros.h>

#include <set>

/// boost
#include <boost/tokenizer.hpp>

namespace nvbg
{
    namespace parse
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Datatypes etc.////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    
    extern std::string const DELIMITERS;
    /// delimiters that don't count as words
    extern std::set<char> const IGNORED_DELIMITERS;
    /// delimiter denoting the end of a sentence (.)
    extern std::set<char> const SENTENCE_ENDINGS;
    
    /// Map string index to the token containing this index
    typedef std::map<size_t, size_t> IndexMap;

    /// Need some way to store indices for sentences
    struct ParsedSpeech
    {
      /// Map index of character in the original string to the index of the non-ignored word it corresponds to
      IndexMap char_to_word_;
      /// Map index of character in the original string to the index of the sentence it corresponds to
      IndexMap char_to_sentence_;
      /// Map from the index of a word to the index of the sentence
      /// containing word
      IndexMap word_to_sentence_;
      /// Map from sentence index to the index of the first word in the sentence
      // IndexMap sentence_to_word_;

      /// Non-ignored tokens, all tokens
      std::vector<std::string> tokens_, all_tokens_;
      std::string speech_, processed_speech_;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Functions//////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    parse::ParsedSpeech parseSpeech(std::string const & speech);
    std::string toLower(std::string const & input);
    bool isIgnored(std::string const & word);
    bool isSentenceEnding(std::string const & word);

    inline size_t wordToStartTime(size_t const & idx){ return 2*idx; }
    inline size_t wordToEndTime(size_t const & idx){ return 2*idx + 1; }
    
  }

} // nvbg

#endif // SBL_NVBG_PARSING
