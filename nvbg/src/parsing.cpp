/***************************************************************************
 *  src/parsing.cpp
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


#include <nvbg/parsing.h>

/// boost
#include <boost/tokenizer.hpp>


namespace nvbg
{
  namespace parse
  {
    std::string const DELIMITERS = " ,;.:\"!?";
    std::set<char> const IGNORED_DELIMITERS = {' ', '\"'};
    std::string const SENTENCE_END = ".";

    /// TODO: Kill leading / trailing whitespace if it exists
    /// TODO: Don't tolwer the original sentence - just save a tolower'd version for strcmp
    parse::ParsedSpeech parseSpeech(std::string const & speech)
    {
      typedef boost::tokenizer<boost::char_separator<char> > _Tokenizer;
      
      // std::string input = toLower(speech);
      std::string input = speech;

      /// Tokenize input using DELIMITERS. These arguments make it so that
      /// all of the delimiters are kept as tokens.
      boost::char_separator<char> sep( "", DELIMITERS.c_str());
      _Tokenizer tokenizer(input, sep);

      ParsedSpeech ps;

      // std::stringstream ss;

      size_t atoken_idx = 0, sentence_idx = 0, char_idx = 0, token_idx = 0;
      // bool first_word = true;
      for( _Tokenizer::iterator token_it = tokenizer.begin(); token_it != tokenizer.end(); ++token_it)
	{
	  std::string const & token = *token_it;
	  // ss << token;
	  ps.all_tokens_.push_back(token);

	  ps.word_to_sentence_.insert( std::make_pair( atoken_idx, sentence_idx ) );
	  
	  for( size_t idx = char_idx; idx < char_idx + token.size(); ++idx )
	    {
	      ps.char_to_word_.insert( std::make_pair( idx, token_idx ));
	      ps.char_to_sentence_.insert( std::make_pair( idx, sentence_idx ));
	    }
	  char_idx += token.size();
	  
	  // if (first_word)
	  //   {
	  //     ps.sentence_to_word_.insert( std::make_pair( sentence_idx, atoken_idx ) );
	  //     first_word = false;
	  //   }

	  if (!isIgnored( token ))
	    {
	      ps.tokens_.push_back(token);
	      ++token_idx;
	    }
	  
	  if( token == SENTENCE_END )
	    {
	      // first_word = true;
	      ++sentence_idx;
	    }
	  
	  ++atoken_idx;
	}
      ROS_ASSERT(char_idx == input.size());
      
      ps.processed_speech_ = toLower(speech);
      ps.speech_ = speech;
      
      return ps;
    }

    std::string toLower(std::string const & input)
    {
      std::string output = input;
      std::transform( output.begin(), output.end(), 
		      output.begin(), ::tolower );
      return output;
    }

    bool isIgnored(std::string const & word)
    {
      return word.size() == 1 && parse::IGNORED_DELIMITERS.count(word.c_str()[0]);
    }

  }
}
