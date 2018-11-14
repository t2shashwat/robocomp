/*
 * Copyright 2016 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef INNERMODELEXCEPTION_H
#define INNERMODELEXCEPTION_H

#include <stdexcept>
#include <iostream>
#include <string>
#include <exception>

namespace RoboComp
{

	class InnerModelException : public std::exception
	{
		public:
			InnerModelException(std::string &&arg_) : arg(std::move(arg_)) {};
			const char* what() const noexcept override { return arg.c_str(); };
			void print() const { std::cout << arg << std::endl;};
		private:
			std::string arg; 
	};

	class BadCastToFinalType : InnerModelException 
	{
		public:
			BadCastToFinalType(std::string &&arg_) : InnerModelException(std::move(arg_)) {};
	};

	class NonExistingNode : InnerModelException 
	{
		public:
			NonExistingNode(std::string &&arg_) : InnerModelException(std::move(arg_)){};
	};
}
#endif // INNERMODELEXCEPTION_H
