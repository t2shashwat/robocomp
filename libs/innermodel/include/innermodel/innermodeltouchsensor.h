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

#ifndef INNERMODELTOUCHSENSOR_H
#define INNERMODELTOUCHSENSOR_H

#include <innermodel/innermodelnode.h>

class InnerModelTouchSensor :public InnerModelNode
{
	public:
		InnerModelTouchSensor(std::string id_, std::string stype, float nx_, float ny_, float nz_, float min_=-INFINITY, float max_=INFINITY, uint32_t port_=0, std::shared_ptr<InnerModelNode> parent_=nullptr);
		void print(bool verbose) {verbose = true;}
		void save(QTextStream &out, int tabs){}
		QVec getMeasure() { return value; }
		virtual std::shared_ptr<InnerModelNode> copyNode(std::map<std::string, std::shared_ptr<InnerModelNode>> &hash, std::shared_ptr<InnerModelNode> parent);

		float nx, ny, nz;
		float min, max;
		float value;
		std::string stype;
		uint32_t port;
};
#endif // INNERMODELTOUCHSENSOR_H
