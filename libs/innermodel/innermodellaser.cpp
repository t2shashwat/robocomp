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

#include "innermodellaser.h"
#include <innermodel/innermodel.h>

InnerModelLaser::InnerModelLaser(std::string id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, std::string _ifconfig, InnerModel *innermodel_, std::shared_ptr<InnerModelNode> parent_) :  InnerModelNode(id_, parent_) , innermodel(innermodel_)
{
	
 #if FCL_SUPPORT==1
 	collisionObject = NULL;    //inherited from InnerModelNode
 #endif
	port = _port;
	min = _min;
	max = _max;
	measures = _measures;
	angle = _angle;
	ifconfig = _ifconfig;
	
	//qDebug() << __FUNCTION__ << id << port << min << max << angle << measures;
}

void InnerModelLaser::save(std::ofstream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";

	out << "<laser id=\"" << id.c_str() <<"\" port=\""<<port<<"\" min=\""<<  std::to_string(min).c_str()<<"\" max=\""<< std::to_string(max).c_str() <<"\" measures=\""<< std::to_string(measures).c_str()<<"\" angle=\""<< std::to_string(angle).c_str() <<"\" ifconfig=\""<<ifconfig.c_str()<< "\" />\n";
}

void InnerModelLaser::print(bool verbose)
{
	if (verbose) printf("LASER.");
}

void InnerModelLaser::update()
{
	if (fixed)
	{
	}
	updateChildren();
}

std::shared_ptr<InnerModelNode> InnerModelLaser::copyNode(std::map<std::string, std::shared_ptr<InnerModelNode>> &hash, std::shared_ptr<InnerModelNode> parent)
{
	std::shared_ptr<InnerModelLaser> ret(new InnerModelLaser(id, port, min, max, angle, measures, ifconfig, innermodel, parent));
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;
	ret->innerModel = parent->innerModel;

	ret->innerModel = parent->innerModel;

	for (auto iterator: children)
	{
		ret->addChild(iterator->copyNode(hash, ret));
	}

	return ret;
}

QVec InnerModelLaser::laserTo(const std::string &dest, float r, float alpha)
{
	QVec p(3);
	p(0) = r * sin(alpha);
	p(1) = 0;
	p(2) = r * cos(alpha);
	return innermodel->transform(dest, p, this->id);
}
