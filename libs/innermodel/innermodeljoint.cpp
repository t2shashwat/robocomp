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

#include "innermodel.h"
#include "innermodeljoint.h"

class InnerModel;

InnerModelJoint::InnerModelJoint() : InnerModelTransform("invalid","static", 0,0,0, 0,0,0, 0, NULL)
{
	throw std::string("Can't actually build InnerModelJoint using the default constructor");
}

InnerModelJoint::InnerModelJoint(std::string id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_, float max_, uint32_t port_, std::string axis_, float home_, std::shared_ptr<InnerModelTransform> parent_) : InnerModelTransform(id_,"static" ,tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
{
	
	#if FCL_SUPPORT==1
	collisionObject = NULL;
	#endif

	min = min_;
	max = max_;
	home = home_;
	port = port_;
	axis = axis_;
	if (axis == "x")
	{
		backl = lx_;
		backh = hx_;
		update(min, 0, 0, max, 0, 0);
	}
	else if (axis == "y")
	{
		backl = ly_;
		backh = hy_;
		update(0, min, 0, 0, max, 0);
	}
	else if (axis == "z")
	{
		backl = lz_;
		backh = hz_;
		update(0, 0, min, 0, 0, max);
	}
	else
	{
		throw std::string("internal error, no such axis " + axis);
	}
}

void InnerModelJoint::print(bool verbose)
{
	std::cout <<"Joint: " << id <<std::endl;
	if (verbose)
	{
		((QMat *)this)->print(QString::fromStdString(id));
		getTr().print(QString::fromStdString(id+"_T"));
		//extractAnglesR().print(id+"_R");
	}
}

void InnerModelJoint::save(std::ofstream &out, int tabs)
{
	QList<InnerModelNode*>::iterator c;
	for (int i=0; i<tabs; i++)
		out << "\t";

	out << "<joint id=\"" << id.c_str() << "\" port=\"" << port << "\" axis=\"" << axis.c_str() ;

	out << "\" home=\"" << std::to_string(home).c_str();
	out << "\" min=\"" << std::to_string(min).c_str() << "\" max=\"" << std::to_string(max).c_str();
	out << "\" tx=\"" << std::to_string(backtX).c_str() << "\" ty=\"" << std::to_string(backtY).c_str() <<"\" tz=\""<< std::to_string(backtZ).c_str();
	out << "\" rx=\"" << std::to_string(backrX).c_str() << "\" ry=\"" << std::to_string(backrY).c_str() << "\" rz=\"" << std::to_string(backrZ).c_str();

	out << "\">\n";

	for (auto c : children)
			c->save(out, tabs+1);

	for (int i=0; i<tabs; i++) out << "\t";
	out << "</joint>\n";
}


void InnerModelJoint::update(float lx_, float ly_, float lz_, float hx_, float hy_, float hz_)
{
	if (axis == "x")
	{
		backh = hx_;
		backl = lx_;
	}
	else if (axis == "y")
	{
		backh = hy_;
		backl = ly_;
	}
	else if (axis == "z")
	{
		backh = hz_;
		backl = lz_;
	}
	fixed = true;
}

float InnerModelJoint::getAngle()
{
	if (axis == "x")
	{
		return backrX;
	}
	else if (axis == "y")
	{
		return backrY;
	}
	else
	{
		return backrZ;
	}
}

float InnerModelJoint::setAngle(float angle, bool force)
{
	float ret = angle;
	if (angle > max)
	{
		ret = max;
	}
	else if (angle < min)
	{
		ret = min;
	}

	if (axis == "x")
	{
		backrX = ret;
		set(ret,0,0, 0,0,0);
	}
	else if (axis == "y")
	{
		backrY = ret;
		set(0,ret,0, 0,0,0);
	}
	else if (axis == "z")
	{
		backrZ = ret;
		set(0,0,ret, 0,0,0);
	}
	else
	{
		throw std::string("internal error, no such axis " + axis);
	}

	std::cout << innerModel << (long int)innerModel << std::endl;
	if (innerModel != nullptr)
		innerModel->cleanupTables();
	return ret;
}

QVec InnerModelJoint::unitaryAxis()
{
	if( axis == "x") return QVec::vec3(1,0,0);
	if( axis == "y") return QVec::vec3(0,1,0);
	if( axis == "z") return QVec::vec3(0,0,1);
	return QVec::zeros(3);
}

std::shared_ptr<InnerModelNode> InnerModelJoint::copyNode(std::unordered_map<std::string, std::shared_ptr<InnerModelNode>> &hash, std::shared_ptr<InnerModelNode> parent)
{
	std::shared_ptr<InnerModelJoint> ret;
	if (axis == "x")
	{
		std::shared_ptr<InnerModelJoint> ret(new InnerModelJoint(id, backl,0,0, backh,0,0, backtX, backtY, backtZ, backrX, 0, 0, min, max, port, axis, home, std::static_pointer_cast<InnerModelTransform>(parent)));
	}
	else if (axis == "y")
	{
		std::shared_ptr<InnerModelJoint> ret(new InnerModelJoint(id, 0,backl,0, 0,backh,0, backtX, backtY, backtZ, 0, backrY, 0, min, max, port, axis, home, std::static_pointer_cast<InnerModelTransform>(parent)));
	}
	else if (axis == "z")
	{
		std::shared_ptr<InnerModelJoint> ret(new InnerModelJoint(id, 0,0,backl, 0,0,backh, backtX, backtY, backtZ, 0, 0, backrZ, min, max, port, axis, home, std::static_pointer_cast<InnerModelTransform>(parent)));
	}
	else
	{
		std::cout << stderr << "InnerModel internal error: invalid axis " << axis << std::endl;
		exit(-1);
	}

	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;
	ret->innerModel = parent->innerModel;
	for (auto iterator: children)
	{
		ret->addChild(iterator->copyNode(hash, ret));
	}
	ret->setAngle(getAngle());

	return ret;
}
