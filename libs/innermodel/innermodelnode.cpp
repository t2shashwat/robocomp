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

#include "innermodelnode.h"

InnerModelNode::InnerModelNode(std::string id_, std::shared_ptr<InnerModelNode> parent_) : RTMat()
{
	collidable = false;
	#if FCL_SUPPORT==1
		collisionObject = NULL; 
	#endif

	fixed = true;
	parent = parent_;
	if (parent)
		level = parent->level+1;
	else
		level = 0;
	id = id_;
	attributes.clear();
}

InnerModelNode::~InnerModelNode()
{
	#if FCL_SUPPORT==1
		if (collisionObject!=NULL)
		{
			delete collisionObject;
		}
		fclMesh.reset();
	#endif
}

void InnerModelNode::treePrint(std::string s, bool verbose)
{
 	std::cout << s << id << "l(" << level << ") [" << children.size() <<"]\n";
	for (auto i=children.begin(); i!=children.end(); i++)
	{
		if (verbose)
			(*i)->print(verbose);
		(*i)->treePrint(s + "  ", verbose);

	}
}

void InnerModelNode::setParent(std::shared_ptr<InnerModelNode> parent_)
{
	parent = parent_;
	level = parent->level + 1;
}

void InnerModelNode::addChild(std::shared_ptr<InnerModelNode> child)
{
	child->innerModel = innerModel;
	
	if (std::find(children.begin(), children.end(),child) == children.end())
	{
		children.push_back(child);
	}
	child->parent.reset(this);  //MIRAR
}

void InnerModelNode::setFixed(bool f)
{
	fixed = f;
}

bool InnerModelNode::isFixed()
{
	return fixed;
}

void InnerModelNode::updateChildren()
{
	std::cout << "NOT IMPLEMENTED CHECK!";
//	foreach(auto child, children)
//		child->update();
}
