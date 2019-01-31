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

#ifndef INNERMODELNODE_H
#define INNERMODELNODE_H

// RoboComp includes
#include <qmat/QMatAll>
#include <innermodel/innermodelconfig.h>
#include <memory>
#include <fstream>

#if FCL_SUPPORT==1
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/ccd/motion.h>
#include <fcl/BV/BV.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/traversal/traversal_node_bvh_shape.h>
#include <fcl/traversal/traversal_node_bvhs.h>
typedef fcl::BVHModel<fcl::OBBRSS> FCLModel;
typedef std::shared_ptr<FCLModel> FCLModelPtr;
#endif

class InnerModel;

class InnerModelNode : public RTMat
{
		friend class InnerModelCamera;
		friend class InnerModelRGBD;
		friend class InnerModelReader;

	public:
		InnerModelNode(std::string id_, std::shared_ptr<InnerModelNode> parent_=nullptr);
		virtual ~InnerModelNode();
	
		struct AttributeType
		{
			std::string type;
			std::string value;
		};
		
		void treePrint(std::string s, bool verbose=false);
		virtual void print(bool verbose) = 0;
//		virtual void update() = 0;
		virtual std::shared_ptr<InnerModelNode> copyNode(std::unordered_map<std::string, std::shared_ptr<InnerModelNode>> &hash, std::shared_ptr<InnerModelNode> parent) = 0;
		virtual void save(std::ofstream &out, int tabs) = 0;
		void setParent(std::shared_ptr<InnerModelNode>parent_);
		void addChild(std::shared_ptr<InnerModelNode> child);
		void setFixed(bool f=true);
		bool isFixed();
		void updateChildren();
		
		//protected:
		std::string id;
		int level;
		bool fixed;
		InnerModel *innerModel;
		std::shared_ptr<InnerModelNode> parent;
		std::list<std::shared_ptr<InnerModelNode>> children;
		std::map<std::string, AttributeType> attributes;

		// FCLModel
		bool collidable;
		#if FCL_SUPPORT==1
			FCLModelPtr fclMesh;
			fcl::CollisionObject *collisionObject;
		#endif
};

#endif // INNERMODELNODE_H
