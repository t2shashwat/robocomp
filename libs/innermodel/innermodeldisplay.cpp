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

#include "innermodel/innermodeldisplay.h"

InnerModelDisplay::InnerModelDisplay(std::string id_, uint32_t port_, std::string texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, bool collidable_, std::shared_ptr<InnerModelNode>parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	if ( abs(nx_)<0.001 and abs(ny_)<0.001 and abs(nz_)<0.001 ) nz_ = -1;
	normal = QVec::vec3(nx_, ny_, nz_);
	point = QVec::vec3(px_, py_, pz_);
	nx = ny = nz = px = py = pz = NULL;
	texture = texture_;
	width = width_;
	height = height_;
	depth = depth_;
	repeat = repeat_;
	collidable = collidable_;
	port = port_;

	id = id_;

#if FCL_SUPPORT==1
	std::vector<fcl::Vec3f> vertices;
	vertices.push_back(fcl::Vec3f(-width/2., +height/2., -depth/2.)); // Front NW
	vertices.push_back(fcl::Vec3f(+width/2., +height/2., -depth/2.)); // Front NE
	vertices.push_back(fcl::Vec3f(-width/2., -height/2., -depth/2.)); // Front SW
	vertices.push_back(fcl::Vec3f(+width/2., -height/2., -depth/2.)); // Front SE
	vertices.push_back(fcl::Vec3f(-width/2., +height/2., +depth/2.)); // Back NW
	vertices.push_back(fcl::Vec3f(+width/2., +height/2., +depth/2.)); // Back NE
	vertices.push_back(fcl::Vec3f(-width/2., -height/2., +depth/2.)); // Back SW
	vertices.push_back(fcl::Vec3f(+width/2., -height/2., +depth/2.)); // Back SE

	osg::Matrix r;
	r.makeRotate(osg::Vec3(0, 0, 1), osg::Vec3(normal(0), normal(1), -normal(2)));
	QMat qmatmat(4,4);
	for (int rro=0; rro<4; rro++)
	{
		for (int cco=0; cco<4; cco++)
		{
			qmatmat(rro,cco) = r(rro,cco);
		}
	}

	for (size_t i=0; i<vertices.size(); i++)
	{
		fcl::Vec3f v = vertices[i];
		const QVec rotated = (qmatmat*(QVec::vec3(v[0], v[1], v[2]).toHomogeneousCoordinates())).fromHomogeneousCoordinates();
		vertices[i] = fcl::Vec3f(rotated(0)+px_, rotated(1)+py_, rotated(2)+pz_);
	}

	std::vector<fcl::Triangle> triangles;
	triangles.push_back(fcl::Triangle(0,1,2)); // Front
	triangles.push_back(fcl::Triangle(1,2,3));
	triangles.push_back(fcl::Triangle(4,5,6)); // Back
	triangles.push_back(fcl::Triangle(5,6,7));
	triangles.push_back(fcl::Triangle(4,0,6)); // Left
	triangles.push_back(fcl::Triangle(0,6,2));
	triangles.push_back(fcl::Triangle(5,1,7)); // Right
	triangles.push_back(fcl::Triangle(1,7,3));
	triangles.push_back(fcl::Triangle(5,1,4)); // Top
	triangles.push_back(fcl::Triangle(1,4,0));
	triangles.push_back(fcl::Triangle(2,3,6)); // Bottom
	triangles.push_back(fcl::Triangle(3,6,7));

	fclMesh = FCLModelPtr(new FCLModel());
	fclMesh->beginModel();
	fclMesh->addSubModel(vertices, triangles);
	fclMesh->endModel();
	collisionObject = new fcl::CollisionObject(fclMesh);

#endif
}

void InnerModelDisplay::updateTexture(std::string texture_)
{
  texture = texture_;
}

void InnerModelDisplay::print(bool verbose)
{
	if (verbose) normal.print(QString::fromStdString("Display: " + id));
}


void InnerModelDisplay::save(std::ofstream &out, int tabs)
{
	for (int i=0; i<tabs; i++) 
		out << "\t";
	out << "<display id=\"" << id.c_str() << "\" texture=\"" << texture.c_str() << "\" size=\"" << std::to_string(width).c_str()<<","<<std::to_string(height).c_str()<<","	<<std::to_string(depth).c_str() << "\" repeat=\"" << std::to_string(repeat).c_str() << "\" nx=\"" << std::to_string(normal(0)).c_str()	<< "\" ny=\"" << std::to_string(normal(1)).c_str() << "\" nz=\"" << std::to_string(normal(2)).c_str() << "\" px=\"" << std::to_string(point(0)).c_str() << "\" py=\"" << std::to_string(point(1)).c_str() << "\" pz=\"" << std::to_string(point(2)).c_str() <<"\" collide=\""<< std::to_string(collidable).c_str() << "\"" << " port=\"" << port <<"\" />\n";
}

void InnerModelDisplay::setUpdatePointers(float *nx_, float *ny_, float *nz_, float *px_, float *py_, float *pz_)
{
	nx = nx_;
	ny = ny_;
	nz = nz_;
	px = px_;
	py = py_;
	pz = pz_;
	nx = ny = nz = px = py = pz = NULL;
	fixed = false;
}

void InnerModelDisplay::update(float nx_, float ny_, float nz_, float px_, float py_, float pz_)
{
	normal(0) = nx_;
	normal(1) = ny_;
	normal(2) = nz_;
	point(0) = px_;
	point(1) = py_;
	point(2) = pz_;
	fixed = true;
}

std::shared_ptr<InnerModelNode> InnerModelDisplay::copyNode(std::unordered_map<std::string, std::shared_ptr<InnerModelNode>> &hash, std::shared_ptr<InnerModelNode>parent)
{
	std::shared_ptr<InnerModelDisplay> ret(new InnerModelDisplay(id, port, texture, width, height, depth, repeat, normal(0), normal(1), normal(2), point(0), point(1), point(2), collidable, parent));
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

	return ret;
}
