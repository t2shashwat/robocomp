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

#include "innermodelmesh.h"


InnerModelMesh::InnerModelMesh(std::string id_, std::string meshPath_, float scale, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable,  std::shared_ptr<InnerModelNode> parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	InnerModelMesh(id_,meshPath_,scale,scale,scale,render_,tx_,ty_,tz_,rx_,ry_,rz_, collidable, parent_);
}

InnerModelMesh::InnerModelMesh(std::string id_, std::string meshPath_, float scalex_, float scaley_, float scalez_, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable_, std::shared_ptr<InnerModelNode> parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	id = id_;
	render = render_;
	meshPath = meshPath_;
	scalex = scalex_;
	scaley = scaley_;
	scalez = scalez_;
	tx = tx_;
	ty = ty_;
	tz = tz_;
	rx = rx_;
	ry = ry_;
	rz = rz_;
	collidable = collidable_;

#if FCL_SUPPORT==1
	// Get to the OSG geode
	//osg::Node *osgnode_ = osgDB::readNodeFile(meshPath.toStdString());
	osg::ref_ptr<osg::Node> osgnode_ = osgDB::readNodeFile(meshPath); 
	if (not osgnode_) printf("Could not open: '%s'.\n", meshPath.c_str());
	if (osgnode_ != NULL)
	{
		// Instanciate the vector of vertices and triangles (that's what we are looking for)
		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;
		CalculateTriangles calcTriangles(&vertices, &triangles);
		osgnode_->accept(calcTriangles);

// 		printf("id: %s\n", id.toStdString().c_str());
// 		printf("scale: %f %f %f\n", scalex, scaley, scalez);
// 		printf("points: %zu\n", vertices.size());
// 		printf("triangles: %zu\n", triangles.size());

		// Get the internal transformation matrix of the mesh
		RTMat rtm(rx, ry, rz, tx, ty, tz);
		// Transform each of the read vertices
		for (size_t i=0; i<vertices.size(); i++)
		{
			fcl::Vec3f v = vertices[i];
			const QMat v2 = (rtm * QVec::vec3(v[0]*scalex, v[1]*scaley, -v[2]*scalez).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
			vertices[i] = fcl::Vec3f(v2(0), v2(1), v2(2));
		}

// ////
// ////   UNCOMMENT THIS CODE TO GENERATE A POINTCLOUD OF THE POINTS IN THE MESHES
// ////
// std::ofstream outputFile;
// outputFile.open((id.toStdString()+".pcd").c_str());
// outputFile << "# .PCD v.7 - Point Cloud Data file format\n";
// outputFile << "VERSION .7\n";
// outputFile << "FIELDS x y z \n";
// outputFile << "SIZE 4 4 4\n";
// outputFile << "TYPE F F F\n";
// outputFile << "COUNT 1 1 1\n";
// outputFile << "WIDTH " << vertices.size() << "\n";
// outputFile << "HEIGHT 1\n";
// outputFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
// outputFile << "POINTS " << vertices.size() << "\n";
// outputFile << "DATA ascii\n";
// for (size_t i=0; i<vertices.size(); i++)
// {
// 	outputFile << vertices[i][0]/1000. << " " << vertices[i][1]/1000. << " " << vertices[i][2]/1000. << "\n";
// }
// outputFile.close();


		// Associate the read vertices and triangles vectors to the FCL collision model object
		fclMesh = FCLModelPtr(new FCLModel());
		fclMesh->beginModel();
		fclMesh->addSubModel(vertices, triangles);
		fclMesh->endModel();
		collisionObject = new fcl::CollisionObject(fclMesh);
		
	}
	else
	{
		throw std::string("Failed to read mesh " + meshPath +" for collision support!\n");
	}
#endif
}

void InnerModelMesh::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<mesh id=\""<<id.c_str()<<"\"" <<" file=\"" << meshPath.c_str()
	<< "\" scale=\"" << std::to_string(scalex).c_str() << ","<< std::to_string(scaley).c_str()<< ","<< std::to_string(scalez) .c_str()
	<< "\" tx=\"" << std::to_string(tx).c_str() << "\" ty=\"" << std::to_string(ty).c_str() << "\" tz=\"" << std::to_string(tz).c_str()
	<< "\" rx=\"" << std::to_string(rx).c_str() << "\" ry=\"" << std::to_string(ry).c_str() << "\" rz=\"" << std::to_string(rz).c_str() 
	<<"\" collide=\""<< std::to_string(collidable).c_str()<< "\" />\n";
}

void InnerModelMesh::print(bool verbose)
{
	if (verbose) 
		std::cout<< "Mesh: "<<id <<std::endl;
}


void InnerModelMesh::setScale(float x, float y, float z)
{
	scalex=x;
	scaley=y;
	scalez=z;
}

bool InnerModelMesh::normalRendering() const
{
	return render == NormalRendering;
}

bool InnerModelMesh::wireframeRendering() const {
	return render == WireframeRendering;
}

std::shared_ptr<InnerModelNode> InnerModelMesh::copyNode(std::map<std::string, std::shared_ptr<InnerModelNode>> &hash, std::shared_ptr<InnerModelNode> parent)
{
	std::shared_ptr<InnerModelMesh> ret(new InnerModelMesh(id, meshPath, scalex, scaley, scalez, render, tx, ty, tz, rx, ry, rz, collidable, parent));
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;
	ret->innerModel = parent->innerModel;

#if FCL_SUPPORT==1
	// Associate the read vertices and triangles vectors to the FCL collision model object
	ret->fclMesh = FCLModelPtr(new FCLModel(*fclMesh.get()));
	ret->collisionObject = new fcl::CollisionObject(ret->fclMesh);
#endif


	for (auto iterator: children)
	{
		ret->addChild(iterator->copyNode(hash, ret));
	}

	return ret;
}

