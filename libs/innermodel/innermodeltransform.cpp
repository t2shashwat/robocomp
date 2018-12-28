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
#include "innermodeltransform.h"

InnerModelTransform::InnerModelTransform(std::string id_, std::string engine_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float mass_, std::shared_ptr<InnerModelNode> parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	engine = engine_;
	set(rx_, ry_, rz_, tx_, ty_, tz_);
	mass = mass_;
	backtX = tx_;
	backtY = ty_;
	backtZ = tz_;
	backrX = rx_;
	backrY = ry_;
	backrZ = rz_;
//	rx = ry = rz = tx = ty = tz = NULL;
	gui_translation = gui_rotation = true;
}

InnerModelTransform::~InnerModelTransform()
{
}


void InnerModelTransform::print(bool verbose)
{
	std::cout << "Transform: " << id << std::endl;
	if (verbose)
	{
		((QMat *)this)->print(QString::fromStdString(id));
		getTr().print(QString::fromStdString(id + "_T"));
		//extractAnglesR().print(id+"_R");
	}
}

void InnerModelTransform::save(std::ofstream &out, int tabs)
{
	if (id == "root")
	{
		for (int i=0; i<tabs; i++) 
			out << "\t";
		out << "<innermodel>\n";
		for (auto c=children.begin(); c!=children.end(); c++)
			(*c)->save(out, tabs+1);
		for (int i=0; i<tabs; i++) 
			out << "\t";
		out << "</innermodel>\n";
	}
	else
	{
		for (int i=0; i<tabs; i++) out << "\t";
		if (gui_translation and not gui_rotation)
			out << "<translation id=\"" << id.c_str() << "\" tx=\""<< std::to_string(backtX).c_str() <<"\" ty=\""<< std::to_string(backtY).c_str() <<"\" tz=\""<< std::to_string(backtZ).c_str() <<"\">\n";
		else if (gui_rotation and not gui_translation)
			out << "<rotation id=\"" << id.c_str() << "\" rx=\""<< std::to_string(backrX).c_str() <<"\" ry=\""<< std::to_string(backrY).c_str() <<"\" rz=\""<< std::to_string(backrZ).c_str() <<"\">\n";
		else
			out << "<transform id=\"" << id.c_str() << "\" tx=\""<< std::to_string(backtX).c_str() <<"\" ty=\""<< std::to_string(backtY).c_str() <<"\" tz=\""<< std::to_string(backtZ).c_str() <<"\"  rx=\""<< std::to_string(backrX).c_str() <<"\" ry=\""<< std::to_string(backrY).c_str() <<"\" rz=\""<< std::to_string(backrZ).c_str() <<"\">\n";

		for (auto c=children.begin(); c!=children.end(); c++)
			(*c)->save(out, tabs+1);

		for (int i=0; i<tabs; i++) 
			out << "\t";
		if (gui_translation and not gui_rotation )
			out << "</translation>\n";
		else if (gui_rotation and not gui_translation)
			out << "</rotation>\n";
		else
			out << "</transform>\n";
	}
}

/*void InnerModelTransform::setUpdatePointers(float *tx_, float *ty_, float *tz_, float *rx_, float *ry_, float *rz_)
{
	tx = tx_;
	ty = ty_;
	tz = tz_;
	rx = rx_;
	ry = ry_;
	rz = rz_;
	fixed = false;
}

void InnerModelTransform::setUpdateTranslationPointers(float *tx_, float *ty_, float *tz_)
{
	tx = tx_;
	ty = ty_;
	tz = tz_;
	fixed = false;
}



void InnerModelTransform::setUpdateRotationPointers(float *rx_, float *ry_, float *rz_)
{
	rx = rx_;
	ry = ry_;
	rz = rz_;
	fixed = false;
}


void InnerModelTransform::update()
{
	if (!fixed)
	{
		if (tx) backtX = *tx;
		if (ty) backtY = *ty;
		if (tz) backtZ = *tz;
		if (rx) backrX = *rx;
		if (ry) backrY = *ry;
		if (rz) backrZ = *rz;
		set(backrX, backrY, backrZ, backtX, backtY, backtZ);
	}
	updateChildren();
}
*/
void InnerModelTransform::update(QVec vec)
{
	//Lock lock(mutex);
	backrX = vec(0); backrY = vec(1); backrZ = vec(2);
	backtX = vec(3); backtY = vec(4); backtZ = vec(5);
	set(backrX, backrY, backrZ, backtX, backtY, backtZ);
	fixed = true;
}

/**
 * @brief Updates the internal values of the node from the values passed in the parameters
 *
 * @param tx_ X Translation
 * @param ty_ Y Translation
 * @param tz_ Z Translation
 * @param rx_ RX Rotation
 * @param ry_ RY Rotation
 * @param rz_ RZ Rotation
 * @return void
 */
void InnerModelTransform::update(float tx_, float ty_, float tz_, float rx_, float ry_, float rz_)
{
	//Lock lock(mutex);
	backrX = rx_; backrY = ry_; backrZ = rz_;
	backtX = tx_; backtY = ty_; backtZ = tz_;
	set(backrX, backrY, backrZ, backtX, backtY, backtZ);
	fixed = true;
	if (innerModel != nullptr){
		innerModel->removeOldHashTrNode(id);
		innerModel->removeOldHashRtNode(id);
	}
}
void InnerModelTransform::updateT(float tx_, float ty_, float tz_)
{
	//Lock lock(mutex);
	backtX = tx_; backtY = ty_; backtZ = tz_;
	set(backrX, backrY, backrZ, backtX, backtY, backtZ);
	fixed = true;
	if (innerModel != nullptr)
		innerModel->removeOldHashTrNode(id);
}
	
void InnerModelTransform::updateR(float rx_, float ry_, float rz_)
{
	//Lock lock(mutex);
	backrX = rx_; backrY = ry_; backrZ = rz_;
	set(backrX, backrY, backrZ, backtX, backtY, backtZ);
	fixed = true;
	if (innerModel != nullptr)
		innerModel->removeOldHashRtNode(id);
} 

std::shared_ptr<InnerModelNode> InnerModelTransform::copyNode(std::map<std::string,std::shared_ptr<InnerModelNode>> &hash, std::shared_ptr<InnerModelNode> parent)
{
	std::shared_ptr<InnerModelTransform> ret( new InnerModelTransform(id, engine, backtX, backtY, backtZ, backrX, backrY, backrZ, mass, parent));
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash.insert(std::pair<std::string, std::shared_ptr<InnerModelNode>>(id,std::dynamic_pointer_cast<InnerModelNode>(ret)));
 
	for (auto i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return std::static_pointer_cast<InnerModelNode>(ret);
}
void InnerModelTransform::transformValues(const RTMat &Tpb, float tx, float ty, float tz, float rx, float ry, float rz)
{	
	RTMat Tbi;
	Tbi.setTr(tx,ty,tz);
	Tbi.setR (rx,ry,rz);
	RTMat Tpi = Tpb*Tbi;
	QVec angles = Tpi.extractAnglesR();
	QVec tr = Tpi.getTr();

	update(tr.x(),tr.y(),tr.z(),angles.x(),angles.y(),angles.z());
}

void InnerModelTransform::translateValues(const RTMat &Tpb, float tx, float ty, float tz)
{
	RTMat Tbi;
	Tbi.setTr(tx,ty,tz);
	RTMat Tpi = Tpb*Tbi;
	QVec tr = Tpi.getTr();
	tx = tr.x();
	ty = tr.y();
	tz = tr.z();
	updateT(tx,ty,tz);
}

void InnerModelTransform::rotateValues(const RTMat &Tpb, float rx, float ry, float rz)
{
	RTMat Tbi;
	Tbi.setR (rx,ry,rz);
	RTMat Tpi = Tpb*Tbi;
	QVec angles = Tpi.extractAnglesR();
	rx = angles.x();
	ry = angles.y();
	rz = angles.z();
	updateR(rx,ry,rz);
}
