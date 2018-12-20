/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2013  pbustos <email>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include <innermodeldraw.h>

void InnerModelDraw::addMesh_ignoreExisting(InnerModelViewer *innerViewer, std::string item, std::string base, QVec t, QVec r, std::string path, QVec scale)
{
	std::shared_ptr<InnerModelTransform> parent = innerViewer->innerModel->getNode<InnerModelTransform>(base);
	//InnerModel *im = innerViewer->innerModel;
	InnerModel *im = innerViewer->innerModel.get();
	
	if (im->getNode<InnerModelNode>(item) != NULL)
	{
		removeNode(innerViewer, item);
	}

	std::shared_ptr<InnerModelMesh> mesh = im->newNode<InnerModelMesh> (
		item,
		path,
		scale(0), scale(1), scale(2),
		InnerModelMesh::NormalRendering,
		t(0), t(1), t(2),
		r(0), r(1), r(2), false,
		std::dynamic_pointer_cast<InnerModelNode>(parent));
	mesh->setScale(scale(0), scale(1), scale(2));
	parent->addChild(mesh);

	innerViewer->recursiveConstructor(mesh.get(), innerViewer->mts[QString::fromStdString(parent->id)], innerViewer->mts, innerViewer->meshHash);
}

bool InnerModelDraw::setScale(InnerModelViewer *innerViewer, const std::string item, float scaleX, float scaleY, float scaleZ)
{
	//InnerModel *im = innerViewer->innerModel;
	InnerModel *im = innerViewer->innerModel.get();
	std::shared_ptr<InnerModelMesh> aux = im->getNode<InnerModelMesh>(item);
	aux->setScale(scaleX, scaleY, scaleZ);
	return true;
}

bool InnerModelDraw::addJoint(InnerModelViewer* innerViewer, const std::string item, const std::string base, QVec t, QVec r, std::string axis)
{
	if (axis == "")
	{
		axis = "z";
	}
	//InnerModel *im = innerViewer->innerModel;
	InnerModel *im = innerViewer->innerModel.get();
	std::shared_ptr<InnerModelTransform> parent= im->getNode<InnerModelTransform>(base);
	std::shared_ptr<InnerModelJoint> jN = im->newNode<InnerModelJoint>(item,
					0,0,0,
					0,0,0,
					t(0), t(1), t(2),
					r(0), r(1), r(2),
					-1000, 1000,
				    0,
				    axis, 0, parent );
	parent->addChild (jN);
	innerViewer->recursiveConstructor(jN.get(), innerViewer->mts[QString::fromStdString(parent->id)], innerViewer->mts, innerViewer->meshHash);
	return true;
}

bool InnerModelDraw::setPlaneTexture(InnerModelViewer *innerViewer, const std::string item, std::string texture)
{
	//InnerModel *im = innerViewer->innerModel;
	InnerModel *im = innerViewer->innerModel.get();
	std::shared_ptr<InnerModelPlane> aux = im->getNode<InnerModelPlane>(item);

	aux->texture = texture;
	bool constantColor = false;
	if (texture.size() == 7)
	{
		if (texture[0] == '#')
		{
			constantColor = true;
		}
	}
	if (not constantColor)
	{
	  osg::Image *image=NULL;
		image = osgDB::readImageFile(texture);
		if (not image)
		{
			throw "Couldn't load texture.";
		}
		innerViewer->planesHash[QString::fromStdString(aux->id)]->image =image;
		innerViewer->planesHash[QString::fromStdString(aux->id)]->texture->setImage(image);
	}
	else
	{
		innerViewer->planesHash[QString::fromStdString(aux->id)]->planeDrawable->setColor(htmlStringToOsgVec4(QString::fromStdString(texture)));
	}
	return true;
}

bool InnerModelDraw::addTransform_ignoreExisting(InnerModelViewer *innerViewer, std::string item, std::string base /*, parametros aqui*/)
{
	if (innerViewer->innerModel->getNode<InnerModelNode>(base) == NULL)
	{
		throw std::string("parent doesn't exist");
	}

	if (innerViewer->innerModel->getNode<InnerModelNode>(item) != NULL)
	{
		removeNode(innerViewer, item);
	}
	addTransform(innerViewer, item, base);
	return true;
}

bool InnerModelDraw::addTransform(InnerModelViewer *innerViewer, std::string item, std::string base)
{
	std::shared_ptr<InnerModelNode> parent = innerViewer->innerModel->getNode<InnerModelNode>(base);
	if (parent == NULL)
		return false;

	std::shared_ptr<InnerModelNode> node = innerViewer->innerModel->getNode<InnerModelNode>(item);
	if (node != NULL)
	{
		printf("%s already exists!\n", item.c_str());
		return false;
	}

	std::shared_ptr<InnerModelTransform> tr;
	try
	{
		tr = innerViewer->innerModel->newNode<InnerModelTransform>(item, "static", 0,0,0, 0,0,0, 0, parent);
		parent->addChild(tr);
		innerViewer->recursiveConstructor(tr.get(), innerViewer->mts[QString::fromStdString(parent->id)], innerViewer->mts, innerViewer->meshHash);

		return true;
	}
	catch (std::string err)
	{
		printf("%s:%s:%d: Exception: %s\n", __FILE__, __FUNCTION__, __LINE__, err.c_str());
		throw;
	}
}

bool InnerModelDraw::addPlane_ignoreExisting(InnerModelViewer *innerViewer, const std::string &item, const std::string &base, const QVec &p, const QVec &n, const std::string &texture, const QVec &size)
{
	if (innerViewer->innerModel->getNode<InnerModelNode>(item))
	{
		removeNode(innerViewer, item);
	}
	addPlane_notExisting(innerViewer, item, base, p, n, texture, size);

	return true;
}

bool InnerModelDraw::addPlane_notExisting(InnerModelViewer *innerViewer, const std::string &item, const std::string &base, const QVec &p, const QVec &n, const std::string &texture, const QVec &size)
{
	std::shared_ptr<InnerModelNode> parent = innerViewer->innerModel->getNode<InnerModelNode>(base);
	if (parent == NULL)
	{
		//printf("%s: parent does not exist\n", __FUNCTION__);
		return false;
	}
	std::shared_ptr<InnerModelPlane> plane = innerViewer->innerModel->newNode<InnerModelPlane>(item, texture, size(0), size(1), size(2), 1, n(0), n(1), n(2), p(0), p(1), p(2), false, parent);
	parent->addChild(plane);
	innerViewer->recursiveConstructor(plane.get(), innerViewer->mts[QString::fromStdString(parent->id)], innerViewer->mts, innerViewer->meshHash);
	return true;
}

void InnerModelDraw::drawLine(InnerModelViewer *innerViewer, std::string name, std::string parent, const QVec& normalVector, const QVec &center, float length, float width, std::string texture)
{
	InnerModelDraw::addPlane_ignoreExisting(innerViewer, name, parent, center, normalVector, texture, QVec::vec3(length, width, width));
}

void InnerModelDraw::drawLine2Points(InnerModelViewer *innerViewer, std::string name, std::string parent, const QVec& p1, const QVec& p2, float width, std::string texture)
{
	QLine2D line( p1 , p2 );	
	float dl = (p1-p2).norm2();
	QVec center = p2 + ((p1 - p2)*(float)0.5);
	InnerModelDraw::drawLine(innerViewer, name, parent, line.getNormalForOSGLineDraw(), center, dl, width, "#0000ff");
}

/**
 * @brief Removes the object name from InnerModelViewer instance
 * 
 * @param innerViewer ...
 * @param name ...
 * @return void
 */
bool InnerModelDraw::removeObject(InnerModelViewer *innerViewer, std::string name)
{
	if (innerViewer->innerModel->getNode<InnerModelNode>(name))
	{
		removeNode(innerViewer, name);
		return true;
	}
	else
	{
		std::cout << __FUNCTION__ << "Object " << name << "does not exist. Could not be removed" << std::endl;
		return false;
	}
}

bool InnerModelDraw::removeNode(InnerModelViewer *innerViewer, const std::string &item)
{
	if (item=="root")
	{
		std::cout << "Can't remove root elements" << item << std::endl;
		return false;
	}

	std::shared_ptr<InnerModelNode> node = innerViewer->innerModel->getNode<InnerModelNode>(item);
	if (node == NULL)
	{
		std::cout << "Can't remove not existing elements" << item << std::endl;
		return false;
	}

	std::list<std::string> l;
	innerViewer->innerModel->getSubTree(node, &l);
	innerViewer->innerModel->removeSubTree(node, &l);

	/// Replicate InnerModel node removals in the InnerModelViewer tree. And in handlers Lists
	foreach(std::string stdn, l)
	{
		QString n = QString::fromStdString(stdn);
		/// Replicate mesh removals
		if (innerViewer->meshHash.contains(n))
		{
			while (innerViewer->meshHash[n].osgmeshPaths->getNumParents() > 0)
				innerViewer->meshHash[n].osgmeshPaths->getParent(0)->removeChild(innerViewer->meshHash[n].osgmeshPaths);
			while(innerViewer->meshHash[n].osgmeshes->getNumParents() > 0)
				innerViewer->meshHash[n].osgmeshes->getParent(0)->removeChild(innerViewer->meshHash[n].osgmeshes);
			while(innerViewer->meshHash[n].meshMts->getNumParents() > 0)
				innerViewer->meshHash[n].meshMts->getParent(0)->removeChild(innerViewer->meshHash[n].meshMts);
			innerViewer->meshHash.remove(n);
		}
		/// Replicate transform removals
		if (innerViewer->mts.contains(n))
		{
 			while (innerViewer->mts[n]->getNumParents() > 0)
				innerViewer->mts[n]->getParent(0)->removeChild(innerViewer->mts[n]);
 			innerViewer->mts.remove(n);
		}
		/// Replicate plane removals
		if (innerViewer->planeMts.contains(n))
		{
			while(innerViewer->planeMts[n]->getNumParents() > 0)
				((osg::Group *)(innerViewer->planeMts[n]->getParent(0)))->removeChild(innerViewer->planeMts[n]);
			innerViewer->planeMts.remove(n);
			innerViewer->planesHash.remove(n);
		}
	}
	return true;
}

