#ifndef INNERMODEL_H
#define INNERMODEL_H

// System includes
#include <stdexcept>
#include <stdint.h>
#include <typeinfo>

// Qt includes
#include <QHash>
//#include <QMutexLocker>
#include <mutex>
#include <memory>

// RoboComp includes
#include <qmat/qmat.h>
#include <qmat/qvec.h>
#include <qmat/qcamera.h>
#include <qmat/qrtmat.h>
#include <qmat/qfundamental.h>

//Derived and auxiliary classes
#include <innermodel/innermodelconfig.h>
#include <innermodel/innermodelexception.h>
#include <innermodel/innermodeltransform.h>
#include <innermodel/innermodelnode.h>
#include <innermodel/innermodeljoint.h>
#include <innermodel/innermodeltouchsensor.h>
#include <innermodel/innermodeldifferentialrobot.h>
#include <innermodel/innermodelomnirobot.h>
#include <innermodel/innermodelprismaticjoint.h>
#include <innermodel/innermodelplane.h>
#include <innermodel/innermodelcamera.h>
#include <innermodel/innermodelrgbd.h>
#include <innermodel/innermodellaser.h>
#include <innermodel/innermodelmesh.h>
#include <innermodel/innermodelimu.h>
#include <innermodel/innermodelpointcloud.h>
#include <innermodel/innermodeltouchsensor.h>
#include <innermodel/innermodeldisplay.h>

// #include <osg/MatrixTransform>
// osg::Vec3 qmatToVec3(const RMat::QMat & m) {return osg::Vec3(m(0),m(1),m(2));}
// #include <osg/TriangleFunctor>
// #include <osg/io_utils>
// #include <osg/Geode>



#ifdef PYTHON_BINDINGS_SUPPORT
#include <boost/python/stl_iterator.hpp>
#endif

using namespace RMat;

class InnerModelReader;

class InnerModel
{
friend InnerModelReader;

public:
	static bool support_fcl();

	/////////////////////////
	/// (Con/De)structors
	/////////////////////////
	InnerModel();
	InnerModel(std::string xmlFilePath);
	InnerModel(const InnerModel &original);
	InnerModel(InnerModel &original);
	InnerModel(InnerModel *original);
	~InnerModel();

	/////////////////////////
	bool open(std::string xmlFilePath);
	bool save(std::string path);
	InnerModel* copy();
	void ChangeHash(std::string new_id, std::shared_ptr<InnerModelNode> node);

	///////////////////////
	/// Tree update methods
	///////////////////////
	void setRoot(std::shared_ptr<InnerModelNode> node);
	void update();
//TODO REMOVE
//	void setUpdateRotationPointers(QString rotationId, float *x, float *y, float *z);
//	void setUpdateTranslationPointers(QString translationId, float *x, float *y, float *z);
//	void setUpdatePlanePointers(QString planeId, float *nx, float *ny, float *nz, float *px, float *py, float *pz);
//	void setUpdateTransformPointers(std::string transformId, float *tx, float *ty, float *tz, float *rx, float *ry, float *rz);
	void cleanupTables();
/*	void updateTransformValues(std::string transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId="");
	void updateTransformValues(std::string transformId, QVec v, std::string parentId="");
	void updateTransformValuesS(std::string transformId, float tx, float ty, float tz, float rx, float ry, float rz, std::string parentId="");
	void updateTransformValuesS(std::string transformId, QVec v, std::string parentId="");
	void updateTranslationValues(std::string transformId, float tx, float ty, float tz, std::string parentId="");
	void updateRotationValues(std::string transformId, float rx, float ry, float rz,std::string parentId="");
	void updateJointValue(std::string jointId, float angle, bool force=false);
	void updatePrismaticJointPosition(std::string jointId, float position);
	void updatePlaneValues(std::string planeId, float nx, float ny, float nz, float px, float py, float pz);
	void updateDisplay(std::string displayId, std::string texture);
*/	
	////////////////////////////////
	/// Factory constructors
	///////////////////////////////
//TODO REMOVE	
/*	InnerModelTransform* newTransform(QString id, QString engine, InnerModelNode *parent, float tx=0, float ty=0, float tz=0, float rx=0, float ry=0, float rz=0, float mass=0);
	InnerModelJoint* newJoint(QString id, InnerModelTransform* parent, float lx = 0, float ly = 0, float lz = 0, float hx = 0, float hy = 0, float hz = 0,  float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, float min=-INFINITY, float max=INFINITY, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelTouchSensor* newTouchSensor(QString id, InnerModelTransform* parent, QString type, float nx = 0, float ny = 0, float nz = 0, float min=0, float max=INFINITY, uint32_t port=0);
	InnerModelPrismaticJoint* newPrismaticJoint(QString id, InnerModelTransform* parent, float min=-INFINITY, float max=INFINITY, float value=0, float offset=0, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelDifferentialRobot* newDifferentialRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0, float noise=0., bool collide=false);
	InnerModelOmniRobot* newOmniRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0, float noise=0., bool collide=false);
	InnerModelCamera* newCamera(QString id, InnerModelNode *parent, float width, float height, float focal);
	InnerModelRGBD* newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, float noise, uint32_t port = 0, QString ifconfig="");
	InnerModelIMU* newIMU(QString id, InnerModelNode *parent, uint32_t port = 0);
	InnerModelLaser* newLaser(QString id, InnerModelNode *parent, uint32_t port = 0, uint32_t min=0, uint32_t max=30000, float angle = M_PIl, uint32_t measures = 360, QString ifconfig="");
	InnerModelPlane* newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx=0, float ny=0, float nz=0, float px=0, float py=0, float pz=0, bool collidable=0);
	InnerModelDisplay* newDisplay(QString id, uint32_t port, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx, float ny, float nz, float px, float py, float pz, bool collidable);
	InnerModelMesh* newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable=0);
	InnerModelMesh* newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable=0);
	InnerModelPointCloud* newPointCloud(QString id, InnerModelNode *parent);

	InnerModelTransform *getTransform(const QString &id)                 { return getNode<InnerModelTransform>(id); }
	InnerModelJoint *getJoint(const QString &id)                         { return getNode<InnerModelJoint>(id); }
	InnerModelJoint *getJoint(const std::string &id)                     { return getNode<InnerModelJoint>(QString::fromStdString(id)); }
	InnerModelJoint *getJointS(const std::string &id)                    { return getNode<InnerModelJoint>(QString::fromStdString(id)); }
	InnerModelJoint &getJointRef(const std::string &id)                  { return *getNode<InnerModelJoint>(QString::fromStdString(id)); }
	InnerModelTouchSensor *getTouchSensor(const QString &id)             { return getNode<InnerModelTouchSensor>(id); }
	InnerModelPrismaticJoint *getPrismaticJoint(const QString &id)       { return getNode<InnerModelPrismaticJoint>(id); }
	InnerModelDifferentialRobot *getDifferentialRobot(const QString &id) { return getNode<InnerModelDifferentialRobot>(id); }
	InnerModelOmniRobot *getOmniRobot(const QString &id)                 { return getNode<InnerModelOmniRobot>(id); }
	InnerModelCamera *getCamera(QString id)                              { return getNode<InnerModelCamera>(id); }
	InnerModelRGBD *getRGBD(QString id)                                  { return getNode<InnerModelRGBD>(id); }
	InnerModelIMU *getIMU(QString id)                                    { return getNode<InnerModelIMU>(id); }
	InnerModelLaser *getLaser(QString id)                                { return getNode<InnerModelLaser>(id); }
	InnerModelPlane *getPlane(const QString &id)                         { return getNode<InnerModelPlane>(id); }
	InnerModelMesh *getMesh(const QString &id)                           { return getNode<InnerModelMesh>(id); }
	InnerModelPointCloud *getPointCloud(const QString &id)               { return getNode<InnerModelPointCloud>(id); }
*/
	///////////////////////////////////
	/// Kinematic transformation methods
	////////////////////////////////////
	QVec transform(  const std::string & destId, const QVec &origVec, const std::string & origId);
	QVec transform(  const std::string &destId, const std::string & origId) { return transform(destId, QVec::vec3(0,0,0), origId); };
//	QVec transformS( const std::string &destId, const QVec &origVec, const std::string & origId);
//	QVec transformS( const std::string &destId, const std::string &origId) { return transform(QString::fromStdString(destId), QVec::vec3(0,0,0), QString::fromStdString(origId)); }

	QVec transform6D(const std::string &destId, const QVec &origVec, const std::string & origId) { Q_ASSERT(origVec.size() == 6); return transform(destId, origVec, origId); }
	QVec transform6D(const std::string &destId, const std::string & origId) { return transform(destId, QVec::vec6(0,0,0,0,0,0), origId); }
//	QVec transformS6D(const std::string &destId, const std::string & origId) 
//		{ return transform(QString::fromStdString(destId), QVec::vec6(0,0,0,0,0,0), QString::fromStdString(origId)); }
//	QVec transformS6D(const std::string &destId, const QVec &origVec, const std::string & origId) 
//		{ return transform(QString::fromStdString(destId), origVec, QString::fromStdString(origId)); }
	
	////////////////////////////////////////////
	/// Transformation matrix retrieval methods
	///////////////////////////////////////////
	RTMat getTransformationMatrix(const std::string &destId, const std::string &origId);
//	RTMat getTransformationMatrixS(const std::string &destId, const std::string &origId);
	QMat getRotationMatrixTo(const std::string &to, const std::string &from);
	QVec getTranslationVectorTo(const std::string &to, const std::string &from);
	QVec rotationAngles(const std::string & destId, const std::string & origId);


	////////////////////////////////
	/// Factory constructor/accessor
	///////////////////////////////
	
	/// New constructor
	template<typename T, typename... Ts>
	std::shared_ptr<T> newNode(Ts&&... params)
	{
		auto t = std::make_tuple(params...);
		std::string id = std::string(std::get<0>(t));
		if(hash.find(id) != hash.end())
			throw InnerModelException("InnerModel::newNode Error: Cannot insert new node with already existing name: " + id);

		std::shared_ptr<T> node(new T(std::forward<Ts>(params)...)); 

		hash.insert(std::make_pair(id, std::static_pointer_cast<InnerModelNode>(node)));
		return node;
	}
	
	std::list<std::string> getIDKeys()
	{	
		std::list<std::string> keys;
		for (auto const& element : hash)
			keys.push_back(element.first);
		return keys; 
	}
	
	template <typename N> 
	std::shared_ptr<N> getNode(const std::string &id) 
	{
		auto n = hash.at(id);
		return std::dynamic_pointer_cast<N>(n);
	}
	
	template <typename N> 
	std::shared_ptr<N> getNode(const std::string &id) const
	{
		auto n = hash.at(id);
		return std::dynamic_pointer_cast<N>(n);
	}

	void createHash(std::shared_ptr<InnerModelNode> &node)
	{
		if(hash.find(node->id) == hash.end())
			hash.insert(std::make_pair(node->id, node));
		for (auto i : node->children)
		{
			createHash(i);
		}
	}
	
	/////////////////////////////////////////////
	/// Graph editing methods
	/////////////////////////////////////////////
	
	void removeSubTree(std::shared_ptr<InnerModelNode> node, std::list<std::string> *l);
	void removeNode(const std::string & id);
	void moveSubTree(std::shared_ptr<InnerModelNode>  nodeSrc, std::shared_ptr<InnerModelNode> nodeDst);
	void getSubTree(std::shared_ptr<InnerModelNode> node, std::list<std::string> *l);
	void getSubTree(std::shared_ptr<InnerModelNode> node, std::list<std::shared_ptr<InnerModelNode> > *l);
	void computeLevels(std::shared_ptr<InnerModelNode> node);
	std::shared_ptr<InnerModelNode> getRoot() { return root; }
	std::string getParentIdentifier(std::string id);
//	std::string getParentIdentifierS(std::string id);

	/////////////////////
	/// Set debug level
	/////////////////////
	int debugLevel(int level=-1) { static int debug_level=0; if (level>-1) debug_level=level; return debug_level; }

	////////////////////////////
	// FCL related
	////////////////////////////
	bool collidable(const std::string &a);
	bool collide(const std::string &a, const std::string &b);
	float distance(const std::string &a, const std::string &b);

#if FCL_SUPPORT==1
	bool collide(const std::string &a, const fcl::CollisionObject *obj);
#endif

	/**
		* @brief Computes the jacobian of a list of joints at a given configuration point given by motores
		*
		* @param listaJoints list of names of joints in InnerModel that conform the open kinematic chain
		* @param motores value of motro joints where the jacobian will be evaluated
		* @param endEffector name of end effector of the kin. chain. It can be an element further away than the last in listaJoint.
		* @return RMat::QMat Jacobian as MxN matrix of evaluated partial derivatives. M=joints, N=6 (pose cartesian coordinates of the endEffector) (CHECK ORDER)
	*/
	QMat jacobian(QStringList &listaJoints, const QVec &motores, const QString &endEffector);
/*	QMat jacobianS(std::vector<std::string> &listaJoints, const QVec &motores, const std::string &endEffector)
	{
		QStringList listaJointQ;//* = QStringList::fromStdList(listaJoints);
		for (auto e : listaJoints)
		{
			listaJointQ.push_back(QString::fromStdString(e));
		}
		return jacobian(listaJointQ, motores, QString::fromStdString(endEffector));
	}
*/
#ifdef PYTHON_BINDINGS_SUPPORT
	QMat jacobianSPython(const  boost::python::list &listaJointsP, const QVec &motores, const std::string &endEffector)
	{
		std::vector<std::string> listaJoint = std::vector<std::string>(boost::python::stl_input_iterator<std::string>(listaJointsP), boost::python::stl_input_iterator<std::string>( ) );
		return jacobianS(listaJoint, motores, endEffector);
	}
#endif


	///////////////////////////////////////
	/// Auxiliary methods
	//////////////////////////////////////
	void print(std::string s="") { treePrint(s, true); }
	void treePrint(std::string s="", bool verbose=false) { root->treePrint(s, verbose); }

	////////////////
	/// Laser stuff DEPRECATED
	////////////////
	QVec laserTo(const std::string &dest, const std::string & laserId , float r, float alfa)
	{
		//qDebug() << __FUNCTION__ << "DEPRECATED. Use getNode<InnerModelLaser>(laserId)->laserTo(dest,laserId, r, alfa) ";
		return getNode<InnerModelLaser>(laserId)->laserTo(dest, r, alfa);
	};

	//QMutex *mutex;
	mutable std::recursive_mutex mutex;

protected:
	std::shared_ptr<InnerModelNode> root;
	std::map<std::string, std::shared_ptr<InnerModelNode>> hash;
	std::map<std::pair<std::string, std::string>, RTMat> localHashTr;
	std::map<std::pair<std::string, std::string>, QMat> localHashRot;
	//QHash<QPair<std::string, std::string>, RTMat> localHashTr;
	//QHash<QPair<std::string, std::string>, QMat> localHashRot;

	void setLists(const std::string &origId, const std::string &destId);
	std::list<std::shared_ptr<InnerModelNode>> listA, listB;
};
#endif

	///////////////////////////////////////
	/// Camera methods
	//////////////////////////////////////
	// QVec project(QString reference, QVec origVec, QString cameraId);
	// QVec project(const QString &cameraId, const QVec &origVec);
	// QVec backProject(const QString &cameraId, const QVec &coord) ;//const;
	// void imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS);
	// QVec anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS);
	// QVec imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString to);
	// QVec projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist);
	// QVec horizonLine(QString planeId, QString cameraId, float heightOffset=0.);
	// QMat getHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	// QMat getAffineHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	// QMat getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera);
	// float getCameraFocal(const QString &cameraId ) const;
	// int getCameraWidth( QString cameraId );
	// int getCameraHeight(const QString &cameraId ) const;
	// int getCameraSize(const QString &cameraId ) const;
	// /// Stereo computations
	// void updateStereoGeometry( const QString &firstCam, const QString & secondCam );
	// QVec compute3DPointInCentral(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
	// QVec compute3DPointInRobot(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
	// QVec compute3DPointFromImageCoords(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);
	// QVec compute3DPointFromImageAngles(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);
	// /// Frustrum
	// struct TPlane { QVec n; float d; };
	// struct TFrustrum { TPlane left; TPlane top; TPlane right; TPlane down; TPlane near; TPlane far;};
	// TFrustrum frustrumLeft, frustrumThird, frustrumRight;
