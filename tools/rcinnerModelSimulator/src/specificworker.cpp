/*
 *    Caopyright (C) 2006-2014 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

// Simulator includes
#include "specificworker.h"

// Qt includes
#include <QDropEvent>
#include <QEvent>
#include <QGLWidget>
#include <QLabel>
#include <QMouseEvent>
#include <QMutexLocker>
#include <QTime>
#include <QWidget>

// OSG includes
#include <osg/io_utils>
#include <osg/BoundingBox>
#include <osg/LineWidth>
#include <osg/Matrixd>
#include <osg/PolygonMode>
#include <osg/TriangleFunctor>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osgText/Font>
#include <osgText/Text>
#include <osgUtil/IntersectVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

//#include <boost/algorithm/string.hpp>

// #define INNERMODELMANAGERDEBUG

/**
 *\brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& _mprx, Ice::CommunicatorPtr _communicator, const char *_innerModelXML, int ms) : GenericWorker(_mprx)
{
	//viewerMutex = new QMutex(QMutex::Recursive);
	
	worker = this;
	communicator = _communicator;

	laserDataCartArray_mutex = new QMutex(QMutex::Recursive);
	laserDataCartArray.clear();

	// Initialize InnerModel stuff
	innerModel = std::make_shared<InnerModel>(_innerModelXML);
	
	qDebug() << __FILE__ << __FUNCTION__ << "InnerModel read";
	
	//add name of .xml
	setWindowTitle(windowTitle() + "\t" + _innerModelXML);

	// Initialize the Inner Model Viewer
	QGLFormat fmt;
	fmt.setDoubleBuffer(true);
	QGLFormat::setDefaultFormat(fmt);
	viewer = new OsgView(frameOSG);
	imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup());
	qDebug() << __FILE__ << __FUNCTION__ << "HOLa";

	manipulator = new osgGA::TrackballManipulator;
	qDebug() << __FILE__ << __FUNCTION__ << "HOLa";

    //manipulator->setHomePosition(osg::Vec3d(0, 10000, 0), osg::Vec3d(0, 0, 0), osg::Vec3d(0, 0, -10000), true);
    //viewer->setCameraManipulator(manipulator, true);
    this->viewer->setHomePosition(osg::Vec3d(0, 10000, 0),osg::Vec3(0.f,0.,-40.),up, false);
	
	// Add mouse pick handler to publish 3D coordinates
	if (rcis_mousepicker_proxy)
	{
		viewer->addEventHandler(new PickHandler(rcis_mousepicker_proxy));
	}

	// Restore previous camera position
//	settings = new QSettings("RoboComp", "RCIS");
//	QString path(_innerModelXML);
//	if (path == settings->value("path").toString() )
//	{
//		//restore matrix view
//		QStringList l = settings->value("matrix").toStringList();
//		if (l.size() > 0)
//		{
//			osg::Matrixd m;
//			for (int i=0; i<4; i++ )
//			{
//				for (int j=0; j<4; j++ )
//				{
//					m(i,j)=l.takeFirst().toDouble();
//				}
//			}
//			manipulator->setByMatrix(m);
//		}
//		else
//		{
//			setTopPOV();
//		}
//	}
//	else
//	{
//		settings->setValue("path",path);
// 	}
	qDebug() << __FILE__ << __FUNCTION__ << "InnerModelViewer created";

	// Connect all the signals
	connect(topView,   SIGNAL(clicked()), this, SLOT(setTopPOV()));
	connect(leftView,  SIGNAL(clicked()), this, SLOT(setLeftPOV()));
	connect(rightView, SIGNAL(clicked()), this, SLOT(setRightPOV()));
	connect(frontView, SIGNAL(clicked()), this, SLOT(setFrontPOV()));
	connect(backView,  SIGNAL(clicked()), this, SLOT(setBackPOV()));
	connect(sp_lightx,  SIGNAL(valueChanged(double)), this, SLOT(setLigthx(double)));
	connect(sp_lighty,  SIGNAL(valueChanged(double)), this, SLOT(setLigthy(double)));
	connect(sp_lightz,  SIGNAL(valueChanged(double)), this, SLOT(setLigthz(double)));
	connect(actionObject, SIGNAL(triggered()), this, SLOT(objectTriggered()));
	connect(actionVisual, SIGNAL(triggered()), this, SLOT(visualTriggered()));
    connect(actionSave, SIGNAL(triggered()), this, SLOT(saveScene()));
    connect(actionFeatures, SIGNAL(triggered()), this, SLOT(featuresTriggered()));
    connect(actionFloor_Texture, SIGNAL(triggered()), this, SLOT(change_textureTriggered()));
    //connect(actionTree, SIGNAL(triggered()), this, SLOT(viewTriggered()));
    connect(addobject_button, SIGNAL(clicked()), this, SLOT(add_object()));
    connect(treepushButton, SIGNAL(clicked()), this, SLOT(add_tree()));
    connect(comboBox_texture,SIGNAL(currentIndexChanged(int)),this,SLOT(floor_texture()));
    //connect(texture,SIGNAL(currentIndexChanged(int)),this,SLOT(object_texture()));
    connect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this,SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
    fillNodeMap(innerModel->getNode("root"), NULL);
    comboBox_texture->addItem("--Choose Texture--");
    comboBox_texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg"),"Metal");
    comboBox_texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg"),"Checkerboard");
    comboBox_texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/blue.jpg"),"Blue");
    comboBox_texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/grid.png"),"Grid");
    comboBox_texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/índice.jpeg"),"Indice");
    comboBox_texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/klein_blue3.jpg"),"Klein Blue");
    comboBox_texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/r.jpeg"),"Raw");
    comboBox_texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/wood.jpg"),"Wood");
    texture->addItem("--Choose Texture--");
    texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg"),"Metal");
    texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg"),"Checkerboard");
    texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/blue.jpg"),"Blue");
    texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/grid.png"),"Grid");
    texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/índice.jpeg"),"Indice");
    texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/klein_blue3.jpg"),"Klein Blue");
    texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/r.jpeg"),"Raw");
    texture->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/wood.jpg"),"Wood");

    /*texture_2->addItem("--Choose Texture--");
    texture_2->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg"),"Metal");
    texture_2->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg"),"Checkerboard");
    texture_2->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/blue.jpg"),"Blue");
    texture_2->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/grid.png"),"Grid");
    texture_2->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/índice.jpeg"),"Indice");
    texture_2->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/klein_blue3.jpg"),"Klein Blue");
    texture_2->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/r.jpeg"),"Raw");
    texture_2->addItem(QIcon("/home/robocomp/robocomp/files/osgModels/textures/wood.jpg"),"Wood");*/
    // Additional widgets
	objectTriggered();
	visualTriggered();
    featuresTriggered();
    //viewTriggered();
	viewer->realize();
	//viewer->setThreadingModel( osgViewer::ViewerBase::ThreadPerCamera);
	
	// Initialize the timer
	setPeriod(ms);	
	qDebug() << __FILE__ << __FUNCTION__ << "Timer period:" << ms;
    groupBox_8->hide();
    groupBox_12->hide();
    treeWidget->hide();
    flag=1;
    trans_gb_2->hide();
    rot_gb_2->hide();
    //mass_gb->show();

    plane_gb_box->hide();
    plane_gb_cyl->hide();
    plane_gb_cone->hide();
    plane_gb_sphere->hide();

    //disconnect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
    //treeWidget->clear;

}


void SpecificWorker::compute()
{
	// Compute the elapsed time interval since the last update
	static QTime lastTime = QTime::currentTime();
	
	QTime currentTime = QTime::currentTime();
	const int elapsed = lastTime.msecsTo (currentTime);
	// 	printf("elapsed %d\n", elapsed);
	lastTime = currentTime;

	guard gl(innerModel->mutex);
	
		updateCameras();
		updateLasers();
		updateJoints(float(elapsed)/1000.0f);
		updateTouchSensors();

		#ifdef INNERMODELMANAGERDEBUG
			printf("Elapsed time: %d\n", elapsed);
		#endif
		
		// Shutdown empty servers
		for (int i=0; i<jointServersToShutDown.size(); i++)
			jointServersToShutDown[i]->shutdown();
		jointServersToShutDown.clear();
	
		// Resize world widget if necessary, and render the world
		if (viewer->size() != frameOSG->size())
			viewer->setFixedSize(frameOSG->width(), frameOSG->height());
        //printf("compute inside imv update");
		imv->update();
		//osg render
		viewer->frame();
			
}

///////////////////////////////////////////////////////////////////////
///Add Save Option
/// ////////////////////////////////////////////////////////////////
void SpecificWorker::saveScene()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                tr("Save XML"), "",
                tr("XML file (*.xml)"));
        if (fileName.isEmpty())
            return;
        else {
            innerModel->save(fileName);
        }
}
///////////////////////////////////////////////////////////////////
//edit object properties
///////////////////////////////////////////////////////////////////
void SpecificWorker::currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous)
{
         interfaceConnections(false);
         currentNode = nodeMapByItem[current];
         printf("on click in tree");
         showAvailableGroups();
         //highlightNode();
         interfaceConnections(true);
}
void SpecificWorker::interfaceConnections(bool enable)
{
    if (enable)
    {


        connect(texture_2, SIGNAL(editingFinished()), this, SLOT(planeChanged()));
        connect(texture_2_s, SIGNAL(editingFinished()), this, SLOT(planeChanged_s()));
        connect(texture_2_cyl, SIGNAL(editingFinished()), this, SLOT(planeChanged_cyl()));
        connect(texture_2_cone, SIGNAL(editingFinished()), this, SLOT(planeChanged_cone()));

        connect(ptx_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        connect(pty_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        connect(ptz_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        connect(normx_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        connect(normx_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        connect(normx_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        connect(ptx_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        connect(pty_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        connect(ptz_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        connect(normx_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        connect(normx_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        connect(normx_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));

        connect(ptx_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        connect(pty_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        connect(ptz_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        connect(normx_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        connect(normx_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        connect(normx_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));

        connect(ptx_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        connect(pty_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        connect(ptz_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        connect(normx_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        connect(normx_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        connect(normx_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));


        connect(rect_w_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        connect(rect_h_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        connect(rect_dep_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        // Rotation-related
        connect(rx_2, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
        connect(ry_2, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
        connect(rz_2, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
        // Translation-related
        connect(tx_2, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
        connect(ty_2, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
        connect(tz_2, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
        //cyclinder-related
        connect(cyl_h_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        connect(cyl_rad_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        //sphere
        connect(radiusval_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        //cone
        connect(cone_h_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        connect(cone_rad_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        // Joint-related
        //connect(jointAngle, SIGNAL(valueChanged(double)), this, SLOT(jointChanged()));
    }
    else
    {
        // Plane-related
        disconnect(texture_2, SIGNAL(editingFinished()), this, SLOT(planeChanged()));
        disconnect(texture_2_s, SIGNAL(editingFinished()), this, SLOT(planeChanged_s()));
        disconnect(texture_2_cyl, SIGNAL(editingFinished()), this, SLOT(planeChanged_cyl()));
        disconnect(texture_2_cone, SIGNAL(editingFinished()), this, SLOT(planeChanged_cone()));

        disconnect(ptx_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        disconnect(pty_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        disconnect(ptz_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        disconnect(normx_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        disconnect(normx_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        disconnect(normx_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));

        disconnect(ptx_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        disconnect(pty_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        disconnect(ptz_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        disconnect(normx_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        disconnect(normx_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        disconnect(normx_3, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));

        disconnect(ptx_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        disconnect(pty_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        disconnect(ptz_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        disconnect(normx_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        disconnect(normx_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        disconnect(normx_4, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));

        disconnect(ptx_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        disconnect(pty_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        disconnect(ptz_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        disconnect(normx_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        disconnect(normx_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        disconnect(normx_5, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));

        disconnect(rect_w_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        disconnect(rect_h_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        disconnect(rect_dep_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged()));
        // Rotation-related
        disconnect(rx_2, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
        disconnect(ry_2, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
        disconnect(rz_2, SIGNAL(valueChanged(double)), this, SLOT(rotationChanged()));
        // Translation-related
        disconnect(tx_2, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
        disconnect(ty_2, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
        disconnect(tz_2, SIGNAL(valueChanged(double)), this, SLOT(translationChanged()));
        //cyclinder-related
        disconnect(cyl_h_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        disconnect(cyl_rad_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cyl()));
        //sphere
        disconnect(radiusval_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_s()));
        //cone
        disconnect(cone_h_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
        disconnect(cone_rad_2, SIGNAL(valueChanged(double)), this, SLOT(planeChanged_cone()));
    }
}

void SpecificWorker::showAvailableGroups()
{
    // Set node type and id
    NodeType1 type = currentNode.type;
    //printf("Node id --  %s",currentNode.type);
    //lineEdit_nodeId->setText(currentNode.id);

    // Treat 'root' special read-only special case
    if (currentNode.id == "root")
    {
        trans_gb_2->hide();
        rot_gb_2->hide();
        //mass_gb->show();
        plane_gb_box->hide();
        plane_gb_cyl->hide();
        plane_gb_cone->hide();
        plane_gb_sphere->hide();

        return;
    }
//    else
//    {
//        //lineEdit_nodeId->setEnabled(true);
//        //nodeType->setText("<b>unknown</b>");
//        //remove_current_nodepushButton->show();
//    }

    // Enable or disable GUI parts depending on the node type
    // and update node data in the interfaze
    switch (type)
    {
        case IMTransform:
            trans_gb_2->show();
            rot_gb_2->show();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();
            showTransform(currentNode.id);
            break;
        case IMRotation:
            trans_gb_2->show();
            rot_gb_2->show();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();

            showRotation(currentNode.id);
            break;
        case IMTranslation:
            trans_gb_2->show();
            rot_gb_2->hide();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();

            showTranslation(currentNode.id);
            break;
        case IMMesh:
            trans_gb_2->hide();
            rot_gb_2->hide();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();
            break;
        case IMPlane:{
            InnerModelPlane *pn = innerModel->getNode<InnerModelPlane>(currentNode.id);
            //char *token=strtok(currentNode.id,"_")
//            vector<QString> result;
//            boost::split(result, currentNode.id, boost::is_any_of("_"));
            //QStringList result=currentNode.id.split("_");
            if(pn->shape==0){
                trans_gb_2->hide();
                rot_gb_2->hide();
                //mass_gb->show();
                plane_gb_box->show();
                plane_gb_cyl->hide();
                plane_gb_cone->hide();
                plane_gb_sphere->hide();
                showPlane(currentNode.id);

            }
            else if(pn->shape==1){
                trans_gb_2->hide();
                rot_gb_2->hide();
                //mass_gb->show();
                plane_gb_box->hide();
                plane_gb_cyl->hide();
                plane_gb_cone->hide();
                plane_gb_sphere->show();
                showPlane_s(currentNode.id);

            }
            else if(pn->shape==2){
                trans_gb_2->hide();
                rot_gb_2->hide();
                //mass_gb->show();
                plane_gb_box->hide();
                plane_gb_cyl->show();
                plane_gb_cone->hide();
                plane_gb_sphere->hide();
                showPlane_cyl(currentNode.id);

            }
            else if(pn->shape==3){
                trans_gb_2->hide();
                rot_gb_2->hide();
                //mass_gb->show();
                plane_gb_box->hide();
                plane_gb_cyl->hide();
                plane_gb_cone->show();
                plane_gb_sphere->hide();
                showPlane_cone(currentNode.id);

            }

        }
            break;

        case IMCamera:
            trans_gb_2->hide();
            rot_gb_2->hide();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();
            break;
        case IMIMU:
            trans_gb_2->hide();
            rot_gb_2->hide();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();
            break;
        case IMLaser:
            trans_gb_2->hide();
            rot_gb_2->hide();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();
            break;
        case IMRGBD:
            trans_gb_2->hide();
            rot_gb_2->hide();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();
            break;
        case IMJoint:
            trans_gb_2->hide();
            rot_gb_2->hide();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();
            break;
        case IMDisplay:
            trans_gb_2->hide();
            rot_gb_2->hide();
            //mass_gb->show();
            plane_gb_box->hide();
            plane_gb_cyl->hide();
            plane_gb_cone->hide();
            plane_gb_sphere->hide();
            break;
    }
}


void SpecificWorker::showTransform(QString id)
{
    showTranslation(id);
    showRotation(id);
}

void SpecificWorker::showRotation(QString id)
{
    InnerModelTransform *t = innerModel->getNode<InnerModelTransform>(id);
    rx_2->setValue(t->backrX);
    ry_2->setValue(t->backrY);
    rz_2->setValue(t->backrZ);
}


void SpecificWorker::showTranslation(QString id)
{
    InnerModelTransform *t = innerModel->getNode<InnerModelTransform>(id);
    tx_2->setValue(t->backtX);
    ty_2->setValue(t->backtY);
    tz_2->setValue(t->backtZ);
}


void SpecificWorker::showPlane(QString id)
{
    NodeType1 type = currentNode.type;
    if (type == IMPlane)
    {
        InnerModelPlane *p = innerModel->getNode<InnerModelPlane>(id);
        ptx_2->setValue(p->point(0));
        pty_2->setValue(p->point(1));
        ptz_2->setValue(p->point(2));
        normx_2->setValue(p->normal(0));
        normy_2->setValue(p->normal(1));
        normz_2->setValue(p->normal(2));
        texture_2->setText(p->texture);
        rect_w_2->setValue(p->width);
        rect_h_2->setValue(p->height);
        rect_dep_2->setValue(p->depth);
        //textureSize->setValue(p->repeat);
    }
    else if (type == IMDisplay)
    {
        InnerModelDisplay *p = innerModel->getNode<InnerModelDisplay>(id);
        ptx_2->setValue(p->point(0));
        pty_2->setValue(p->point(1));
        ptz_2->setValue(p->point(2));
        normx_2->setValue(p->normal(0));
        normy_2->setValue(p->normal(1));
        normz_2->setValue(p->normal(2));
        texture_2->setText(p->texture);
        rect_w_2->setValue(p->width);
        rect_h_2->setValue(p->height);
        rect_dep_2->setValue(p->depth);
    }
}
void SpecificWorker::showPlane_s(QString id)
{
    NodeType1 type = currentNode.type;
    if (type == IMPlane)
    {
        InnerModelPlane *p = innerModel->getNode<InnerModelPlane>(id);
        ptx_5->setValue(p->point(0));
        pty_5->setValue(p->point(1));
        ptz_5->setValue(p->point(2));
        normx_5->setValue(p->normal(0));
        normy_5->setValue(p->normal(1));
        normz_5->setValue(p->normal(2));
        texture_2_s->setText(p->texture);
        radiusval_2->setValue(p->width);
        //rect_h_2->setValue(p->height);
        //rect_dep_2->setValue(p->depth);
        //textureSize->setValue(p->repeat);
    }
    else if (type == IMDisplay)
    {
        InnerModelDisplay *p = innerModel->getNode<InnerModelDisplay>(id);
        ptx_5->setValue(p->point(0));
        pty_5->setValue(p->point(1));
        ptz_5->setValue(p->point(2));
        normx_5->setValue(p->normal(0));
        normy_5->setValue(p->normal(1));
        normz_5->setValue(p->normal(2));
        texture_2_s->setText(p->texture);
        radiusval_2->setValue(p->width);
        //rect_h_2->setValue(p->height);
        //rect_dep_2->setValue(p->depth);
    }

}
void SpecificWorker::showPlane_cyl(QString id)
{
    NodeType1 type = currentNode.type;
    if (type == IMPlane)
    {
        InnerModelPlane *p = innerModel->getNode<InnerModelPlane>(id);
        ptx_3->setValue(p->point(0));
        pty_3->setValue(p->point(1));
        ptz_3->setValue(p->point(2));
        normx_3->setValue(p->normal(0));
        normy_3->setValue(p->normal(1));
        normz_3->setValue(p->normal(2));
        texture_2_cyl->setText(p->texture);
        cyl_rad_2->setValue(p->width);
        cyl_h_2->setValue(p->height);
        //rect_dep_2->setValue(p->depth);
        //textureSize->setValue(p->repeat);
    }
    else if (type == IMDisplay)
    {
        InnerModelDisplay *p = innerModel->getNode<InnerModelDisplay>(id);
        ptx_3->setValue(p->point(0));
        pty_3->setValue(p->point(1));
        ptz_3->setValue(p->point(2));
        normx_3->setValue(p->normal(0));
        normy_3->setValue(p->normal(1));
        normz_3->setValue(p->normal(2));
        texture_2_cyl->setText(p->texture);
        cyl_rad_2->setValue(p->width);
        cyl_h_2->setValue(p->height);
        //rect_dep_2->setValue(p->depth);
    }
}

void SpecificWorker::showPlane_cone(QString id)
{
    NodeType1 type = currentNode.type;
    if (type == IMPlane)
    {
        InnerModelPlane *p = innerModel->getNode<InnerModelPlane>(id);
        ptx_4->setValue(p->point(0));
        pty_4->setValue(p->point(1));
        ptz_4->setValue(p->point(2));
        normx_4->setValue(p->normal(0));
        normy_4->setValue(p->normal(1));
        normz_4->setValue(p->normal(2));
        texture_2_cone->setText(p->texture);
        cone_rad_2->setValue(p->width);
        cone_h_2->setValue(p->height);
        //rect_dep_2->setValue(p->depth);
        //textureSize->setValue(p->repeat);
    }
    else if (type == IMDisplay)
    {
        InnerModelDisplay *p = innerModel->getNode<InnerModelDisplay>(id);
        ptx_4->setValue(p->point(0));
        pty_4->setValue(p->point(1));
        ptz_4->setValue(p->point(2));
        normx_4->setValue(p->normal(0));
        normy_4->setValue(p->normal(1));
        normz_4->setValue(p->normal(2));
        texture_2_cone->setText(p->texture);
        cone_rad_2->setValue(p->width);
        cone_h_2->setValue(p->height);
        //rect_dep_2->setValue(p->depth);
    }
}

void SpecificWorker::planeChanged()
{
    NodeType1 type = currentNode.type;
    if (type == IMPlane)
    {
        InnerModelPlane *m = innerModel->getNode<InnerModelPlane>(currentNode.id);
        m->normal = QVec::vec3(normx_2->value(), normy_2->value(), normz_2->value());
        m->point = QVec::vec3(ptx_2->value(), pty_2->value(), ptz_2->value());
        m->width = rect_w_2->value();
        m->height = rect_h_2->value();
        m->depth = rect_dep_2->value();
        m->texture = texture_2->text();
        //m->repeat = texture_sz_2->value();
        //m->shape = 0;
        //printf("plane changed");
        //prevNode = NULL;
        imv->update();
    }
    else if (type == IMDisplay)
    {
        InnerModelDisplay *m = innerModel->getNode<InnerModelDisplay>(currentNode.id);
        m->normal = QVec::vec3(normx_2->value(), normy_2->value(), normz_2->value());
        m->point = QVec::vec3(ptx_2->value(), pty_2->value(), ptz_2->value());
        m->width = rect_w_2->value();
        m->height = rect_h_2->value();
        m->depth = rect_dep_2->value();
        m->texture = texture_2->text();
        //m->repeat = texture_sz_2->value();
        //m->shape = 0;
        //prevNode = NULL;
        imv->update();
    }
}
void SpecificWorker::planeChanged_cone()
{
    NodeType1 type = currentNode.type;
    if (type == IMPlane)
    {
        InnerModelPlane *m = innerModel->getNode<InnerModelPlane>(currentNode.id);
        m->normal = QVec::vec3(normx_4->value(), normy_4->value(), normz_4->value());
        m->point = QVec::vec3(ptx_4->value(), pty_4->value(), ptz_4->value());
        m->width = cone_rad_2->value();
        m->height = cone_h_2->value();
        //m->depth = rect_dep_2->value();
        m->texture = texture_2_cone->text();
        //m->repeat = texture_sz_2->value();
        //m->shape = 2;
        printf("plane changed");
        //prevNode = NULL;
        imv->update();
    }
    else if (type == IMDisplay)
    {
        InnerModelDisplay *m = innerModel->getNode<InnerModelDisplay>(currentNode.id);
        m->normal = QVec::vec3(normx_4->value(), normy_4->value(), normz_4->value());
        m->point = QVec::vec3(ptx_4->value(), pty_4->value(), ptz_4->value());
        m->width = cone_rad_2->value();
        m->height = cone_h_2->value();
        //m->depth = rect_dep_2->value();
        m->texture = texture_2_cone->text();
        //m->repeat = texture_sz_2->value();
        //m->shape = 0;
        //prevNode = NULL;
        imv->update();
    }
}
void SpecificWorker::planeChanged_cyl()
{
    NodeType1 type = currentNode.type;
    if (type == IMPlane)
    {
        InnerModelPlane *m = innerModel->getNode<InnerModelPlane>(currentNode.id);
        m->normal = QVec::vec3(normx_3->value(), normy_3->value(), normz_3->value());
        m->point = QVec::vec3(ptx_3->value(), pty_3->value(), ptz_3->value());
        m->width = cyl_rad_2->value();
        m->height = cyl_h_2->value();
        //m->depth = rect_dep_2->value();
        m->texture = texture_2_cyl->text();
        //m->repeat = texture_sz_2->value();
        //m->shape = 2;
        printf("plane changed");
        //prevNode = NULL;
        imv->update();
    }
    else if (type == IMDisplay)
    {
        InnerModelDisplay *m = innerModel->getNode<InnerModelDisplay>(currentNode.id);
        m->normal = QVec::vec3(normx_3->value(), normy_3->value(), normz_3->value());
        m->point = QVec::vec3(ptx_3->value(), pty_3->value(), ptz_3->value());
        m->width = cyl_rad_2->value();
        m->height = cyl_h_2->value();
        //m->depth = rect_dep_2->value();
        m->texture = texture_2_cyl->text();
       // m->repeat = texture_sz_2->value();
        //m->shape = 0;
        //prevNode = NULL;
        imv->update();
    }
}

void SpecificWorker::planeChanged_s()
{
    NodeType1 type = currentNode.type;
    if (type == IMPlane)
    {
        InnerModelPlane *m = innerModel->getNode<InnerModelPlane>(currentNode.id);
        m->normal = QVec::vec3(normx_5->value(), normy_5->value(), normz_5->value());
        m->point = QVec::vec3(ptx_5->value(), pty_5->value(), ptz_5->value());
        m->width = radiusval_2->value();
        //m->height = rect_h_2->value();
        //m->depth = rect_dep_2->value();
        m->texture = texture_2_s->text();
        //m->repeat = texture_sz_2->value();
        //m->shape = 1;
        printf("plane changed");
        //prevNode = NULL;
        imv->update();
    }
    else if (type == IMDisplay)
    {
        InnerModelDisplay *m = innerModel->getNode<InnerModelDisplay>(currentNode.id);
        m->normal = QVec::vec3(normx_5->value(), normy_5->value(), normz_5->value());
        m->point = QVec::vec3(ptx_5->value(), pty_5->value(), ptz_5->value());
        m->width = radiusval_2->value();
        //m->height = rect_h_2->value();
        //m->depth = rect_dep_2->value();
        m->texture = texture_2_s->text();
        //m->repeat = texture_sz_2->value();
        //m->shape = 0;
        //prevNode = NULL;
        imv->update();
    }
}





void SpecificWorker::translationChanged()
{
    NodeType1 type = currentNode.type;

    // Treat 'root' special read-only special case
    if (type == IMTransform or type == IMTranslation)
    {
        innerModel->updateTranslationValues(currentNode.id, tx_2->value(), ty_2->value(), tz_2->value());
    }
//    else if (type == IMMesh)
//    {
//        InnerModelMesh *m = innerModel->getNode<InnerModelMesh>(currentNode.id);
//        m->tx = tx->value();
//        m->ty = ty->value();
//        m->tz = tz->value();
//    }
    else
        qFatal("Internal error worker.cpp:%d\n", __LINE__);
}

void SpecificWorker::rotationChanged()
{
    NodeType1 type = currentNode.type;

    // Treat 'root' special read-only special case
    if (type == IMTransform or type == IMTranslation)
    {
        innerModel->updateRotationValues(currentNode.id, rx_2->value(), ry_2->value(), rz_2->value());
    }
//    else if (type == IMMesh)
//    {
//        InnerModelMesh *m = innerModel->getNode<InnerModelMesh>(currentNode.id);
//        m->rx = rx->value();
//        m->ry = ry->value();
//        m->rz = rz->value();
//    }
    else
        qFatal("Internal error worker.cpp:%d\n", __LINE__);
}






////////////////////////////////////////////////////////////////////
///add object
/// ///////////////////////////////////////////////////////////////
void SpecificWorker::add_object()
{
    //newnodeConnections(false);
    groupBox_8->show();
    add_object_final->hide();
    newnodeConnections(true);

}
void SpecificWorker:: add_tree()
{
    treeWidget->show();
}
void SpecificWorker::newnodeConnections(bool enable)
{
    if(enable)
            {
                    connect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(shownode()));
                    connect(add_object_final,SIGNAL(clicked()),this,SLOT(add_new_node()));

            }

             else
             {
                 disconnect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(shownode()));
                 disconnect(add_object_final,SIGNAL(clicked()),this,SLOT(add_new_node()));

    }


}
void SpecificWorker::shownode()
{
    if(comboBox->currentText()=="Click to show type")
    {
        groupBox_12->hide();

    }
    if(comboBox->currentText()=="Box")
    {   add_object_final->show();
        groupBox_12->show();
        trans_gb->show();
        rot_gb->show();
        mass_gb->show();
        texture_gb->show();
        texture_size_gb->show();
        texture_val_gb->show();
        plane_gb->show();
        texture_sp_gb->hide();
        cylinder_gb->hide();
        tx->setValue(0.00);
        ty->setValue(0.00);
        tz->setValue(0.00);
        rx->setValue(0.00);
        ry->setValue(0.00);
        rz->setValue(0.00);
        normx->setValue(0.00);
        normy->setValue(0.00);
        normz->setValue(0.00);
        ptx->setValue(0.00);
        pty->setValue(0.00);
        ptz->setValue(0.00);
        texture_sz->setValue(0.00);
        rect_dep->setValue(200.00);
        rect_w->setValue(400.00);
        rect_h->setValue(400.00);
        texture->setCurrentIndex(1);
       // connect_texture(true);

    }
    if(comboBox->currentText()=="Sphere")
    {   add_object_final->show();
        groupBox_12->show();
        trans_gb->show();
        rot_gb->show();
        mass_gb->show();
        texture_gb->show();
        texture_size_gb->hide();
        texture_val_gb->hide();
        plane_gb->show();
        texture_sp_gb->show();
        cylinder_gb->hide();
        tx->setValue(0.00);
        ty->setValue(0.00);
        tz->setValue(0.00);
        rx->setValue(0.00);
        ry->setValue(0.00);
        rz->setValue(0.00);
        normx->setValue(0.00);
        normy->setValue(0.00);
        normz->setValue(0.00);
        ptx->setValue(0.00);
        pty->setValue(0.00);
        ptz->setValue(0.00);
        texture_sz->setValue(0.00);
        radiusval->setValue(200.00);
        texture->setCurrentIndex(1);
       // connect_texture(true);

    }
    if(comboBox->currentText()=="Cylinder")
    {   add_object_final->show();
        groupBox_12->show();
        trans_gb->show();
        rot_gb->show();
        mass_gb->show();
        texture_gb->show();
        texture_size_gb->hide();
        texture_val_gb->hide();
        plane_gb->show();
        texture_sp_gb->hide();
        cylinder_gb->show();
        tx->setValue(0.00);
        ty->setValue(0.00);
        tz->setValue(0.00);
        rx->setValue(0.00);
        ry->setValue(0.00);
        rz->setValue(0.00);
        normx->setValue(0.00);
        normy->setValue(0.00);
        normz->setValue(0.00);
        ptx->setValue(0.00);
        pty->setValue(1.00);
        ptz->setValue(0.00);
        texture_sz->setValue(0.00);
        cyl_h->setValue(400.00);
        cyl_rad->setValue(100.00);
        texture->setCurrentIndex(1);
       // connect_texture(true);

    }
    if(comboBox->currentText()=="Cone")
    {   add_object_final->show();
        groupBox_12->show();
        trans_gb->show();
        rot_gb->show();
        mass_gb->show();
        texture_gb->show();
        texture_size_gb->hide();
        texture_val_gb->hide();
        plane_gb->show();
        texture_sp_gb->hide();
        cylinder_gb->show();
        tx->setValue(0.00);
        ty->setValue(0.00);
        tz->setValue(0.00);
        rx->setValue(0.00);
        ry->setValue(0.00);
        rz->setValue(0.00);
        normx->setValue(0.00);
        normy->setValue(0.00);
        normz->setValue(0.00);
        ptx->setValue(0.00);
        pty->setValue(1.00);
        ptz->setValue(0.00);
        texture_sz->setValue(0.00);
        cyl_h->setValue(400.00);
        cyl_rad->setValue(100.00);
        texture->setCurrentIndex(1);
       // connect_texture(true);

    }
    //connect_texture(true);
}

 void SpecificWorker::add_new_node()
 {

     InnerModelNode *par= innerModel->getNode(parentid->text());
     //InnerModelNode *par= innerModel->getNode("world");
     //qDebug("%s",parentid->text().toLatin1().constData());
     //printf(parentid->text());
//         if (par==NULL)
//         {
//             msgBox.setText("Enter valid Parent Id");
//             msgBox.exec();
//         }

         //else
        // {
             InnerModelNode *check= innerModel->getNode(nodeid->text());
            //printf(nodeid->text());
            qDebug("%s",nodeid->text().toLatin1().constData());
             if(check==NULL)
             {
                 //printf("");
                 if(comboBox->currentText()=="Box")
                 {
                     InnerModelTransform *newnode = (InnerModelTransform *)innerModel->newTransform(nodeid->text(), "static", par, tx->value(), ty->value(), tz->value(), rx->value(), ry->value(), rz->value(), mass_b->value());
                     par->addChild(newnode);
                     InnerModelNode *par1= innerModel->getNode<InnerModelNode>(nodeid->text());
                     InnerModelNode *check2= innerModel->getNode(nodeid->text()+"_pBox");
                     if(check2==NULL)
                     {
                      if(texture->currentText()=="Checkerboard")
                      {
                          InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg", rect_w->value(), rect_h->value()
                                                  , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                  , ptx->value(), pty->value(), ptz->value(), 0,0);
                          par1->addChild(newnode1);
                          flag=0;
                          printf("checkerboard box");
                      }
                      else if(texture->currentText()=="Metal")
                      {

                          InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg", rect_w->value(), rect_h->value()
                                                  , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                  , ptx->value(), pty->value(), ptz->value(), 0,0);
                          par1->addChild(newnode1);
                          flag=0;
                      }
                      else if(texture->currentText()=="Blue"){
                          InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/blue.jpg", rect_w->value(), rect_h->value()
                                                  , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                  , ptx->value(), pty->value(), ptz->value(), 0,0);
                          par1->addChild(newnode1);
                          flag=0;
                      }
                      else if(texture->currentText()=="Raw")
                      {
                          InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/r.jpeg", rect_w->value(), rect_h->value()
                                                  , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                  , ptx->value(), pty->value(), ptz->value(), 0,0);
                          par1->addChild(newnode1);
                          flag=0;
                      }
                      else if(texture->currentText()=="Klein Blue")
                      {
                          InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/klein_blue3.jpg", rect_w->value(), rect_h->value()
                                                  , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                  , ptx->value(), pty->value(), ptz->value(), 0,0);
                          par1->addChild(newnode1);
                          flag=0;
                      }
                      else if(comboBox_texture->currentText()=="Wood")
                      {
                          InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/wood.jpg", rect_w->value(), rect_h->value(), rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value(), ptx->value(), pty->value(), ptz->value(), 0,0);
                          par1->addChild(newnode1);
                          flag=0;
                      }
                      else if(texture->currentText()=="Grid")
                      {
                          InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/grid.png", rect_w->value(), rect_h->value()
                                                  , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                  , ptx->value(), pty->value(), ptz->value(), 0,0);
                          par1->addChild(newnode1);
                          flag=0;
                      }
                      else if(texture->currentText()=="Indice")
                      {
                          InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/indice.jpeg", rect_w->value(), rect_h->value()
                                                  , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                  , ptx->value(), pty->value(), ptz->value(), 0,0);
                          par1->addChild(newnode1);
                          flag=0;
                      }



                     //printf("add_object");
                     //qDebug() << "add_object";
                     //qDebug("texture used ; ..   ***** %s",texture->text().toLatin1().constData());
                     //qDebug() << "add_object"<<texture->text();
                 }
                 }
                 else if(comboBox->currentText()=="Sphere")
                 {
                     InnerModelTransform *newnode = (InnerModelTransform *)innerModel->newTransform(nodeid->text(), "static", par, tx->value(), ty->value(), tz->value(), rx->value(), ry->value(), rz->value(), mass_b->value());
                     par->addChild(newnode);
                     InnerModelNode *par1= innerModel->getNode<InnerModelNode>(nodeid->text());
                     InnerModelNode *check2= innerModel->getNode(nodeid->text()+"_pSphere");
                     if(check2==NULL)
                     {
                     //connect_texture(true);
                     //texture_out= connect_texture(true);
                         if(texture->currentText()=="Checkerboard")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg", radiusval->value(), rect_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,1);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Metal")
                         {

                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg", radiusval->value(), rect_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,1);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Blue"){
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/blue.jpg", radiusval->value(), rect_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,1);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         if(texture->currentText()=="Raw")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/r.jpeg", radiusval->value(), rect_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,1);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Klein Blue")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/klein_blue3.jpg", radiusval->value(), rect_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,1);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Wood")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/wood.jpg", radiusval->value(), rect_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,1);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Grid")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/grid.png", radiusval->value(), rect_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,1);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Indice")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/indice.jpeg", radiusval->value(), rect_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,1);
                             par1->addChild(newnode1);
                             flag=0;
                         }


                     //InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, texture->currentText(), radiusval->value(), rect_h->value()
                                     //        , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                      //       , ptx->value(), pty->value(), ptz->value(), 0,1);

                     //printf("add_object");
                     //qDebug() << "add_object";
                     //qDebug("texture used ; ..   ***** %s",texture->text().toLatin1().constData());
                     //qDebug() << "add_object"<<texture->text();
                 }
                 }
                 else if(comboBox->currentText()=="Cylinder")
                 {
                     InnerModelTransform *newnode = (InnerModelTransform *)innerModel->newTransform(nodeid->text(), "static", par, tx->value(), ty->value(), tz->value(), rx->value(), ry->value(), rz->value(), mass_b->value());
                     par->addChild(newnode);
                     InnerModelNode *par1= innerModel->getNode<InnerModelNode>(nodeid->text());
                     InnerModelNode *check2= innerModel->getNode(nodeid->text()+"_pCylinder");
                     if(check2==NULL)
                     {
                     //connect_texture(true);
                     //texture_out= connect_texture(true);
                         if(texture->currentText()=="Checkerboard")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,2);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Metal")
                         {

                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,2);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Blue"){
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/blue.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,2);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         if(texture->currentText()=="Raw")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/r.jpeg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,2);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Klein Blue")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/klein_blue3.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,2);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Wood")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/wood.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,2);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Grid")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/grid.png", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,2);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Indice")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/indice.jpeg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,2);
                             par1->addChild(newnode1);
                             flag=0;
                         }


                     //InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, texture->currentText(), radiusval->value(), rect_h->value()
                                     //        , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                      //       , ptx->value(), pty->value(), ptz->value(), 0,1);

                     //printf("add_object");
                     //qDebug() << "add_object";
                     //qDebug("texture used ; ..   ***** %s",texture->text().toLatin1().constData());
                     //qDebug() << "add_object"<<texture->text();
                 }
                 }
                 else if(comboBox->currentText()=="Cone")
                 {
                     InnerModelTransform *newnode = (InnerModelTransform *)innerModel->newTransform(nodeid->text(), "static", par, tx->value(), ty->value(), tz->value(), rx->value(), ry->value(), rz->value(), mass_b->value());
                     par->addChild(newnode);
                     InnerModelNode *par1= innerModel->getNode<InnerModelNode>(nodeid->text());
                     InnerModelNode *check2= innerModel->getNode(nodeid->text()+"_pCone");
                     if(check2==NULL)
                     {
                     //connect_texture(true);
                     //texture_out= connect_texture(true);
                         if(texture->currentText()=="Checkerboard")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,3);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Metal")
                         {

                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,3);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Blue"){
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/blue.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,3);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         if(texture->currentText()=="Raw")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/r.jpeg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,3);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Klein Blue")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/klein_blue3.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,3);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Wood")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/wood.jpg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,3);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Grid")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/grid.png", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,3);
                             par1->addChild(newnode1);
                             flag=0;
                         }
                         else if(texture->currentText()=="Indice")
                         {
                             InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, "/home/robocomp/robocomp/files/osgModels/textures/indice.jpeg", cyl_rad->value(), cyl_h->value()
                                                     , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                                     , ptx->value(), pty->value(), ptz->value(), 0,3);
                             par1->addChild(newnode1);
                             flag=0;
                         }


                     //InnerModelPlane *newnode1 = (InnerModelPlane *)innerModel->newPlane(nodeid->text()+"_p", par1, texture->currentText(), radiusval->value(), rect_h->value()
                                     //        , rect_dep->value(), texture_sz->value(), normx->value(), normy->value(), normz->value()
                                      //       , ptx->value(), pty->value(), ptz->value(), 0,1);

                     //printf("add_object");
                    // qDebug() << "add_object";
                     //qDebug("texture used ; ..   ***** %s",texture->text().toLatin1().constData());
                     //qDebug() << "add_object"<<texture->text();
                 }
                 }
             }
             if(flag==0)
             {  qDebug()<<"flag=0";

                 this->viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );


                  viewer->~OsgView();
//                  if(!rgbd_id.isEmpty()){
//                       qDebug()<<"destroy";
//                       imv->cameras[rgbd_id].viewerCamera->~Viewer();
//                  }
                  //rgbd_id.clear();
                  viewer = new OsgView(frameOSG);
                  imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup(),false);
                   qDebug()<< "hogaya....hahahhaha " << nodeid->text();
                   nodeid->clear();
                   comboBox->setCurrentIndex(0);
                   texture->setCurrentIndex(0);
                   comboBox_texture->setCurrentIndex(0);
                   //parentid->clear();
                   groupBox_12->hide();
                   groupBox_8->hide();
                   tx->setValue(0.00);
                   ty->setValue(0.00);
                   tz->setValue(0.00);
                   rx->setValue(0.00);
                   ry->setValue(0.00);
                   rz->setValue(0.00);
                   normx->setValue(0.00);
                   normy->setValue(0.00);
                   normz->setValue(0.00);
                   ptx->setValue(0.00);
                   pty->setValue(0.00);
                   ptz->setValue(0.00);
                   texture_sz->setValue(0.00);
                   rect_dep->setValue(0.00);
                   rect_w->setValue(0.00);
                   rect_h->setValue(0.00);
                   radiusval->setValue(0.00);
                   mass_b->setValue(0.00);
                   cyl_h->setValue(0.00);
                   cyl_rad->setValue(0.00);


                   this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);

//                   plane1 = "";
//                   plane2 = "";
                   flag=1;
                  // fillNodeMap(innerModel->getNode("root"), NULL);
             }
             disconnect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
             treeWidget->clear();
             connect(treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this, SLOT(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)));
             qDebug()<<"again tree update";
             fillNodeMap(innerModel->getNode("root"), NULL);
//             if(!rgbd_id.isEmpty()){
//                  qDebug()<<"destroy";
//                  imv->cameras[rgbd_id].viewerCamera->~Viewer();
//                 }
             //this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);
             //rgbd_id.clear();
             imv->update();
         //}

 }

 void SpecificWorker::fillNodeMap(InnerModelNode *node, QTreeWidgetItem *parent)
 {   InnerModelMesh *mesh;
     InnerModelPlane *plane;
     InnerModelCamera *camera;
     InnerModelTransform *transform;
     InnerModelIMU *imu;
     InnerModelLaser *laser;
     InnerModelRGBD *rgbd;
     InnerModelJoint *joint;
     InnerModelDisplay *display;

     QTreeWidgetItem *item=new QTreeWidgetItem(QTreeWidgetItem::Type);
     WorkerNode wnode;
     wnode.id =node->id;
     wnode.item = item;

     item->setText(0,node->id);
     if(not parent)
     {
         treeWidget->addTopLevelItem(item);
     }
     else
     {
         parent->addChild(item);
     }
     if ((transform = dynamic_cast<InnerModelTransform *>(node)))
         {
             if ((joint = dynamic_cast<InnerModelJoint *>(node)))
                 wnode.type = IMJoint;
             else if (transform->gui_translation and transform->gui_rotation)
                 wnode.type = IMTransform;
             else if (transform->gui_translation)
                 wnode.type = IMTranslation;
             else if (transform->gui_rotation)
                 wnode.type = IMRotation;
             else
                 qFatal("Void transformation node (%s)?\n", transform->id.toStdString().c_str());

             nodeMap[wnode.id] = wnode;
             nodeMapByItem[item] = wnode;
             for(int i=0; i<node->children.size(); i++)
             {
                 fillNodeMap(node->children[i], item);
             }
         }
         else if ((camera = dynamic_cast<InnerModelCamera *>(node)))
         {
             if ((rgbd = dynamic_cast<InnerModelRGBD *>(node)))
             {
                 wnode.type = IMRGBD;
                 rgbd_id = node->id;
                 qDebug()<<"rgbd-id %s"<<node->id;
             }
             else
                 wnode.type = IMCamera;
             nodeMap[wnode.id] = wnode;
             nodeMapByItem[item] = wnode;
         }
         else if ((plane = dynamic_cast<InnerModelPlane *>(node)))
         {
             wnode.type = IMPlane;
             nodeMap[wnode.id] = wnode;
             nodeMapByItem[item] = wnode;
         }
         else if ((mesh = dynamic_cast<InnerModelMesh *>(node)))
         {
             wnode.type = IMMesh;
             nodeMap[wnode.id] = wnode;
             nodeMapByItem[item] = wnode;
         }
         else if ((imu = dynamic_cast<InnerModelIMU *>(node)))
         {
             wnode.type = IMIMU;
             nodeMap[wnode.id] = wnode;
             nodeMapByItem[item] = wnode;
         }
         else if ((laser = dynamic_cast<InnerModelLaser *>(node)))
         {
             wnode.type = IMLaser;
             nodeMap[wnode.id] = wnode;
             nodeMapByItem[item] = wnode;
         }
         else if ((display = dynamic_cast<InnerModelDisplay *>(node)))
         {
             wnode.type = IMDisplay;
             nodeMap[wnode.id] = wnode;
             nodeMapByItem[item] = wnode;
         }
         else
         {
             qDebug() << "InnerModelReader::InnerModelReader(): Error: Unknown type of node (see node id=\n" << node->id << "\")";
             throw "InnerModelReader::InnerModelReader(): Error: Unknown type of node";
         }


 }
// void SpecificWorker:: changeTexture()
// {
//     connect(comboBox_texture,SIGNAL(currentIndexChanged(int)),this,SLOT(floor_texture()));
// }
  void SpecificWorker:: floor_texture()
  {
      qDebug() << "floor_texture";
      if(comboBox_texture->currentText()=="Metal")
      {
          InnerModelPlane *m = innerModel->getNode<InnerModelPlane>("ddG");
          m->normal = QVec::vec3(0,1,0);
          m->point = QVec::vec3(0,0,0);
          m->width = 5000.00;
          m->height = 5000.00;
          //const char* str = "home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg";
          //m->texture =QString::fromUtf8(str);
          m->texture ="/home/robocomp/robocomp/files/osgModels/textures/Metal.jpg";
          m->repeat = 1000;
          //prevNode = NULL;
          //imv->update();
         // qDebug() << "metal hogaya";
          this->viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );
//          if(!rgbd_id.isEmpty())
//               imv->cameras[rgbd_id].viewerCamera->~Viewer();
          viewer->~OsgView();
          //rgbd_id.clear();
          viewer = new OsgView(frameOSG);
          imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup(),false);
          //fillNodeMap(innerModel->getNode("root"), NULL);
          this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);
          //imv->update();

      }
      else if(comboBox_texture->currentText()=="Blue")
      {
          InnerModelPlane *m = innerModel->getNode<InnerModelPlane>("ddG");
          m->normal = QVec::vec3(0,1,0);
          m->point = QVec::vec3(0,0,0);
          m->width = 5000.00;
          m->height = 5000.00;
          //const char* str = "home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg";
          //m->texture =QString::fromUtf8(str);
          m->texture ="/home/robocomp/robocomp/files/osgModels/textures/blue.jpg";
          m->repeat = 1000;
          //prevNode = NULL;
          //imv->update();
          //qDebug() << "metal hogaya";

          this->viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );
//          if(!rgbd_id.isEmpty())
//               imv->cameras[rgbd_id].viewerCamera->~Viewer();
          viewer->~OsgView();
        //  rgbd_id.clear();

          viewer = new OsgView(frameOSG);
          imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup(),false);

          this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);
          imv->update();

      }
      else if(comboBox_texture->currentText()=="Checkerboard")
      {
          InnerModelPlane *m = innerModel->getNode<InnerModelPlane>("ddG");
          m->normal = QVec::vec3(0,1,0);
          m->point = QVec::vec3(0,0,0);
          m->width = 5000.00;
          m->height = 5000.00;
          //const char* str = "home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg";
          //m->texture =QString::fromUtf8(str);
          m->texture ="/home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg";
          m->repeat = 1000;
          //prevNode = NULL;
          //imv->update();
          //qDebug() << "metal hogaya";

          this->viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );
//          if(!rgbd_id.isEmpty())
//               imv->cameras[rgbd_id].viewerCamera->~Viewer();
          viewer->~OsgView();
      //    rgbd_id.clear();


          viewer = new OsgView(frameOSG);
          imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup(),false);

          this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);
          imv->update();

      }
      else if(comboBox_texture->currentText()=="Grid")
      {
          InnerModelPlane *m = innerModel->getNode<InnerModelPlane>("ddG");
          m->normal = QVec::vec3(0,1,0);
          m->point = QVec::vec3(0,0,0);
          m->width = 5000.00;
          m->height = 5000.00;
          //const char* str = "home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg";
          //m->texture =QString::fromUtf8(str);
          m->texture ="/home/robocomp/robocomp/files/osgModels/textures/grid.png";
          m->repeat = 1000;
          //prevNode = NULL;
          //imv->update();
         // qDebug() << "metal hogaya";

          this->viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );
//          if(!rgbd_id.isEmpty())
//               imv->cameras[rgbd_id].viewerCamera->~Viewer();
          //viewer->~OsgView();
        //  rgbd_id.clear();
          viewer->~OsgView();
          viewer = new OsgView(frameOSG);
          imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup(),false);

          this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);
          imv->update();

      }
      else if(comboBox_texture->currentText()=="Indice")
      {
          InnerModelPlane *m = innerModel->getNode<InnerModelPlane>("ddG");
          m->normal = QVec::vec3(0,1,0);
          m->point = QVec::vec3(0,0,0);
          m->width = 5000.00;
          m->height = 5000.00;
          //const char* str = "home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg";
          //m->texture =QString::fromUtf8(str);
          m->texture ="/home/robocomp/robocomp/files/osgModels/textures/índice.jpeg";
          m->repeat = 1000;
          //prevNode = NULL;
          //imv->update();
          //qDebug() << "metal hogaya";

          this->viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );
//          if(!rgbd_id.isEmpty())
//               imv->cameras[rgbd_id].viewerCamera->~Viewer();
          viewer->~OsgView();
       //   rgbd_id.clear();
          //viewer->~OsgView();
          viewer = new OsgView(frameOSG);
          imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup(),false);

          this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);
          imv->update();

      }
      else if(comboBox_texture->currentText()=="Klein Blue")
      {
          InnerModelPlane *m = innerModel->getNode<InnerModelPlane>("ddG");
          m->normal = QVec::vec3(0,1,0);
          m->point = QVec::vec3(0,0,0);
          m->width = 5000.00;
          m->height = 5000.00;
          //const char* str = "home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg";
          //m->texture =QString::fromUtf8(str);
          m->texture ="/home/robocomp/robocomp/files/osgModels/textures/klein_blue3.jpg";
          m->repeat = 1000;
          //prevNode = NULL;
          //imv->update();
         // qDebug() << "metal hogaya";

          this->viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );
//          if(!rgbd_id.isEmpty())
//               imv->cameras[rgbd_id].viewerCamera->~Viewer();
          viewer->~OsgView();
        //  rgbd_id.clear();
          //viewer->~OsgView();
          viewer = new OsgView(frameOSG);
          imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup(),false);

          this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);
          imv->update();

      }
      else if(comboBox_texture->currentText()=="Raw")
      {
          InnerModelPlane *m = innerModel->getNode<InnerModelPlane>("ddG");
          m->normal = QVec::vec3(0,1,0);
          m->point = QVec::vec3(0,0,0);
          m->width = 5000.00;
          m->height = 5000.00;
          //const char* str = "home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg";
          //m->texture =QString::fromUtf8(str);
          m->texture ="/home/robocomp/robocomp/files/osgModels/textures/r.jpeg";
          m->repeat = 1000;
          //prevNode = NULL;
          //imv->update();
         // qDebug() << "metal hogaya";

          this->viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );
//          if(!rgbd_id.isEmpty())
//               imv->cameras[rgbd_id].viewerCamera->~Viewer();
          viewer->~OsgView();
     //     rgbd_id.clear();
          //viewer->~OsgView();
          viewer = new OsgView(frameOSG);
          imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup(),false);

          this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);
          imv->update();

      }
      else if(comboBox_texture->currentText()=="Wood")
      {
          InnerModelPlane *m = innerModel->getNode<InnerModelPlane>("ddG");
          m->normal = QVec::vec3(0,1,0);
          m->point = QVec::vec3(0,0,0);
          m->width = 5000.00;
          m->height = 5000.00;
          //const char* str = "home/robocomp/robocomp/files/osgModels/textures/checkerboard.jpg";
          //m->texture =QString::fromUtf8(str);
          m->texture ="/home/robocomp/robocomp/files/osgModels/textures/wood.jpg";
          m->repeat = 1000;
          //prevNode = NULL;
          //imv->update();
         // qDebug() << "metal hogaya";

          this->viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );
//          if(!rgbd_id.isEmpty())
//               imv->cameras[rgbd_id].viewerCamera->~Viewer();
          viewer->~OsgView();
      //    rgbd_id.clear();
          //viewer->~OsgView();
          viewer = new OsgView(frameOSG);
          imv = new InnerModelViewer(innerModel, "root", viewer->getRootGroup(),false);

          this->viewer->setHomePosition(eye,osg::Vec3(0.f,0.,-40.),up, false);
          imv->update();

      }

  }


//////////////////////////////////////////////////////////////////////
/// Updates
////////////////////////////////////////////////////////////////////////

void SpecificWorker::updateCameras()
{
	QHash<QString, IMVCamera>::const_iterator i = imv->cameras.constBegin();
	{
		//QMutexLocker vm(viewerMutex);

		while (i != imv->cameras.constEnd())
		{
			RTMat rt= innerModel->getTransformationMatrix("root",i.key());
			// Put camera in its position
			imv->cameras[i.key()].viewerCamera->getCameraManipulator()->setByMatrix(QMatToOSGMat4(rt));

			for (int n=0; n<imv->cameras.size() ; ++n)
			{
				imv->cameras[i.key()].viewerCamera->frame();
			}
			i++;
		}
	}	
}

void SpecificWorker::updateLasers()
{
	/// Remove previous laser shapes 
	for (QHash<QString, IMVLaser>::iterator laser = imv->lasers.begin(); laser != imv->lasers.end(); laser++)
	{
		//QMutexLocker locker(viewerMutex);
		if (laser->osgNode->getNumChildren() > 0)
		{
			laser->osgNode->removeChild(0, laser->osgNode->getNumChildren());
		}
	}
	// Laser
	{
		//QMutexLocker vm(viewerMutex);
		QMutexLocker lcds(laserDataCartArray_mutex);
		for (QHash<QString, IMVLaser>::iterator laser = imv->lasers.begin(); laser != imv->lasers.end(); laser++)
		{
			QString id=laser->laserNode->id;

			if (laserDataCartArray.contains(id) == false)
			{
				osg::Vec3Array *v= new osg::Vec3Array();
				v->resize(laser->laserNode->measures+1);
				laserDataCartArray.insert(id,v);
			}

			// create and insert laser data
			worker = this;
			laserDataArray.insert(laser->laserNode->id, LASER_createLaserData(laser.value()));

			// create and insert laser shape
			if (true) // DRAW LASER
			{
				osg::ref_ptr<osg::Node> p=NULL;
				if (id=="laserSecurity")
				{
					p = viewer->addPolygon(*(laserDataCartArray[id]), osg::Vec4(0.,0.,1.,0.4));
				}
				else
				{
					p = viewer->addPolygon(*(laserDataCartArray[id]));
				}
				if (p!=NULL)
				{
					laser->osgNode->addChild(p);
				}
			}
		}
	}
}

// Update all the joint positions
void SpecificWorker::updateJoints(const float delta)
{
	// 	printf("%s: %d\n", __FILE__, __LINE__);
	QHash<QString, JointMovement>::const_iterator iter;
	for (iter = jointMovements.constBegin() ; iter != jointMovements.constEnd() ; ++iter)
	{
		InnerModelNode *node = innerModel->getNode(iter.key());
		InnerModelJoint *ajoint;
		InnerModelPrismaticJoint *pjoint;
		if ((ajoint = dynamic_cast<InnerModelJoint*>(node)) != NULL)
		{
			const float angle = ajoint->getAngle();
			const float amount = fminf(fabsf(iter->endPos - angle), iter->endSpeed  *delta);
			switch (iter->mode)
			{
			case JointMovement::FixedPosition:
				ajoint->setAngle(iter->endPos);
				break;
			case JointMovement::TargetPosition:
				if (iter->endPos > angle)
					ajoint->setAngle(angle + amount);
				else if (iter->endPos < angle)
					ajoint->setAngle(angle - amount);
				break;
			case JointMovement::TargetSpeed:
				ajoint->setAngle(angle + iter->endSpeed  *delta);
				break;
			default:
				break;
			}
		}
		else if ((pjoint = dynamic_cast<InnerModelPrismaticJoint*>(node)) != NULL)
		{
			pjoint->setPosition(iter->endPos);
		}
	}
	// 	printf("%s: %d\n", __FILE__, __LINE__);
}

void SpecificWorker::updateTouchSensors()
{
	std::map<uint32_t, TouchSensorServer>::iterator touchIt;
	for (touchIt=touch_servers.begin(); touchIt!= touch_servers.end(); touchIt++)
	{
		for (uint32_t sss=0; sss<touchIt->second.sensors.size(); sss++)
		{
			// 	TouchSensorI *interface;
			// touchIt->interface->sensorMap[touchIt->sensors[sss].id].value = XXX
			InnerModelTouchSensor *sensorr = touchIt->second.sensors[sss];
			std::string idd = sensorr->id.toStdString();
// 				printf("%d: %s (%f)\n",
//
// 					touchIt->second.port,
//
// 					idd.c_str(),
//
// 					touchIt->second.interface->sensorMap[idd].value
//
// 				);
		}
	}
}

///////////////////////////////////////////////////////////////////////
/// start sensors and motors
///////////////////////////////////////////////////////////////////////

void SpecificWorker::startServers()
{
	walkTree();
	includeLasers();
	includeRGBDs();
}

void SpecificWorker::addDFR(InnerModelDifferentialRobot *node)
{
	const uint32_t port = node->port;
	if (dfr_servers.count(port) == 0)
	{
		dfr_servers.insert(std::pair<uint32_t, DifferentialRobotServer>(port, DifferentialRobotServer(communicator, worker, port)));
	}
	dfr_servers.at(port).add(node);
}


void SpecificWorker::addOMN(InnerModelOmniRobot *node)
{
	const uint32_t port = node->port;
	if (omn_servers.count(port) == 0)
	{
		omn_servers.insert(std::pair<uint32_t, OmniRobotServer>(port, OmniRobotServer(communicator, worker, port)));
	}
	omn_servers.at(port).add(node);
}

void SpecificWorker::addDisplay(InnerModelDisplay *node)
{
	const uint32_t port = node->port;
	if (display_servers.count(port) == 0)
	{
		display_servers.insert(std::pair<uint32_t, DisplayServer>(port, DisplayServer(communicator, this, port)));
	}
	display_servers.at(port).add(node);
}

void SpecificWorker::addIMU(InnerModelIMU *node)
{
	const uint32_t port = node->port;
	if (imu_servers.count(port) == 0)
	{
		imu_servers.insert(std::pair<uint32_t, IMUServer>(port, IMUServer(communicator, worker, port)));
	}
	imu_servers.at(port).add(node);
}


void SpecificWorker::addJM(InnerModelJoint *node)
{
	const uint32_t port = node->port;
	if (jm_servers.count(port) == 0)
	{
		jm_servers.insert(std::pair<uint32_t, JointMotorServer>(port, JointMotorServer(communicator, worker, port)));
	}
	jm_servers.at(port).add(node);
}

void SpecificWorker::addJM(InnerModelPrismaticJoint *node)
{
	const uint32_t port = node->port;
	if (jm_servers.count(port) == 0)
	{
		jm_servers.insert(std::pair<uint32_t, JointMotorServer>(port, JointMotorServer(communicator, worker, port)));
	}
	jm_servers.at(port).add(node);
}

void SpecificWorker::addTouch(InnerModelTouchSensor *node)
{
	const uint32_t port = node->port;
	if (touch_servers.count(port) == 0)
	{
		touch_servers.insert(std::pair<uint32_t, TouchSensorServer>(port, TouchSensorServer(communicator, worker, port)));
	}
	touch_servers.at(port).add(node);
}


void SpecificWorker::addLaser(InnerModelLaser *node)
{
	const uint32_t port = node->port;
	if (laser_servers.count(port) == 0)
	{
		laser_servers.insert(std::pair<uint32_t, LaserServer>(port, LaserServer(communicator, worker, port)));
	}
	laser_servers.at(port).add(node);
}


void SpecificWorker::addRGBD(InnerModelRGBD *node)
{
	const uint32_t port = node->port;
	if (rgbd_servers.count(port) == 0)
	{
		rgbd_servers.insert(std::pair<uint32_t, RGBDServer>(port, RGBDServer(communicator, worker, port)));
	}
	rgbd_servers.at(port).add(node);
}

void SpecificWorker::removeJM(InnerModelJoint *node)
{
	std::map<uint32_t, JointMotorServer>::iterator it;
	for (it = jm_servers.begin(); it != jm_servers.end(); ++it)
	{
		it->second.remove(node);
		// TODO: arreglar
// 		if (it->second.empty())
// 		{
// 			worker->scheduleShutdown(&(it->second));
// 			servers.erase(it);
// 		}
	}
}

void SpecificWorker::includeLasers()
{
	QHash<QString, IMVLaser>::const_iterator it;
	for (it = imv->lasers.constBegin() ; it != imv->lasers.constEnd() ; ++it)
	{
		qDebug() << it.key() << ": ";
		printf(" %p\n", it.value().laserNode);
		addLaser(it.value().laserNode);
	}
}

void SpecificWorker::includeRGBDs()
{
	QHash<QString, IMVCamera>::const_iterator it;
	for (it = imv->cameras.constBegin() ; it != imv->cameras.constEnd() ; ++it)
	{
		addRGBD(it.value().RGBDNode);
        qDebug()<<"rgbd-include-existing function";
	}

}

void SpecificWorker::walkTree(InnerModelNode *node)
{
	if (node == NULL)
	{
		node = innerModel->getRoot();
		//std::cout << "ROOT: " << (void *)node << "  " << (uint64_t)node << std::endl;
	}
	else
	{
		//std::cout << "nrml: " << (void *)node << "  " << (uint64_t)node << std::endl;
	}

	QList<InnerModelNode*>::iterator it;
	for (it=node->children.begin(); it!=node->children.end(); ++it)
	{
		//std::cout << "  --> " << (void *)*it << "  " << (uint64_t)*it << std::endl;
		InnerModelDifferentialRobot *differentialNode = dynamic_cast<InnerModelDifferentialRobot *>(*it);
		if (differentialNode != NULL)
		{
			//qDebug() << "DifferentialRobot " << differentialNode->id << differentialNode->port;
			addDFR(differentialNode);
		}

		InnerModelOmniRobot *omniNode = dynamic_cast<InnerModelOmniRobot *>(*it);
		if (omniNode != NULL)
		{
			//qDebug() << "OmniRobot " << omniNode->id << omniNode->port;
			addOMN(omniNode);
		}

   		InnerModelDisplay *displayNode = dynamic_cast<InnerModelDisplay *>(*it);

		if (displayNode != NULL)
		{

			//qDebug() << "OmniRobot " << omniNode->id << omniNode->port;
			addDisplay(displayNode);
		}

		InnerModelIMU *imuNode = dynamic_cast<InnerModelIMU *>(*it);
		if (imuNode != NULL)
		{
			//qDebug() << "IMU " << imuNode->id << imuNode->port;
			addIMU(imuNode);
		}

		InnerModelJoint *jointNode = dynamic_cast<InnerModelJoint *>(*it);
		if (jointNode != NULL)
		{
			//qDebug() << "Joint " << (*it)->id;
			addJM(jointNode);
		}
		InnerModelPrismaticJoint *pjointNode = dynamic_cast<InnerModelPrismaticJoint *>(*it);
		if (pjointNode != NULL)
		{
			//qDebug() << "Joint " << (*it)->id;
			addJM(pjointNode);
		}

		InnerModelTouchSensor *touchNode = dynamic_cast<InnerModelTouchSensor *>(*it);
		if (touchNode != NULL)
		{
			qDebug() << "Touch " << (*it)->id << "CALLING addTouch";
			addTouch(touchNode);
		}

		walkTree(*it);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
///// Aux
////////////////////////////////////////////////////////////////////////////////////////
osg::Group *SpecificWorker::getRootGroup()
{
	//QMutexLocker vm(viewerMutex);
	guard gl(innerModel->mutex);
	return viewer->getRootGroup();
}

// ------------------------------------------------------------------------------------------------
// Private
// ------------------------------------------------------------------------------------------------

InnerModel *SpecificWorker::getInnerModel()
{
	guard gl(innerModel->mutex);
	return innerModel.get();
}

InnerModelViewer *SpecificWorker::getInnerModelViewer()
{
	guard gl(innerModel->mutex);
	return imv;
}

void SpecificWorker::scheduleShutdown(JointMotorServer *j)
{
	jointServersToShutDown.push_back(j);
}

// Refills laserData with new values
RoboCompLaser::TLaserData SpecificWorker::LASER_createLaserData(const IMVLaser &laser)
{
	//QMutexLocker vm(worker->viewerMutex);
	//QMutexLocker locker(worker->mutex);
	QMutexLocker ldc(laserDataCartArray_mutex);
	guard gl(innerModel->mutex);
// 		printf("osg threads running... %d\n", viewer->areThreadsRunning());
	static RoboCompLaser::TLaserData laserData;
	int measures = laser.laserNode->measures;
	QString id = laser.laserNode->id;
	float iniAngle = -laser.laserNode->angle/2;
	float finAngle = laser.laserNode->angle/2;
	float_t maxRange = laser.laserNode->max;
	laserData.resize(measures);

	double angle = finAngle;  //variable to iterate angle increments
	//Origin is the starting point of the laser
	const osg::Vec3 P = QVecToOSGVec(innerModel->laserTo("root", id, 0, 0));
	const float incAngle = (fabs(iniAngle)+fabs(finAngle)) / (float)measures;
	osg::Vec3 Q,R;


	for (int i=0 ; i<measures; i++)
	{
		laserData[i].angle = angle;
		laserData[i].dist = maxRange;


		laserDataCartArray[id]->operator[](i) = QVecToOSGVec(QVec::vec3(maxRange*sin(angle), 0, maxRange*cos(angle)));

		//Destination point calculation
		Q = QVecToOSGVec(innerModel->laserTo("root", id, maxRange, angle));
		//Create the segment of intersection
		osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::MODEL, P, Q);
		osgUtil::IntersectionVisitor visitor(intersector.get());

		/// Passing the visitor to root
		viewer->getRootGroup()->accept(visitor);

		if (intersector->containsIntersections() and id!="laserSecurity")
		{
			osgUtil::LineSegmentIntersector::Intersection result = *(intersector->getIntersections().begin());
			R = result.getWorldIntersectPoint(); // in world space

			R.x() = R.x() - P.x();
			R.y() = R.y() - P.y();
			R.z() = R.z() - P.z();
			const float dist = sqrt(R.x() *R.x() + R.y() *R.y() + R.z() *R.z());

			if (dist <= maxRange)
			{
				laserData[i].dist = dist;//*1000.;
				laserDataCartArray[id]->operator[](i) = QVecToOSGVec(innerModel->laserTo(id, id, dist, laserData[i].angle));
			}
		}
		else
		{
			laserDataCartArray[id]->operator[](i) = QVecToOSGVec(innerModel->laserTo(id, id, maxRange, laserData[i].angle));
		}
		angle -= incAngle;
	}
	// the point of the laser robot
// 		laserDataCartArray[id]->operator[](measures) = QVecToOSGVec(innerModel->laserTo(id, id, 0.0001, 0.001));
// 		viewer->startThreading();
	return laserData;
}

/*

// Refills touch sensor with new values
RoboCompTouchSensor::SensorMap TOUCH_createTouchData(const IMVLaser &laser)
{
}

*/
///--- useful functions.
InnerModelNode* SpecificWorker::getNode(const QString &id, const QString &msg)
{
	guard gl(innerModel->mutex);
	InnerModelNode *node = innerModel->getNode(id);
	if (node==NULL)
	{
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::NonExistingNode;
		std::ostringstream oss;
		oss << msg.toStdString() << " error: Node " << id.toStdString() << " does not exist.";
		err.text = oss.str();
		throw err;
	}
	else
	{
		return node;
	}
}

void SpecificWorker::checkOperationInvalidNode(InnerModelNode *node,QString msg)
{
	if (node==NULL)
	{
		#ifdef INNERMODELMANAGERDEBUG
					qDebug() <<msg<<node->id<<"is not transform type";
		#endif
			RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::OperationInvalidNode;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: Node " << node->id.toStdString() <<" is not of the type require";
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::checkNodeAlreadyExists(const QString &id, const QString &msg)
{
	if (innerModel->getIDKeys().contains(id))
	{
		#ifdef INNERMODELMANAGERDEBUG
			qDebug("item already exist. %s\n", id.toStdString().c_str());
		#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::NodeAlreadyExists;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: Node " << id.toStdString() << " already exists.";
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::checkInvalidMeshValues(RoboCompInnerModelManager::meshType m, QString msg)
{
	///check Scale
	osg::Node *osgMesh = osgDB::readNodeFile(m.meshPath);
	if (m.scaleX<0.0 or m.scaleY <0.0 or m.scaleZ <0.0)
	{
#ifdef INNERMODELMANAGERDEBUG
		qDebug() <<"--- Fatal:"<<msg<<"Scale can not be negative";
		qDebug() <<"m.scaleX "<<m.scaleX<<"m.scaleY"<<m.scaleY<<"m.scaleZ"<<m.scaleZ;
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::InvalidValues;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: Scale (" << m.scaleX << ", " << m.scaleY << ", " << m.scaleZ << ") is invalid.";
		err.text = oss.str();
		throw err;
	}
	///check valid osg Node.
	else if (osgMesh==NULL)
	{
		#ifdef INNERMODELMANAGERDEBUG
				qDebug() <<"--- Fatal:"<<msg<<"meshPath:"<<QString::fromStdString(m.meshPath) <<"does not exist or no it is a type valid for his OpenSceneGraph.";
		#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::InvalidPath;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: meshPath: " << m.meshPath << ", " <<"does not exist or no it is a type valid for his OpenSceneGraph.";
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::AttributeAlreadyExists(InnerModelNode *node, QString attributeName, QString msg)
{
	if (node->attributes.contains(attributeName))
	{
		#ifdef INNERMODELMANAGERDEBUG
				qDebug("attribute already exist. %s\n", attributeName.toStdString().c_str());
		#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::AttributeAlreadyExists;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: attribute " << attributeName.toStdString() << " already exists." <<" in node "<<node->id.toStdString();
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::NonExistingAttribute(InnerModelNode *node, QString attributeName, QString msg)
{
	if (node->attributes.contains(attributeName) ==false)
	{
#ifdef INNERMODELMANAGERDEBUG
		qDebug("attribute NO exist. %s\n", attributeName.toStdString().c_str());
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::AttributeAlreadyExists;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: attribute " << attributeName.toStdString() << " NO exists."<<" in node "<<node->id.toStdString();
		err.text = oss.str();
		throw err;
	}
}

void SpecificWorker::getRecursiveNodeInformation(RoboCompInnerModelManager::NodeInformationSequence& nodesInfo, InnerModelNode *node)
{
	/// Add current node information
	RoboCompInnerModelManager::NodeInformation ni;
	ni.id = node->id.toStdString();


	if (node->parent)
	{
		ni.parentId = node->parent->id.toStdString();
	}
	else
	{
		ni.parentId = "";
	}
	ni.nType = getNodeType(node);

	RoboCompInnerModelManager::AttributeType a;
	foreach (const QString &str, node->attributes.keys())
	{
		a.type=node->attributes.value(str).type.toStdString();
		a.value=node->attributes.value(str).value.toStdString();
		ni.attributes[str.toStdString()]=a;
	}
	nodesInfo.push_back(ni);

	/// Recursive call for all children
	QList<InnerModelNode *>::iterator child;
	for (child = node->children.begin(); child != node->children.end(); child++)
	{
		getRecursiveNodeInformation(nodesInfo, *child);
	}
}

RoboCompInnerModelManager::NodeType SpecificWorker::getNodeType(InnerModelNode *node)
{
	if (dynamic_cast<InnerModelJoint*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Joint;
	}
	else if (dynamic_cast<InnerModelTouchSensor*>(node) != NULL)
	{
		return RoboCompInnerModelManager::TouchSensor;
	}
	else if (dynamic_cast<InnerModelDifferentialRobot*>(node) != NULL)
	{
		return RoboCompInnerModelManager::DifferentialRobot;
	}
	else if (dynamic_cast<InnerModelOmniRobot*>(node) != NULL)
	{
		return RoboCompInnerModelManager::OmniRobot;
	}
	else if (dynamic_cast<InnerModelPlane*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Plane;
	}
	else if (dynamic_cast<InnerModelDisplay*>(node) != NULL)
	{
		return RoboCompInnerModelManager::DisplayII;
	}
	else if (dynamic_cast<InnerModelRGBD*>(node) != NULL)
	{
		return RoboCompInnerModelManager::RGBD;
	}
	else if (dynamic_cast<InnerModelCamera*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Camera;
	}
	else if (dynamic_cast<InnerModelIMU*>(node) != NULL)
	{
		return RoboCompInnerModelManager::IMU;
	}
	else if (dynamic_cast<InnerModelLaser*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Laser;
	}
	else if (dynamic_cast<InnerModelMesh*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Mesh;
	}
	else if (dynamic_cast<InnerModelPointCloud*>(node) != NULL)
	{
		return RoboCompInnerModelManager::PointCloud;
	}
	else if (dynamic_cast<InnerModelTransform*>(node) != NULL)
	{
		return RoboCompInnerModelManager::Transform;
	}
	else
	{
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::InternalError;
		std::ostringstream oss;
		oss << "RoboCompInnerModelManager::getNodeType() error: Type of node " << node->id.toStdString() << " is unknown.";
		err.text = oss.str();
		throw err;
	}
}

// Change the color of mesh
void SpecificWorker::cambiaColor(QString id, osg::Vec4 color)
{
	osg::Node *node = imv->meshHash[id].osgmeshes;//imv->osgmeshes[id];
	node = dynamic_cast<osg::Group*>(imv->meshHash[id].osgmeshes.get())->getChild(0);
	if (node)
	{
		osg::Material *mat = new osg::Material;
		mat->setDiffuse(osg::Material::FRONT_AND_BACK, color);
		node->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::OVERRIDE);
	}
}

// Returns the initial color to a mesh
void SpecificWorker::devuelveColor(QString id)
{
	osg::Node *node = imv->meshHash[id].osgmeshes;
	node = dynamic_cast<osg::Group*>(imv->meshHash[id].osgmeshes.get())->getChild(0);
	if (node)
	{
		osg::Material *mat = new osg::Material;
		node->getOrCreateStateSet()->setAttributeAndModes(mat, osg::StateAttribute::ON);
	}
}

// Turn lights on / off
void SpecificWorker::changeLigthState(bool apagar)
{
	//QMutexLocker vm(worker->viewerMutex);
	guard gl(innerModel->mutex);
	osg::StateSet *state = viewer->getRootGroup()->getOrCreateStateSet();

	if(apagar)
	{/// turn off lights
		state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	}
	else
	{/// turn on lights
		state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
	}
}


void SpecificWorker::objectTriggered()
{
	if (actionObject->isChecked())
	{
		OBJECTWidget->show();
	}
	else
	{
		OBJECTWidget->hide();
	}
}

void SpecificWorker::visualTriggered()
{
	if (actionVisual->isChecked())
	{
		VISUALWidget->show();
	}
	else
	{
		VISUALWidget->hide();
	}
}
void SpecificWorker::featuresTriggered()
{
    if (actionFeatures->isChecked())
    {
        FEATURESWidget->show();
        comboBox_texture->hide();
    }
    else
    {
        FEATURESWidget->hide();
        comboBox_texture->hide();
    }
    printf("features\n");
}
void SpecificWorker::change_textureTriggered()
{
    if (actionFloor_Texture->isChecked())
    {
        FEATURESWidget->show();
        comboBox_texture->show();
        addobject_button->hide();
    }
    else
    {
        FEATURESWidget->hide();
        comboBox_texture->hide();
        addobject_button->hide();
    }
   // printf("features\n");
}

//void SpecificWorker :: viewTriggered()
//{
//    if (actionTree->isChecked())
//    {
//        treeWidget->show();
//    }
//    else
//    {
//        treeWidget->hide();
//    }
//    printf("features\n");
//}


// ------------------------------------------------------------------------------------------------
// Slots
// ------------------------------------------------------------------------------------------------


void SpecificWorker::setTopPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::TOP_POV);
}


void SpecificWorker::setFrontPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::FRONT_POV);
}


void SpecificWorker::setBackPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::BACK_POV);
}


void SpecificWorker::setLeftPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::LEFT_POV);
}


void SpecificWorker::setRightPOV()
{
	guard gl(innerModel->mutex);
	imv->setMainCamera(manipulator, InnerModelViewer::RIGHT_POV);
}


void SpecificWorker::closeEvent(QCloseEvent *event)
{
	event->accept();
	osg::Matrixd m = manipulator->getMatrix();
	QString s="";
	QStringList l;
	for (int i=0; i<4; i++ )
	{
		for (int j=0; j<4; j++ )
		{
			l.append(s.number(m(i,j)));
		}
	}
	settings->setValue("matrix", l);
	settings->sync();

	exit(EXIT_SUCCESS);
}

void SpecificWorker::setLigthx(double v)
{
	guard gl(innerModel->mutex);
	osg::Vec4 p= viewer->getLight()->getPosition();
	p.set(v,p.y(),p.z(),p.w());
	viewer->getLight()->setPosition(p);
}

void SpecificWorker::setLigthy(double v)
{
	guard gl(innerModel->mutex);
	osg::Vec4 p= viewer->getLight()->getPosition();
	p.set(p.x(),v,p.z(),p.w());
	viewer->getLight()->setPosition(p);
}

void SpecificWorker::setLigthz(double v)
{
	guard gl(innerModel->mutex);
	osg::Vec4 p= viewer->getLight()->getPosition();
	p.set(p.x(),p.y(),v,p.w());
	viewer->getLight()->setPosition(p);
}




// ------------------------------------------------------------------------------------------------
// ICE interfaces
// ------------------------------------------------------------------------------------------------

// void SpecificWorker::checkPoseCollision(QString node,QString msg)
//
//{
// 	///for each descendant mesh check collisions with everyone except their mesh sisters
// #ifdef INNERMODELMANAGERDEBUG
// 	qDebug() <<"checkPoseCollision"<<msg<<node<<"A?";
// #endif
// 	QStringList l;
// 	l.clear();
//
// 	innerModel->getSubTree(innerModel->getNode(node),&l);
//
// 	/// Checking
// 	foreach (QString n, l)
// 	{
// 		/// Replicate plane removals
// 		if (imv->meshHash.contains(n))
// 		{
// 			QList <QString> excludingList;
// 			excludingList.clear();
// 			detectarColision1toN(n,excludingList,msg);
//
// 		}
//
// // 		/// Replicate plane removals
// // 		if (imv->planeMts.contains(n))
// // 		{
// // 			while (imv->planeMts[n]->getNumParents() > 0)
// // 			{
// // 				((osg::Group *)(imv->planeMts[n]->getParent(0)))->removeChild(imv->planeMts[n]);
// // 			}
// // 			imv->planeMts.remove(n);
// // 			imv->planesHash.remove(n);
// // 		}
// 	}
// }
